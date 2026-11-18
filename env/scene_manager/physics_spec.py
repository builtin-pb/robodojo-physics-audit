"""Validated physics specification, resolver, and pre-parse USD authoring (#62).

Implements the D2 merged design:
- Toggleable fixes ``{order, values, size, collision, gripper, robot_limits, solver}`` with presets ``shipped``
  (default; RoboDojo's behaviour) and ``corrected`` (all seven fixes). ``gripper`` is D3's
  default (#90): the gripper command becomes the drive target under a 0.15 m/s per-finger
  slew on the previous target (no clamp to, or re-interpolation from, the measured opening),
  never below the closed stop and frozen while the drive force exceeds the per-finger effort
  threshold (arx5-sdk's anti-windup, ``GripperSpec.command_step``); arx5-sdk clips no torque at
  that threshold, and the drive is limited at the force its over-current trip maps to (#160;
  scenario ``gripper_drive_limit: motor_peak`` limits it at the motor's peak instead),
  joint7 carries the per-finger drive and the motor's reflected rotor inertia (armature) from
  ``robots.json`` for both fingers (scenario ``gripper_armature: none`` drops the armature), and joint8
  mimics joint7 through ``PhysxMimicJointAPI`` (``rotX`` token) with its own drive off.
  ``robot_limits`` (#11) authors the ARX X5 arm's joint ranges before parse and its velocity and
  effort limits in the actuator config (``ArmLimitsSpec``); scenario key ``arm_effort`` = ``shipped``
  keeps RoboDojo's 100 N*m effort limit. ``solver`` (Bo, November 13; #95, #97, #156) turns the PhysX
  scene's stabilization off and raises its position iterations 4x (``resolve_scene_physx``), authored on
  the physics scene before any body is parsed.
- Strict validation of layout declarations and parameter tables: unknown keys,
  None, bool-as-number, non-finite values (NaN/inf), negative bounds, and
  ``static_friction < dynamic_friction`` raise ``PhysicsSpecError`` at load.
  Every leaf parameter that reaches the engine defaults ``status`` to ``unsourced``
  and is gated against ``allowed_statuses`` (default ``{"sourced", "estimated"}``).
- Per-asset + class-pair JSON parameter table loader (file or directory, including
  ``#73`` ``classes.json`` + ``class_values.json``) with PhysX 5.6.1 max-priority
  combine-mode pair-band checks, solid and hollow-shell density-band checks, and
  dormant-value quarantine (layout ``friction``, ``size``, and ``mass`` are diffed
  and recorded in the manifest, never applied directly under ``values``/``size``).
- Pure-USD pre-parse authoring per object class before any wrapper constructor
  or step that triggers PhysX parsing. Note: enabling ``order`` adds ``"dynamic"``
  to ``SceneManager.spawnable_object_types`` so ``DynamicObject`` is spawned on
  episode reload rather than once in ``initialize``; the real-conveyor witness
  for the two conveyor tasks remains open (no conveyor in ``put_bottles_into_dustbin``).
- Declared dormant upstream crash guards active in all presets (both dormant across
  all 8,070 pinned evaluation layouts): ``Table.mass`` initialization when
  ``static: false`` (#50, ``table.py``) and skipping unauthored ``None`` schema
  attributes in ``ArticulationObject.fix_root_link`` (``articulation.py``).
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
import os
from typing import Any

VALID_FIXES: frozenset[str] = frozenset({"order", "values", "size", "collision", "gripper", "robot_limits", "solver"})
# G4 attribution (#62 review position 4): ``values`` is the union of two parts that may be toggled alone. ``materials``
# takes friction, restitution and combine modes from the table; ``masses`` takes rigid masses, inertias and centres of
# mass and authors them with ``UsdPhysics.MassAPI`` before PhysX parses the body in every combination, because a mass set
# after the parse keeps the parse-time inertia (#113). The parts belong to no preset; ``values`` keeps PR 2's behaviour.
VALUE_PARTS: frozenset[str] = frozenset({"materials", "masses"})
FIX_TOKENS: frozenset[str] = VALID_FIXES | VALUE_PARTS
# ``solver``: stabilization moves friction off bodies with several contact patches (#95, #97; a class in #157) and 16
# position iterations let contact-rich bodies creep at rest (#156). 16 is the count RoboDojo's rigid objects author and
# the one GPU dynamics runs (the scene runs its largest count; the X5 authors 8); the scene minimum clamps every actor
# from below, so 64 is 4x that count. Readback checks the factor against the bodies of the built scene.
SOLVER_POSITION_ITERATION_FACTOR: int = 4
SOLVER_SHIPPED_POSITION_ITERATIONS: int = 16
PRESETS: dict[str, frozenset[str]] = {
    "shipped": frozenset(),
    "corrected": VALID_FIXES,
    "declared": frozenset({"order"}),
}

COMBINE_MODES: tuple[str, ...] = ("average", "min", "multiply", "max")
COMBINE_PRIORITY: dict[str, int] = {"average": 0, "min": 1, "multiply": 2, "max": 3}
VALID_APPROXIMATIONS: frozenset[str] = frozenset(
    {"asset", "convexHull", "convexDecomposition", "sdf", "boundingCube", "boundingSphere", "meshSimplification", "none"}
)
VALID_STATUSES: frozenset[str] = frozenset({"sourced", "estimated", "scenario", "unsourced", "test_only"})
STATUS_RANK: dict[str, int] = {"unsourced": 0, "scenario": 1, "estimated": 2, "test_only": 3, "sourced": 4}

ALLOWED_DECLARATION_KEYS: dict[str, frozenset[str]] = {
    "rigid": frozenset({"type", "mass", "friction", "static_friction", "dynamic_friction", "restitution", "size"}),
    "geometry": frozenset(
        {
            "type",
            "mass",
            "friction",
            "static_friction",
            "dynamic_friction",
            "restitution",
            "size",
            "collision",
            "track_contact_forces",
            "prepare_contact_sensor",
            "disable_stablization",
            "contact_filter_prim_paths_expr",
        }
    ),
    "articulation": frozenset({"type", "mass", "friction", "fixed_base", "reset_drive", "ratio"}),
    "dynamic": frozenset({"type", "mass", "friction", "size"}),
}

ALLOWED_GROUND_MATERIAL_KEYS: frozenset[str] = frozenset(
    {
        "static_friction",
        "dynamic_friction",
        "restitution",
        "friction_combine_mode",
        "restitution_combine_mode",
        "compliant_contact_stiffness",
        "compliant_contact_damping",
    }
)
ALLOWED_TABLE_KEYS: frozenset[str] = frozenset(
    {
        "default",
        "random",
        "scale",
        "default_pos",
        "default_ori",
        "static",
        "mass",
        "mdl_path",
        "mdl_name",
        "prim_path",
        "inst_name",
    }
)
ALLOWED_ROOM_KEYS: frozenset[str] = frozenset(
    {
        "default",
        "random",
        "default_pos",
        "default_ori",
        "default_rot",
        "scale",
        "collision",
        "usd_path",
        "prim_path",
        "inst_name",
    }
)


class PhysicsSpecError(ValueError):
    """Raised when a physics declaration, parameter entry, or resolved spec is invalid."""

    def __init__(
        self,
        message: str,
        *,
        task: str | None = None,
        layout: int | str | None = None,
        env_id: int | None = None,
        prim_path: str | None = None,
        key: str | None = None,
    ) -> None:
        self.task = task
        self.layout = layout
        self.env_id = env_id
        self.prim_path = prim_path
        self.key = key
        ctx_parts = [
            f"{k}={v}"
            for k, v in (("task", task), ("layout", layout), ("env", env_id), ("prim", prim_path), ("key", key))
            if v is not None
        ]
        prefix = f"[{', '.join(ctx_parts)}] " if ctx_parts else ""
        super().__init__(f"{prefix}{message}")


class UnsourcedParameterError(PhysicsSpecError):
    """Raised when corrected mode encounters a missing, unsourced, or unreviewed parameter."""


def _to_plain(obj: Any) -> Any:
    if obj is None or isinstance(obj, (str, int, float, bool)):
        return obj
    try:
        from omegaconf import DictConfig, ListConfig, OmegaConf

        if isinstance(obj, (DictConfig, ListConfig)):
            return OmegaConf.to_container(obj, resolve=True)
    except Exception:
        pass
    if isinstance(obj, Mapping):
        return {str(k): _to_plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_plain(v) for v in obj]
    return obj


def _require_finite_float(val: Any, *, name: str, **ctx: Any) -> float:
    if val is None or isinstance(val, bool) or not isinstance(val, (int, float)):
        raise PhysicsSpecError(f"Expected finite number for '{name}', got {val!r}", key=name, **ctx)
    fval = float(val)
    if not math.isfinite(fval):
        raise PhysicsSpecError(f"Non-finite number for '{name}': {val!r}", key=name, **ctx)
    return fval


def _require_bool(val: Any, *, name: str, **ctx: Any) -> bool:
    if not isinstance(val, bool):
        raise PhysicsSpecError(f"Expected bool for '{name}', got {val!r}", key=name, **ctx)
    return val


def _require_vec3_pos(val: Any, *, name: str, allow_zero: bool = False, **ctx: Any) -> tuple[float, float, float]:
    if not isinstance(val, Sequence) or isinstance(val, (str, bytes)) or len(val) != 3:
        raise PhysicsSpecError(f"Expected 3-element vector for '{name}', got {val!r}", key=name, **ctx)
    out = tuple(_require_finite_float(x, name=name, **ctx) for x in val)
    for x in out:
        if (x < 0.0) if allow_zero else (x <= 0.0):
            raise PhysicsSpecError(f"Expected positive components for '{name}', got {out!r}", key=name, **ctx)
    return (out[0], out[1], out[2])


def resolve_fixes(
    preset: str | None = None,
    fixes: Iterable[str] | str | None = None,
) -> tuple[str, frozenset[str]]:
    """Resolve ``(preset_name, active_fixes)`` and reject unknown preset/fix names."""
    preset_name = "shipped" if (preset is None or str(preset).strip() == "") else str(preset).strip()
    if preset_name not in PRESETS:
        raise PhysicsSpecError(
            f"Unknown physics preset {preset_name!r}; expected one of {sorted(PRESETS)}",
            key="physics_preset",
        )
    if fixes is None:
        return preset_name, PRESETS[preset_name]
    if isinstance(fixes, str):
        s = fixes.strip()
        if s == "" or s.lower() == "none":
            tokens: list[str] = []
        elif s in PRESETS:
            return s, PRESETS[s]
        else:
            tokens = [t.strip() for t in s.split(",") if t.strip()]
    elif isinstance(fixes, Iterable):
        tokens = [str(t).strip() for t in fixes if str(t).strip()]
    else:
        raise PhysicsSpecError(f"Invalid physics_fixes value: {fixes!r}", key="physics_fixes")

    if tokens and all(t.startswith(("+", "-")) for t in tokens):
        cur = set(PRESETS[preset_name])
        for tok in tokens:
            name = tok[1:]
            if name not in FIX_TOKENS:
                raise PhysicsSpecError(
                    f"Unknown physics fix {name!r}; expected subset of {sorted(FIX_TOKENS)}",
                    key="physics_fixes",
                )
            if tok[0] == "+":
                cur.add(name)
            elif name in VALUE_PARTS and "values" in cur:
                # ``values`` is the union of its parts: subtracting one leaves the other.
                cur.discard("values")
                cur.update(VALUE_PARTS - {name})
            elif name == "values":
                cur.difference_update({"values"} | VALUE_PARTS)
            else:
                cur.discard(name)
        return preset_name, frozenset(cur)

    unknown = [t for t in tokens if t not in FIX_TOKENS]
    if unknown:
        raise PhysicsSpecError(
            f"Unknown physics fix(es) {unknown!r}; expected subset of {sorted(FIX_TOKENS)}",
            key="physics_fixes",
        )
    return preset_name, frozenset(tokens)


def validate_cli_dormant_gate(
    preset: str | None = None,
    fixes: Iterable[str] | str | None = None,
    *,
    allow_dormant: bool = False,
) -> tuple[str, frozenset[str]]:
    """Refuse diagnostic presets/fix sets that enable ``order`` without table materials unless ``allow_dormant=True``."""
    preset_name, active_fixes = resolve_fixes(preset, fixes)
    no_table_materials = not (active_fixes & {"values", "materials"})
    if (preset_name == "declared" or ("order" in active_fixes and no_table_materials)) and not allow_dormant:
        raise PhysicsSpecError(
            f"Refusing physics configuration (preset={preset_name!r}, fixes={sorted(active_fixes)}) with 'order' enabled "
            "without 'values': this activates dormant layout values (e.g. 10,000 and 2.0 friction) and is a diagnostic "
            "ablation, not a reported benchmark arm (#62 D2). Pass --allow_dormant_physics_values to run it explicitly.",
            key="physics_preset",
        )
    return preset_name, active_fixes


def combine_values(
    v_a: float,
    v_b: float,
    mode_a: str | None = "average",
    mode_b: str | None = "average",
) -> tuple[float, str]:
    """PhysX 5.6.1 ``PxCombineMode`` rule: max-priority mode wins (average < min < multiply < max)."""
    ma = mode_a or "average"
    mb = mode_b or "average"
    if ma not in COMBINE_PRIORITY or mb not in COMBINE_PRIORITY:
        raise PhysicsSpecError(f"Invalid combine modes ({ma!r}, {mb!r})", key="combine_mode")
    eff_mode = ma if COMBINE_PRIORITY[ma] >= COMBINE_PRIORITY[mb] else mb
    if eff_mode == "average":
        return 0.5 * (v_a + v_b), eff_mode
    if eff_mode == "min":
        return min(v_a, v_b), eff_mode
    if eff_mode == "multiply":
        return v_a * v_b, eff_mode
    return max(v_a, v_b), eff_mode


@dataclass(frozen=True)
class MaterialSpec:
    static_friction: float
    dynamic_friction: float
    restitution: float = 0.0
    friction_combine_mode: str | None = "average"
    restitution_combine_mode: str | None = "average"
    compliant_contact_stiffness: float | None = None
    compliant_contact_damping: float | None = None
    allow_legacy_inversion: bool = False

    def __post_init__(self) -> None:
        sf = _require_finite_float(self.static_friction, name="static_friction")
        df = _require_finite_float(self.dynamic_friction, name="dynamic_friction")
        rest = _require_finite_float(self.restitution, name="restitution")
        if sf < 0.0:
            raise PhysicsSpecError(f"static_friction must be >= 0, got {sf}", key="static_friction")
        if df < 0.0:
            raise PhysicsSpecError(f"dynamic_friction must be >= 0, got {df}", key="dynamic_friction")
        if not (0.0 <= rest <= 1.0):
            raise PhysicsSpecError(f"restitution must be in [0, 1], got {rest}", key="restitution")
        if not self.allow_legacy_inversion and sf + 1e-12 < df:
            raise PhysicsSpecError(
                f"static_friction ({sf}) < dynamic_friction ({df}); PhysX silently clamps static to dynamic (#2)",
                key="static_friction",
            )
        for attr_name, mode in (
            ("friction_combine_mode", self.friction_combine_mode),
            ("restitution_combine_mode", self.restitution_combine_mode),
        ):
            if mode is None:
                if not self.allow_legacy_inversion:
                    raise PhysicsSpecError(f"{attr_name} must not be None in validated MaterialSpec", key=attr_name)
            elif mode not in COMBINE_PRIORITY:
                raise PhysicsSpecError(
                    f"Invalid {attr_name}={mode!r}; expected one of {COMBINE_MODES}",
                    key=attr_name,
                )
        if self.compliant_contact_stiffness is not None:
            cs = _require_finite_float(self.compliant_contact_stiffness, name="compliant_contact_stiffness")
            if cs < 0.0:
                raise PhysicsSpecError("compliant_contact_stiffness must be >= 0", key="compliant_contact_stiffness")
            object.__setattr__(self, "compliant_contact_stiffness", cs)
        if self.compliant_contact_damping is not None:
            cd = _require_finite_float(self.compliant_contact_damping, name="compliant_contact_damping")
            if cd < 0.0:
                raise PhysicsSpecError("compliant_contact_damping must be >= 0", key="compliant_contact_damping")
            object.__setattr__(self, "compliant_contact_damping", cd)
        object.__setattr__(self, "static_friction", sf)
        object.__setattr__(self, "dynamic_friction", df)
        object.__setattr__(self, "restitution", rest)

    @property
    def effective_static_friction(self) -> float:
        return max(self.static_friction, self.dynamic_friction)


@dataclass(frozen=True)
class CollisionSpec:
    enabled: bool | None = None
    approximation: str = "asset"
    contact_offset_m: float | None = None
    rest_offset_m: float | None = None
    torsional_patch_radius_m: float | None = None
    min_torsional_patch_radius_m: float | None = None

    def __post_init__(self) -> None:
        if self.enabled is not None:
            _require_bool(self.enabled, name="collision.enabled")
        if not isinstance(self.approximation, str) or self.approximation not in VALID_APPROXIMATIONS:
            raise PhysicsSpecError(
                f"Invalid collision approximation {self.approximation!r}; expected one of {sorted(VALID_APPROXIMATIONS)}",
                key="collision.approximation",
            )
        if self.contact_offset_m is not None:
            co = _require_finite_float(self.contact_offset_m, name="contact_offset_m")
            if co <= 0.0:
                raise PhysicsSpecError(f"contact_offset_m must be > 0, got {co}", key="contact_offset_m")
            object.__setattr__(self, "contact_offset_m", co)
        if self.rest_offset_m is not None:
            ro = _require_finite_float(self.rest_offset_m, name="rest_offset_m")
            if self.contact_offset_m is not None and ro > self.contact_offset_m:
                raise PhysicsSpecError("rest_offset_m must be <= contact_offset_m", key="rest_offset_m")
            object.__setattr__(self, "rest_offset_m", ro)
        for rname in ("torsional_patch_radius_m", "min_torsional_patch_radius_m"):
            rv = getattr(self, rname)
            if rv is not None:
                rf = _require_finite_float(rv, name=rname)
                if rf < 0.0:
                    raise PhysicsSpecError(f"{rname} must be >= 0, got {rf}", key=rname)
                object.__setattr__(self, rname, rf)
        if (
            self.torsional_patch_radius_m is not None
            and self.min_torsional_patch_radius_m is not None
            and self.min_torsional_patch_radius_m > self.torsional_patch_radius_m
        ):
            raise PhysicsSpecError(
                "min_torsional_patch_radius_m must be <= torsional_patch_radius_m",
                key="min_torsional_patch_radius_m",
            )


@dataclass(frozen=True)
class DriveSpec:
    stiffness: float
    damping: float
    max_force: float | None = None
    max_velocity: float | None = None
    drive_type: str = "angular"

    def __post_init__(self) -> None:
        st = _require_finite_float(self.stiffness, name="stiffness")
        da = _require_finite_float(self.damping, name="damping")
        if st < 0.0 or da < 0.0:
            raise PhysicsSpecError("Drive stiffness and damping must be >= 0", key="drive")
        if not isinstance(self.drive_type, str) or self.drive_type not in ("angular", "linear"):
            raise PhysicsSpecError(f"Invalid drive_type {self.drive_type!r}", key="drive_type")
        object.__setattr__(self, "stiffness", st)
        object.__setattr__(self, "damping", da)
        if self.max_force is not None:
            mf = _require_finite_float(self.max_force, name="max_force")
            if mf <= 0.0:
                raise PhysicsSpecError("Drive max_force must be > 0", key="max_force")
            object.__setattr__(self, "max_force", mf)
        if self.max_velocity is not None:
            mv = _require_finite_float(self.max_velocity, name="max_velocity")
            if mv <= 0.0:
                raise PhysicsSpecError("Drive max_velocity must be > 0", key="max_velocity")
            object.__setattr__(self, "max_velocity", mv)


@dataclass(frozen=True)
class InertialSpec:
    mass_kg: float
    com_offset_m: tuple[float, float, float] | None = None
    inertia_kg_m2: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        m = _require_finite_float(self.mass_kg, name="mass_kg")
        if m <= 0.0:
            raise PhysicsSpecError(f"mass_kg must be > 0, got {m}", key="mass_kg")
        object.__setattr__(self, "mass_kg", m)
        if self.com_offset_m is not None:
            if not isinstance(self.com_offset_m, Sequence) or isinstance(self.com_offset_m, (str, bytes)) or len(self.com_offset_m) != 3:
                raise PhysicsSpecError(f"com_offset_m must be a 3-element sequence, got {self.com_offset_m!r}", key="com_offset_m")
            c = tuple(_require_finite_float(x, name="com_offset_m") for x in self.com_offset_m)
            object.__setattr__(self, "com_offset_m", (c[0], c[1], c[2]))
        if self.inertia_kg_m2 is not None:
            iner = _require_vec3_pos(self.inertia_kg_m2, name="inertia_kg_m2")
            object.__setattr__(self, "inertia_kg_m2", iner)


GRIPPER_EFFORT_CASES: tuple[str, ...] = ("nominal", "low", "high")
GRIPPER_ARMATURE_CASES: tuple[str, ...] = ("nominal", "none")
# #160 (Bo, November 14): the drive's engine limit. "trip": the force arx5-sdk's over-current trip maps to (default);
# "motor_peak": the SDK's law uncapped up to the motor's peak (sensitivity).
GRIPPER_DRIVE_LIMIT_CASES: tuple[str, ...] = ("trip", "motor_peak")


@dataclass(frozen=True)
class GripperSpec:
    """D3 (#90) gripper: per-finger drive values, command slew, and the joint8 -> joint7 mimic.

    joint8 follows joint7 through a hard mimic constraint (PhysX applies the constraint impulse to the
    reference joint as well), so joint7's drive carries both fingers' load: the engine values on joint7
    are twice the per-finger values (virtual work, as for one motor driving a symmetric rack,
    ``audit/out/32/phase1.md`` A.4), and joint8's own drive is off.
    """

    robot_name: str
    driven_joint: str
    mimic_joint: str
    finger_stiffness_n_m: float
    finger_damping_n_s_m: float
    finger_effort_n: float
    effort_case: str
    slew_m_s: float
    mimic_gearing: float
    mimic_offset: float
    mimic_axis: str = "rotX"
    provenance: str = "unsourced"
    finger_armature_kg: float = 0.0
    armature_case: str = "nominal"
    # #160: the drive's engine effort limit per finger. Under ``drive_limit_case`` "trip" it is the force arx5-sdk's
    # over-current trip maps to, so the simulation never sustains a grip that controller would trip on; under
    # "motor_peak" it is the motor's peak. Both values are required: a spec without them must not fall back to D3's
    # first form (the limit at the freeze threshold, under which #160's grip collapsed).
    finger_trip_n: float | None = None
    finger_motor_peak_n: float | None = None
    drive_limit_case: str = "trip"

    def __post_init__(self) -> None:
        for name in ("robot_name", "driven_joint", "mimic_joint"):
            v = getattr(self, name)
            if not isinstance(v, str) or not v.strip():
                raise PhysicsSpecError(f"GripperSpec.{name} must be a non-empty string, got {v!r}", key=name)
        if self.driven_joint == self.mimic_joint:
            raise PhysicsSpecError("GripperSpec.driven_joint and mimic_joint must differ", key="mimic_joint")
        for name in ("finger_stiffness_n_m", "finger_damping_n_s_m", "finger_effort_n", "slew_m_s"):
            v = _require_finite_float(getattr(self, name), name=name)
            if v <= 0.0:
                raise PhysicsSpecError(f"GripperSpec.{name} must be > 0, got {v}", key=name)
            object.__setattr__(self, name, v)
        if self.effort_case not in GRIPPER_EFFORT_CASES:
            raise PhysicsSpecError(
                f"GripperSpec.effort_case must be one of {list(GRIPPER_EFFORT_CASES)}, got {self.effort_case!r}",
                key="effort_case",
            )
        g = _require_finite_float(self.mimic_gearing, name="mimic_gearing")
        if abs(abs(g) - 1.0) > 1e-9:
            # Equal finger forces (and the factor 2 on joint7) hold only for a symmetric gripper.
            raise PhysicsSpecError(f"GripperSpec.mimic_gearing must be +-1 (symmetric fingers), got {g}", key="mimic_gearing")
        object.__setattr__(self, "mimic_gearing", g)
        object.__setattr__(self, "mimic_offset", _require_finite_float(self.mimic_offset, name="mimic_offset"))
        # The X5's fingers share one coordinate convention (both in [0, open]), so the only valid coupling is
        # q_mimic = q_driven: PhysX gearing -1, offset 0. Anything else (e.g. +1 with an offset) pins the
        # fingers to a single opening while still passing a reset-time readback (#105 GPT review, item 5).
        if g != -1.0 or self.mimic_offset != 0.0:
            raise PhysicsSpecError(
                f"GripperSpec mimic must be gearing -1, offset 0 for the X5's same-convention fingers, "
                f"got gearing {g}, offset {self.mimic_offset}",
                key="mimic_gearing",
            )
        if self.mimic_axis not in ("rotX", "rotY", "rotZ"):
            raise PhysicsSpecError(f"GripperSpec.mimic_axis must be rotX/rotY/rotZ, got {self.mimic_axis!r}", key="mimic_axis")
        a = _require_finite_float(self.finger_armature_kg, name="finger_armature_kg")
        if a < 0.0 or self.armature_case not in GRIPPER_ARMATURE_CASES or (a == 0.0) != (self.armature_case == "none"):
            raise PhysicsSpecError(
                f"GripperSpec.finger_armature_kg {a} does not match armature_case {self.armature_case!r}", key="finger_armature_kg"
            )
        object.__setattr__(self, "finger_armature_kg", a)
        for name in ("finger_trip_n", "finger_motor_peak_n"):
            if getattr(self, name) is None:
                raise PhysicsSpecError(f"GripperSpec.{name} is required (#160)", key=name)
            object.__setattr__(self, name, _require_finite_float(getattr(self, name), name=name))
        if not self.finger_effort_n < self.finger_trip_n <= self.finger_motor_peak_n:
            raise PhysicsSpecError(
                f"GripperSpec needs threshold < trip <= motor peak per finger, got {self.finger_effort_n}, "
                f"{self.finger_trip_n}, {self.finger_motor_peak_n}",
                key="finger_trip_n",
            )
        if self.drive_limit_case not in GRIPPER_DRIVE_LIMIT_CASES:
            raise PhysicsSpecError(
                f"GripperSpec.drive_limit_case must be one of {list(GRIPPER_DRIVE_LIMIT_CASES)}, got {self.drive_limit_case!r}",
                key="drive_limit_case",
            )

    @property
    def drive_stiffness(self) -> float:
        return 2.0 * self.finger_stiffness_n_m

    @property
    def drive_damping(self) -> float:
        return 2.0 * self.finger_damping_n_s_m

    @property
    def freeze_threshold(self) -> float:
        """joint7 drive force above which the command stops advancing (arx5-sdk ``gripper_torque_max / 2``)."""
        return 2.0 * self.finger_effort_n

    @property
    def drive_effort_limit(self) -> float:
        """joint7's engine effort limit (#160): the trip's force by default, the motor's peak under ``motor_peak``."""
        return 2.0 * (self.finger_trip_n if self.drive_limit_case == "trip" else self.finger_motor_peak_n)

    @property
    def drive_armature(self) -> float:
        return 2.0 * self.finger_armature_kg

    def command_step(
        self, prev_target: float, command: float, q: float, qdot: float, dt: float, upper: float
    ) -> tuple[float, bool, float]:
        """One control step of the finger target, in the order arx5-sdk's ``update_output_cmd_`` applies it.

        arx5-sdk ``c160db7`` ``src/app/controller_base.cpp``:
        - L528-540: move the previous output toward the command by at most ``gripper_vel_max * dt``;
        - L548-561: clip the result to ``[0, gripper_width]``, so the target never passes fully closed;
        - L562-578: while the measured gripper torque exceeds the freeze threshold, keep the previous output
          if this step would move further in the blocked direction (closing against a held object).

        Per finger: 0 is the closed stop (x5 joint7 lower limit) and ``upper`` the open stop. The threshold is
        ``freeze_threshold`` (D3's 80 N per finger nominal). The measured torque is joint7's drive force from the
        engine state, ``K7 * (prev_target - q) - c7 * qdot``. The SDK clips no torque at the threshold, so the
        steady grip is ``K7`` times the lead at which the target froze, up to the engine limit
        (``drive_effort_limit``, #160: the force of the SDK's over-current trip by default). Returns ``(target,
        frozen, joint7 drive force in N)``.
        """
        delta = command - prev_target
        max_step = self.slew_m_s * dt
        target = prev_target + math.copysign(max_step, delta) if abs(delta) > max_step else command
        target = min(max(target, 0.0), upper)
        force = self.drive_stiffness * (prev_target - q) - self.drive_damping * qdot
        frozen = False
        if abs(force) > self.freeze_threshold:
            blocked = 1.0 if force > 0.0 else -1.0  # +1: opening blocked, -1: closing blocked (q grows opening)
            if (target - prev_target) * blocked > 0.0:
                target, frozen = prev_target, True
        return target, frozen, force

    def manifest(self) -> dict[str, Any]:
        out = asdict(self)
        out["engine"] = {
            self.driven_joint: {
                "stiffness": self.drive_stiffness,
                "damping": self.drive_damping,
                "max_force": self.drive_effort_limit,
                "armature": self.drive_armature,
                "freeze_threshold": self.freeze_threshold,
            },
            self.mimic_joint: {"stiffness": 0.0, "damping": 0.0},
        }
        out["command_law"] = (
            "Per control step (arx5-sdk c160db7 controller_base.cpp L528-578): slew from the previous target, clip to "
            "[0, open], keep the previous target while joint7's drive force exceeds the freeze threshold (2 x the "
            "per-finger effort) in the direction of motion. The engine limits the drive at the force arx5-sdk's "
            "over-current trip maps to (L380-392; drive_limit_case trip) or at the motor's peak (motor_peak), #160; "
            "the trip's emergency path is not modelled."
        )
        out["observation_note"] = (
            "The observed gripper state is the last target (RoboDojo records prev_control). Under this fix a held "
            "close reads where the target froze at the freeze threshold (about the free-closing lag inside the contact "
            "point, never below 0), and a full close in air reads 0; shipped read measured - 10.8 mm."
        )
        return out


def split_gripper_actuators(actuators: Mapping[str, Any], spec: GripperSpec) -> dict[str, Any]:
    """Return a copy of an IsaacLab ``actuators`` mapping with the gripper joints driven per ``spec``.

    The actuator config naming both gripper joints (x5: ``gripper`` on ``["joint7", "joint8"]``) is replaced by
    ``gripper`` on the driven joint (2x per-finger stiffness, damping and engine effort limit, the latter per
    ``drive_limit_case``, #160) and ``gripper_mimic`` on the mimic joint (stiffness and damping 0). Every other
    field, including ``velocity_limit_sim``, stays as shipped.
    """
    joints = {spec.driven_joint, spec.mimic_joint}
    matches = [k for k, cfg in actuators.items() if joints & set(getattr(cfg, "joint_names_expr", None) or ())]
    if len(matches) != 1 or set(actuators[matches[0]].joint_names_expr) != joints:
        raise PhysicsSpecError(
            f"Expected exactly one actuator naming {sorted(joints)}, found {matches!r}",
            key=f"robots.{spec.robot_name}.gripper",
        )
    out = {k: v for k, v in actuators.items() if k != matches[0]}
    base = actuators[matches[0]]
    kw = {"armature": spec.drive_armature} if spec.drive_armature > 0.0 else {}
    out["gripper"] = base.replace(
        joint_names_expr=[spec.driven_joint],
        stiffness=spec.drive_stiffness,
        damping=spec.drive_damping,
        effort_limit_sim=spec.drive_effort_limit,
        **kw,
    )
    out["gripper_mimic"] = base.replace(joint_names_expr=[spec.mimic_joint], stiffness=0.0, damping=0.0)
    return out


ARM_EFFORT_CASES: tuple[str, ...] = ("nominal", "shipped")


@dataclass(frozen=True)
class ArmLimitsSpec:
    """#11 arm limits: per-joint range (rad), velocity limit (rad/s) and effort limit (N*m, or None to keep shipped)."""

    robot_name: str
    joints: tuple[str, ...]
    lower_rad: tuple[float, ...]
    upper_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    effort_nm: tuple[float, ...] | None
    effort_case: str = "nominal"
    provenance: str = "unsourced"

    def __post_init__(self) -> None:
        n = len(self.joints)
        if n == 0 or not all(isinstance(j, str) and j.strip() for j in self.joints) or len(set(self.joints)) != n:
            raise PhysicsSpecError(f"ArmLimitsSpec.joints must be distinct non-empty names, got {self.joints!r}", key="joints")
        object.__setattr__(self, "joints", tuple(self.joints))
        for name in ("lower_rad", "upper_rad", "velocity_rad_s", "effort_nm"):
            v = getattr(self, name)
            if v is None and name == "effort_nm":
                continue
            if not isinstance(v, Sequence) or isinstance(v, (str, bytes)) or len(v) != n:
                raise PhysicsSpecError(f"ArmLimitsSpec.{name} must have {n} entries, got {v!r}", key=name)
            vals = tuple(_require_finite_float(x, name=name) for x in v)
            if name in ("velocity_rad_s", "effort_nm") and any(x <= 0.0 for x in vals):
                raise PhysicsSpecError(f"ArmLimitsSpec.{name} must be > 0, got {vals}", key=name)
            object.__setattr__(self, name, vals)
        if any(lo >= hi for lo, hi in zip(self.lower_rad, self.upper_rad)):
            raise PhysicsSpecError(f"ArmLimitsSpec lower_rad must be below upper_rad, got {self.lower_rad} / {self.upper_rad}", key="lower_rad")
        if (self.effort_nm is None) != (self.effort_case == "shipped") or self.effort_case not in ARM_EFFORT_CASES:
            raise PhysicsSpecError(f"ArmLimitsSpec.effort_case {self.effort_case!r} does not match effort_nm", key="effort_case")

    def manifest(self) -> dict[str, Any]:
        return {k: list(v) if isinstance(v, tuple) else v for k, v in asdict(self).items()}


def apply_arm_limits_to_actuators(actuators: Mapping[str, Any], spec: ArmLimitsSpec) -> dict[str, Any]:
    """Return a copy of an IsaacLab ``actuators`` mapping whose arm actuator carries ``spec``'s per-joint limits.

    The actuator naming the arm joints (x5: ``arm`` on ``joint[1-6]``) gets per-joint ``velocity_limit_sim`` and,
    unless the effort case is ``shipped``, per-joint ``effort_limit_sim``; IsaacLab writes both to PhysX when the
    articulation is created. Stiffness, damping and armature stay as shipped. Ranges are authored in USD instead
    (``author_robot_physics``).
    """
    import re

    def names(cfg: Any) -> list[str]:
        exprs = getattr(cfg, "joint_names_expr", None) or ()
        return [j for j in spec.joints if any(re.fullmatch(e, j) for e in exprs)]

    matches = [k for k, cfg in actuators.items() if names(cfg)]
    if len(matches) != 1 or names(actuators[matches[0]]) != list(spec.joints):
        raise PhysicsSpecError(
            f"Expected exactly one actuator naming all of {list(spec.joints)}, found {matches!r}",
            key=f"robots.{spec.robot_name}.arm_limits",
        )
    out = dict(actuators)
    kw: dict[str, Any] = {"velocity_limit_sim": {j: v for j, v in zip(spec.joints, spec.velocity_rad_s)}}
    if spec.effort_nm is not None:
        kw["effort_limit_sim"] = {j: e for j, e in zip(spec.joints, spec.effort_nm)}
    out[matches[0]] = actuators[matches[0]].replace(**kw)
    return out


@dataclass(frozen=True)
class RigidSpec:
    asset_id: str
    mass_kg: float
    scale: tuple[float, float, float]
    expected_extents_m: tuple[float, float, float] | None
    material: MaterialSpec
    collision: CollisionSpec
    com_offset_m: tuple[float, float, float] | None = None
    inertia_kg_m2: tuple[float, float, float] | None = None
    solid_collider_inertia_over_mass_m2: tuple[float, float, float] | None = None
    provenance: str = "shipped"
    author_pre_parse: bool = False
    author_usd_mass: bool = False
    author_collision: bool = False

    def __post_init__(self) -> None:
        m = _require_finite_float(self.mass_kg, name="mass_kg")
        if m <= 0.0:
            raise PhysicsSpecError(f"RigidSpec.mass_kg must be > 0, got {m}", key="mass_kg")
        object.__setattr__(self, "mass_kg", m)
        object.__setattr__(self, "scale", _require_vec3_pos(self.scale, name="scale"))
        if self.expected_extents_m is not None:
            object.__setattr__(
                self, "expected_extents_m", _require_vec3_pos(self.expected_extents_m, name="expected_extents_m")
            )
        if self.com_offset_m is not None:
            if not isinstance(self.com_offset_m, Sequence) or isinstance(self.com_offset_m, (str, bytes)) or len(self.com_offset_m) != 3:
                raise PhysicsSpecError(f"RigidSpec.com_offset_m must be 3 floats, got {self.com_offset_m!r}", key="com_offset_m")
            c = tuple(_require_finite_float(x, name="com_offset_m") for x in self.com_offset_m)
            object.__setattr__(self, "com_offset_m", (c[0], c[1], c[2]))
        if self.inertia_kg_m2 is not None:
            object.__setattr__(self, "inertia_kg_m2", _require_vec3_pos(self.inertia_kg_m2, name="inertia_kg_m2"))
        if self.solid_collider_inertia_over_mass_m2 is not None:
            object.__setattr__(
                self,
                "solid_collider_inertia_over_mass_m2",
                _require_vec3_pos(self.solid_collider_inertia_over_mass_m2, name="solid_collider_inertia_over_mass_m2"),
            )


@dataclass(frozen=True)
class StaticSpec:
    asset_id: str
    scale: tuple[float, float, float]
    expected_extents_m: tuple[float, float, float] | None
    material: MaterialSpec
    collision: CollisionSpec
    bind_material: bool = False
    author_collision: bool = False
    provenance: str = "shipped"

    def __post_init__(self) -> None:
        object.__setattr__(self, "scale", _require_vec3_pos(self.scale, name="scale"))
        if self.expected_extents_m is not None:
            object.__setattr__(
                self, "expected_extents_m", _require_vec3_pos(self.expected_extents_m, name="expected_extents_m")
            )


@dataclass(frozen=True)
class TableSpec:
    asset_id: str
    is_static: bool
    scale: tuple[float, float, float]
    expected_extents_m: tuple[float, float, float]
    material: MaterialSpec
    collision: CollisionSpec
    mass_kg: float | None = None
    bind_pre_parse: bool = False
    author_collision: bool = False
    provenance: str = "shipped"

    def __post_init__(self) -> None:
        _require_bool(self.is_static, name="is_static")
        object.__setattr__(self, "scale", _require_vec3_pos(self.scale, name="scale"))
        object.__setattr__(
            self, "expected_extents_m", _require_vec3_pos(self.expected_extents_m, name="expected_extents_m")
        )
        if self.is_static and self.mass_kg is not None:
            raise PhysicsSpecError("mass_kg is forbidden on static TableSpec", key="mass_kg")
        if not self.is_static:
            if self.mass_kg is None:
                raise PhysicsSpecError("Dynamic TableSpec requires mass_kg (#50)", key="mass_kg")
            m = _require_finite_float(self.mass_kg, name="mass_kg")
            if m <= 0.0:
                raise PhysicsSpecError(f"Dynamic TableSpec mass_kg must be > 0, got {m}", key="mass_kg")
            object.__setattr__(self, "mass_kg", m)


@dataclass(frozen=True)
class ArticulationSpec:
    asset_id: str
    fixed_base: bool
    reset_drive: bool
    scale: tuple[float, float, float]
    link_masses_kg: dict[str, float] = field(default_factory=dict)
    link_materials: dict[str, MaterialSpec] = field(default_factory=dict)
    joint_drives: dict[str, DriveSpec] = field(default_factory=dict)
    default_material: MaterialSpec | None = None
    collision: CollisionSpec = field(default_factory=CollisionSpec)
    author_pre_parse: bool = False
    author_collision: bool = False
    provenance: str = "shipped"

    def __post_init__(self) -> None:
        _require_bool(self.fixed_base, name="fixed_base")
        _require_bool(self.reset_drive, name="reset_drive")
        object.__setattr__(self, "scale", _require_vec3_pos(self.scale, name="scale"))
        clean_masses: dict[str, float] = {}
        for k, v in (self.link_masses_kg or {}).items():
            mf = _require_finite_float(v, name=f"link_masses_kg.{k}")
            if mf <= 0.0:
                raise PhysicsSpecError(f"Articulation link mass must be > 0 for {k}", key=f"link_masses_kg.{k}")
            clean_masses[str(k)] = mf
        object.__setattr__(self, "link_masses_kg", clean_masses)


@dataclass(frozen=True)
class DynamicSpec:
    asset_id: str
    scale: tuple[float, float, float]
    expected_extents_m: tuple[float, float, float] | None
    belt_velocity_m_s: tuple[float, float, float] | None
    material: MaterialSpec | None
    collision: CollisionSpec = field(default_factory=CollisionSpec)
    author_pre_parse: bool = False
    author_collision: bool = False
    provenance: str = "shipped"

    def __post_init__(self) -> None:
        object.__setattr__(self, "scale", _require_vec3_pos(self.scale, name="scale"))
        if self.expected_extents_m is not None:
            object.__setattr__(
                self, "expected_extents_m", _require_vec3_pos(self.expected_extents_m, name="expected_extents_m")
            )
        if self.belt_velocity_m_s is not None:
            if not isinstance(self.belt_velocity_m_s, Sequence) or isinstance(self.belt_velocity_m_s, (str, bytes)) or len(self.belt_velocity_m_s) != 3:
                raise PhysicsSpecError(f"belt_velocity_m_s must be a 3-element sequence, got {self.belt_velocity_m_s!r}", key="belt_velocity_m_s")
            bv = tuple(_require_finite_float(x, name="belt_velocity_m_s") for x in self.belt_velocity_m_s)
            object.__setattr__(self, "belt_velocity_m_s", (bv[0], bv[1], bv[2]))


@dataclass(frozen=True)
class RobotSpec:
    robot_name: str
    deinstance_fingers: bool = False
    sanitize_camera_inertials: bool = False
    pad_links: tuple[str, ...] = ()
    link_materials: dict[str, MaterialSpec] = field(default_factory=dict)
    link_inertials: dict[str, InertialSpec] = field(default_factory=dict)
    pad_torsional_radius_m: float | None = None
    pad_min_torsional_radius_m: float | None = None
    provenance: str = "shipped"
    gripper: GripperSpec | None = None
    arm_limits: ArmLimitsSpec | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.robot_name, str) or not self.robot_name.strip():
            raise PhysicsSpecError(f"RobotSpec.robot_name must be a non-empty string, got {self.robot_name!r}", key="robot_name")
        if self.gripper is not None and not isinstance(self.gripper, GripperSpec):
            raise PhysicsSpecError(f"RobotSpec.gripper must be a GripperSpec or None, got {self.gripper!r}", key="gripper")
        if self.arm_limits is not None and not isinstance(self.arm_limits, ArmLimitsSpec):
            raise PhysicsSpecError(f"RobotSpec.arm_limits must be an ArmLimitsSpec or None, got {self.arm_limits!r}", key="arm_limits")
        _require_bool(self.deinstance_fingers, name="deinstance_fingers")
        _require_bool(self.sanitize_camera_inertials, name="sanitize_camera_inertials")
        if not isinstance(self.pad_links, Sequence) or isinstance(self.pad_links, (str, bytes)) or not all(
            isinstance(x, str) and x.strip() for x in self.pad_links
        ):
            raise PhysicsSpecError(f"RobotSpec.pad_links must be a sequence of non-empty link names, got {self.pad_links!r}", key="pad_links")
        object.__setattr__(self, "pad_links", tuple(self.pad_links))
        if not isinstance(self.link_materials, Mapping) or not all(
            isinstance(k, str) and isinstance(v, MaterialSpec) for k, v in self.link_materials.items()
        ):
            raise PhysicsSpecError("RobotSpec.link_materials must map link names to MaterialSpec", key="link_materials")
        if not isinstance(self.link_inertials, Mapping) or not all(
            isinstance(k, str) and isinstance(v, InertialSpec) for k, v in self.link_inertials.items()
        ):
            raise PhysicsSpecError("RobotSpec.link_inertials must map link names to InertialSpec", key="link_inertials")
        object.__setattr__(self, "link_materials", dict(self.link_materials))
        object.__setattr__(self, "link_inertials", dict(self.link_inertials))
        for rname in ("pad_torsional_radius_m", "pad_min_torsional_radius_m"):
            rv = getattr(self, rname)
            if rv is not None:
                rf = _require_finite_float(rv, name=rname)
                if rf < 0.0:
                    raise PhysicsSpecError(f"RobotSpec.{rname} must be >= 0, got {rf}", key=rname)
                object.__setattr__(self, rname, rf)
        if (
            self.pad_torsional_radius_m is not None
            and self.pad_min_torsional_radius_m is not None
            and self.pad_min_torsional_radius_m > self.pad_torsional_radius_m
        ):
            raise PhysicsSpecError(
                f"RobotSpec.pad_min_torsional_radius_m ({self.pad_min_torsional_radius_m}) > pad_torsional_radius_m ({self.pad_torsional_radius_m})",
                key="pad_min_torsional_radius_m",
            )


def validate_declaration(
    physics_dict: Mapping[str, Any] | None,
    *,
    section: str,
    task: str | None = None,
    layout: int | str | None = None,
    env_id: int | None = None,
    prim_path: str | None = None,
    scale: Sequence[Any] | None = None,
) -> dict[str, Any]:
    """Validate an upstream layout ``physics`` dictionary before any prim is created."""
    ctx = {"task": task, "layout": layout, "env_id": env_id, "prim_path": prim_path}
    raw = _to_plain(physics_dict or {})
    if not isinstance(raw, Mapping):
        raise PhysicsSpecError(f"Expected mapping for physics config, got {raw!r}", key="physics", **ctx)

    if scale is not None:
        _require_vec3_pos(scale, name="scale", **ctx)

    default_type = section.lower()
    obj_type = raw.get("type", default_type)
    if not isinstance(obj_type, str) or obj_type not in ("rigid", "geometry", "articulation", "dynamic", "garment", "fluid"):
        raise PhysicsSpecError(f"Invalid physics.type={obj_type!r}", key="type", **ctx)

    if obj_type in ("garment", "fluid"):
        # Out of scope per AGENTS.md: type is checked, remaining keys pass opaque and are marked unaudited.
        return dict(raw, type=obj_type)

    allowed = ALLOWED_DECLARATION_KEYS[obj_type]
    unknown = sorted(set(raw.keys()) - allowed)
    if unknown:
        raise PhysicsSpecError(
            f"Unknown physics key(s) {unknown} for type={obj_type!r}; allowed={sorted(allowed)}",
            key=unknown[0],
            **ctx,
        )

    for num_key in ("mass", "friction", "static_friction", "dynamic_friction", "restitution", "ratio"):
        if num_key in raw:
            v = _require_finite_float(raw[num_key], name=num_key, **ctx)
            if v < 0.0:
                raise PhysicsSpecError(f"Negative value for '{num_key}': {v}", key=num_key, **ctx)
            if num_key == "restitution" and v > 1.0:
                raise PhysicsSpecError(f"restitution > 1.0: {v}", key=num_key, **ctx)

    if "static_friction" in raw and "dynamic_friction" in raw:
        sf = float(raw["static_friction"])
        df = float(raw["dynamic_friction"])
        if sf + 1e-12 < df:
            raise PhysicsSpecError(
                f"Declared static_friction ({sf}) < dynamic_friction ({df})",
                key="static_friction",
                **ctx,
            )

    if "size" in raw:
        _require_vec3_pos(raw["size"], name="size", **ctx)

    for bool_key in ("collision", "track_contact_forces", "prepare_contact_sensor", "disable_stablization", "fixed_base", "reset_drive"):
        if bool_key in raw:
            _require_bool(raw[bool_key], name=bool_key, **ctx)

    if "contact_filter_prim_paths_expr" in raw:
        cf = raw["contact_filter_prim_paths_expr"]
        if not isinstance(cf, Sequence) or isinstance(cf, (str, bytes)) or not all(isinstance(x, str) for x in cf):
            raise PhysicsSpecError(
                f"Expected list[str] for contact_filter_prim_paths_expr, got {cf!r}",
                key="contact_filter_prim_paths_expr",
                **ctx,
            )

    return dict(raw, type=obj_type)


def validate_fixture_declaration(
    fixture_kind: str,
    cfg: Mapping[str, Any] | None,
    *,
    task: str | None = None,
    layout: int | str | None = None,
    env_id: int | None = None,
) -> dict[str, Any]:
    """Validate Table, Ground, or Room scene/layout configuration."""
    raw = _to_plain(cfg or {})
    ctx = {"task": task, "layout": layout, "env_id": env_id, "prim_path": raw.get("prim_path", fixture_kind)}
    if not isinstance(raw, Mapping):
        raise PhysicsSpecError(f"Expected mapping for {fixture_kind}, got {raw!r}", key=fixture_kind, **ctx)

    if fixture_kind == "Table":
        unknown = sorted(set(raw.keys()) - ALLOWED_TABLE_KEYS)
        if unknown:
            raise PhysicsSpecError(f"Unknown Table key(s) {unknown}", key=unknown[0], **ctx)
        if "scale" in raw:
            _require_vec3_pos(raw["scale"], name="Table.scale", **ctx)
        if "static" in raw:
            _require_bool(raw["static"], name="Table.static", **ctx)
        if "mass" in raw:
            m = _require_finite_float(raw["mass"], name="Table.mass", **ctx)
            if m <= 0.0:
                raise PhysicsSpecError(f"Table.mass must be > 0, got {m}", key="Table.mass", **ctx)
    elif fixture_kind == "Room":
        unknown = sorted(set(raw.keys()) - ALLOWED_ROOM_KEYS)
        if unknown:
            raise PhysicsSpecError(f"Unknown Room key(s) {unknown}", key=unknown[0], **ctx)
        if "scale" in raw:
            _require_vec3_pos(raw["scale"], name="Room.scale", **ctx)
        if "collision" in raw:
            _require_bool(raw["collision"], name="Room.collision", **ctx)
    elif fixture_kind == "Ground":
        if "thickness" in raw:
            th = _require_finite_float(raw["thickness"], name="Ground.thickness", **ctx)
            if th <= 0.0:
                raise PhysicsSpecError(f"Ground.thickness must be > 0, got {th}", key="Ground.thickness", **ctx)
        pm = raw.get("physics_material")
        if pm is not None:
            if not isinstance(pm, Mapping):
                raise PhysicsSpecError("Ground.physics_material must be a mapping", key="physics_material", **ctx)
            unknown = sorted(set(pm.keys()) - ALLOWED_GROUND_MATERIAL_KEYS)
            if unknown:
                raise PhysicsSpecError(f"Unknown Ground.physics_material key(s) {unknown}", key=unknown[0], **ctx)
            for nk in ("static_friction", "dynamic_friction", "restitution", "compliant_contact_stiffness", "compliant_contact_damping"):
                if nk in pm:
                    v = _require_finite_float(pm[nk], name=f"Ground.physics_material.{nk}", **ctx)
                    if v < 0.0 or (nk == "restitution" and v > 1.0):
                        raise PhysicsSpecError(f"Invalid Ground.physics_material.{nk}={v}", key=nk, **ctx)
            if "static_friction" in pm and "dynamic_friction" in pm:
                if float(pm["static_friction"]) + 1e-12 < float(pm["dynamic_friction"]):
                    raise PhysicsSpecError(
                        "Ground.physics_material static_friction < dynamic_friction",
                        key="static_friction",
                        **ctx,
                    )
            for mk in ("friction_combine_mode", "restitution_combine_mode"):
                if mk in pm and pm[mk] not in COMBINE_PRIORITY:
                    raise PhysicsSpecError(f"Invalid Ground.physics_material.{mk}={pm[mk]!r}", key=mk, **ctx)
    return dict(raw)


# --- Parameter table loader & feasibility checks -------------------------------------------------------------------

_ALLOWED_TOP_TABLE_KEYS: frozenset[str] = frozenset(
    {
        "schema",
        "schema_version",
        "description",
        "provenance",
        "status_vocabulary",
        "field_status",
        "class_evidence_vocabulary",
        "object_classes",
        "surface_classes",
        "scenarios",
        "classes",
        "restitution_schema",
        "camera_stand",
        "unary_values",
        "unary_values_default_scenario",
        "cooccurrence_definition",
        "pairs",
        "realised_vs_nominal",
        "models",
        "assets",
        "robots",
        "reviewed_tasks",
    }
)
_SHARED_DIR_META_KEYS: frozenset[str] = frozenset(
    {"schema", "schema_version", "description", "provenance", "status_vocabulary", "scenarios"}
)
_ALLOWED_VALUE_BAND_KEYS: frozenset[str] = frozenset(
    {
        "value",
        "range",
        "unit",
        "status",
        "value_status",
        "range_status",
        "confirmed_source_span",
        "sources",
        "conditions",
        "checked_by",
        "basis",
        "derivation",
        "reason_v2",
        "reasoning",
        "v3_change",
        "why",
        "fill_overrides",
        "value_is_for_fill",
        "impact_speed_above_0.5_m_s",
        "notes",
    }
)
_ALLOWED_SCALAR_LEAF_KEYS: frozenset[str] = frozenset(
    {"value", "unit", "status", "sources", "conditions", "checked_by", "basis", "form_limit", "notes"}
)
_ALLOWED_CLASS_KEYS: frozenset[str] = frozenset(
    {"density_kg_m3", "shell_density_kg_m3", "restitution", "unary_material", "scenario_membership", "notes", "description"}
)
_ALLOWED_UNARY_MAP_KEYS: frozenset[str] = frozenset(
    {
        "note",
        "fit",
        "static",
        "kinetic",
        "restitution",
        "friction_combine_mode",
        "restitution_combine_mode",
        "static_margin_mu",
        "kinetic_margin_mu",
        "status",
        "restitution_status",
    }
)
_ALLOWED_PAIR_KEYS: frozenset[str] = frozenset(
    {
        "a",
        "b",
        "key",
        "kind",
        "mu_static",
        "mu_kinetic",
        "restitution",
        "scenario_membership",
        "cooccurrence",
        "origin",
        "source_group",
        "superseded_duplicates",
        "notes",
    }
)
_ALLOWED_MODEL_KEYS: frozenset[str] = frozenset(
    {
        "id",
        "type",
        "category",
        "material_class",
        "alternative_material_classes",
        "bbox_extents_m",
        "metadata_source",
        "confidence",
        "identity",
        "reason",
        "flags",
        "declared_mass_kg",
        "declared_friction",
        "declared_size_field",
        "implied_bbox_density_kg_m3",
        "caption_name",
        "caption_material",
        "caption_other_material_kinds",
        "caption_status",
        "class_evidence",
        "evidence",
        "polymer_subtype",
        "polymer_subtype_basis",
        "prior_class",
        "prior_confidence",
        "prior_identity",
        "prior_reason",
        "real_vs_sim",
        "v2_changes",
        "v3_changes",
        "size_status",
        "notes",
    }
)
_ALLOWED_ASSET_KEYS: frozenset[str] = frozenset(
    {
        "id",
        "type",
        "category",
        "material_class",
        "alternative_material_classes",
        "confidence",
        "identity",
        "flags",
        "size_m",
        "collider_extents_m",
        "size_decision",
        "mass_kg",
        "mass_by_fill",
        "inertia_over_mass_by_fill",
        "com_offset_by_fill",
        "solid_collider_inertia_over_mass_m2",
        "shell_density_kg_m3",
        "material_override",
        "collision",
        "tasks",
        "task_variants",
        "fixed_base",
        "reset_drive",
        "link_masses_kg",
        "link_materials",
        "joint_drives",
        "belt_velocity_m_s",
        "notes",
    }
)
_UNAUTHORED_ROBOT_KEYS: frozenset[str] = frozenset({"actuators", "disable_gravity", "self_collisions"})
_ALLOWED_ROBOT_KEYS: frozenset[str] = frozenset(
    {
        "robot_name",
        "pad_material_class",
        "non_pad_link_material_classes",
        "link_materials",
        "link_inertials",
        "pad_torsional_radius_m",
        "pad_min_torsional_radius_m",
        "sanitize_camera_inertials",
        "gripper",
        "arm_limits",
        "notes",
    }
)
_ALLOWED_ARM_LIMITS_KEYS: frozenset[str] = frozenset({"joints", "lower_rad", "upper_rad", "velocity_rad_s", "effort_nm", "notes"})
_ALLOWED_GRIPPER_KEYS: frozenset[str] = frozenset(
    {
        "driven_joint",
        "mimic_joint",
        "finger_stiffness_n_per_m",
        "finger_damping_n_s_per_m",
        "finger_effort_limit_n",
        "finger_armature_kg",
        "finger_trip_n",
        "finger_motor_peak_n",
        "slew_m_per_s",
        "mimic",
        "notes",
    }
)
_ALLOWED_GRIPPER_MIMIC_KEYS: frozenset[str] = frozenset({"gearing", "offset", "axis", "notes"})


class _StatusCounter(dict):
    """Status count dict carrying the set of active leaf parameter keys visited during gating."""

    def __init__(self) -> None:
        super().__init__()
        self.checked_keys: set[str] = set()


def _check_status(
    status_raw: Any,
    *,
    key: str,
    allowed_statuses: frozenset[str],
    allow_unsourced_keys: frozenset[str] = frozenset(),
    allow_scenario_keys: frozenset[str] = frozenset(),
    status_counts: dict[str, int] | None = None,
    unsourced_keys_used: list[str] | None = None,
    scenario_keys_used: list[str] | None = None,
) -> str:
    if status_raw is None:
        status = "unsourced"
    elif isinstance(status_raw, str):
        status = status_raw
    else:
        raise PhysicsSpecError(f"Expected string status on '{key}', got {status_raw!r}", key=f"{key}.status")
    if status not in VALID_STATUSES:
        raise PhysicsSpecError(f"Invalid status {status!r} on '{key}'", key=f"{key}.status")
    if status not in allowed_statuses:
        if status == "unsourced" and key in allow_unsourced_keys:
            pass
        elif status == "scenario" and key in allow_scenario_keys:
            pass
        else:
            raise UnsourcedParameterError(
                f"Parameter '{key}' has status={status!r}, not in allowed={sorted(allowed_statuses)}",
                key=key,
            )
    if status == "unsourced" and unsourced_keys_used is not None:
        unsourced_keys_used.append(key)
    elif status == "scenario" and scenario_keys_used is not None:
        scenario_keys_used.append(key)
    if status_counts is not None:
        status_counts[status] = status_counts.get(status, 0) + 1
        checked = getattr(status_counts, "checked_keys", None)
        if checked is not None:
            checked.add(key)
    return status


def _weakest_status(statuses: Iterable[str]) -> str:
    st_list = list(statuses)
    if not st_list:
        return "unsourced"
    return min(st_list, key=lambda s: STATUS_RANK.get(s, 0))


def _parse_scalar_leaf(
    raw_val: Any,
    *,
    key: str,
    parser_fn: Any,
    allowed_statuses: frozenset[str],
    allow_unsourced_keys: frozenset[str] = frozenset(),
    allow_scenario_keys: frozenset[str] = frozenset(),
    status_counts: dict[str, int] | None = None,
    unsourced_keys_used: list[str] | None = None,
    scenario_keys_used: list[str] | None = None,
) -> tuple[Any, str]:
    """Validate a scalar/vector parameter leaf (mapping ``{value, status, ...}`` or bare value) and gate its status (default ``unsourced``)."""
    if isinstance(raw_val, Mapping):
        unknown = sorted(set(raw_val.keys()) - _ALLOWED_SCALAR_LEAF_KEYS)
        if unknown:
            raise PhysicsSpecError(f"Unknown key(s) {unknown} in parameter entry '{key}'", key=f"{key}.{unknown[0]}")
        val = parser_fn(raw_val.get("value"), f"{key}.value")
        st = _check_status(
            raw_val.get("status"),
            key=key,
            allowed_statuses=allowed_statuses,
            allow_unsourced_keys=allow_unsourced_keys,
            allow_scenario_keys=allow_scenario_keys,
            status_counts=status_counts,
            unsourced_keys_used=unsourced_keys_used,
            scenario_keys_used=scenario_keys_used,
        )
        return val, st
    val = parser_fn(raw_val, key)
    st = _check_status(
        None,
        key=key,
        allowed_statuses=allowed_statuses,
        allow_unsourced_keys=allow_unsourced_keys,
        allow_scenario_keys=allow_scenario_keys,
        status_counts=status_counts,
        unsourced_keys_used=unsourced_keys_used,
        scenario_keys_used=scenario_keys_used,
    )
    return val, st


def _validate_band_entry(
    entry: Mapping[str, Any],
    *,
    key: str,
    positive: bool = False,
    max_val: float | None = None,
    gate_field: str = "status",
    fill_mode: str | None = None,
    allowed_statuses: frozenset[str],
    allow_unsourced_keys: frozenset[str] = frozenset(),
    allow_scenario_keys: frozenset[str] = frozenset(),
    status_counts: dict[str, int] | None = None,
    unsourced_keys_used: list[str] | None = None,
    scenario_keys_used: list[str] | None = None,
) -> tuple[float, tuple[float, float], str]:
    if not isinstance(entry, Mapping):
        raise PhysicsSpecError(f"Expected mapping for parameter entry '{key}', got {entry!r}", key=key)
    unknown = sorted(set(entry.keys()) - _ALLOWED_VALUE_BAND_KEYS)
    if unknown:
        raise PhysicsSpecError(f"Unknown key(s) {unknown} in parameter entry '{key}'", key=f"{key}.{unknown[0]}")

    if "impact_speed_above_0.5_m_s" in entry and entry["impact_speed_above_0.5_m_s"] is not None:
        _require_bool(entry["impact_speed_above_0.5_m_s"], name=f"{key}.impact_speed_above_0.5_m_s")

    if "fill_overrides" in entry and entry["fill_overrides"] is not None:
        fo = entry["fill_overrides"]
        if not isinstance(fo, Mapping):
            raise PhysicsSpecError(f"Expected mapping for '{key}.fill_overrides'", key=f"{key}.fill_overrides")
        if fill_mode and fill_mode in fo and isinstance(fo[fill_mode], Mapping):
            merged_fo = dict(entry)
            merged_fo.pop("fill_overrides", None)
            merged_fo.update(fo[fill_mode])
            return _validate_band_entry(
                merged_fo,
                key=f"{key}.fill_overrides.{fill_mode}",
                positive=positive,
                max_val=max_val,
                gate_field=gate_field,
                fill_mode=None,
                allowed_statuses=allowed_statuses,
                allow_unsourced_keys=allow_unsourced_keys,
                allow_scenario_keys=allow_scenario_keys,
                status_counts=status_counts,
                unsourced_keys_used=unsourced_keys_used,
                scenario_keys_used=scenario_keys_used,
            )

    val = _require_finite_float(entry.get("value"), name=f"{key}.value")
    if positive and val <= 0.0:
        raise PhysicsSpecError(f"Expected positive value for '{key}', got {val}", key=f"{key}.value")
    if not positive and val < 0.0:
        raise PhysicsSpecError(f"Expected non-negative value for '{key}', got {val}", key=f"{key}.value")
    if max_val is not None and val > max_val:
        raise PhysicsSpecError(f"Value {val} exceeds max {max_val} for '{key}'", key=f"{key}.value")

    rng = entry.get("range")
    if not isinstance(rng, Sequence) or isinstance(rng, (str, bytes)) or len(rng) != 2:
        raise PhysicsSpecError(f"Expected [low, high] range for '{key}', got {rng!r}", key=f"{key}.range")
    lo = _require_finite_float(rng[0], name=f"{key}.range[0]")
    hi = _require_finite_float(rng[1], name=f"{key}.range[1]")
    if lo > hi or val + 1e-9 < lo or val - 1e-9 > hi:
        raise PhysicsSpecError(
            f"Value {val} outside range [{lo}, {hi}] for '{key}'",
            key=f"{key}.range",
        )

    if "confirmed_source_span" in entry and entry["confirmed_source_span"] is not None:
        css = entry["confirmed_source_span"]
        if not isinstance(css, Sequence) or isinstance(css, (str, bytes)) or len(css) != 2:
            raise PhysicsSpecError(
                f"Expected [low, high] confirmed_source_span for '{key}', got {css!r}",
                key=f"{key}.confirmed_source_span",
            )
        css_lo = _require_finite_float(css[0], name=f"{key}.confirmed_source_span[0]")
        css_hi = _require_finite_float(css[1], name=f"{key}.confirmed_source_span[1]")
        if css_lo > css_hi:
            raise PhysicsSpecError(
                f"confirmed_source_span low > high on '{key}': [{css_lo}, {css_hi}]",
                key=f"{key}.confirmed_source_span",
            )

    for sf_name in ("value_status", "range_status", "status"):
        if sf_name in entry and entry[sf_name] is not None:
            sv = entry[sf_name]
            if not isinstance(sv, str) or sv not in VALID_STATUSES:
                raise PhysicsSpecError(f"Invalid {sf_name}={sv!r} on '{key}'", key=f"{key}.{sf_name}")

    raw_gate_status = entry.get(gate_field) if (gate_field != "status" and gate_field in entry) else entry.get("status")
    status = _check_status(
        raw_gate_status,
        key=key,
        allowed_statuses=allowed_statuses,
        allow_unsourced_keys=allow_unsourced_keys,
        allow_scenario_keys=allow_scenario_keys,
        status_counts=status_counts,
        unsourced_keys_used=unsourced_keys_used,
        scenario_keys_used=scenario_keys_used,
    )
    return val, (lo, hi), status


def _parse_material_dict(
    d: Mapping[str, Any],
    *,
    key: str,
    fill_mode: str | None = None,
    allowed_statuses: frozenset[str],
    allow_unsourced_keys: frozenset[str] = frozenset(),
    allow_scenario_keys: frozenset[str] = frozenset(),
    status_counts: dict[str, int] | None = None,
    unsourced_keys_used: list[str] | None = None,
    scenario_keys_used: list[str] | None = None,
) -> MaterialSpec:
    if not isinstance(d, Mapping):
        raise PhysicsSpecError(f"Expected material mapping at '{key}'", key=key)
    allowed_m = {
        "static_friction",
        "dynamic_friction",
        "restitution",
        "restitution_by_fill",
        "friction_combine_mode",
        "restitution_combine_mode",
        "status",
        "sources",
        "conditions",
        "notes",
    }
    unknown = sorted(set(d.keys()) - allowed_m)
    if unknown:
        raise PhysicsSpecError(f"Unknown material key(s) {unknown} at '{key}'", key=f"{key}.{unknown[0]}")

    rest_val = d.get("restitution", 0.0)
    rest_mode = d.get("restitution_combine_mode", "average")
    if d.get("restitution_by_fill") is not None:
        rbf = d["restitution_by_fill"]
        if not isinstance(rbf, Mapping):
            raise PhysicsSpecError(f"Expected mapping for '{key}.restitution_by_fill'", key=f"{key}.restitution_by_fill")
        f_sel = fill_mode or "empty"
        if f_sel not in rbf:
            raise PhysicsSpecError(
                f"Material '{key}.restitution_by_fill' missing fill={f_sel!r}",
                key=f"{key}.restitution_by_fill",
            )
        rf_entry = rbf[f_sel]
        rf_key = f"{key}.restitution_by_fill.{f_sel}"
        if isinstance(rf_entry, Mapping) and "range" in rf_entry:
            rest_val, _, _ = _validate_band_entry(
                rf_entry,
                key=rf_key,
                positive=False,
                max_val=1.0,
                allowed_statuses=allowed_statuses,
                allow_unsourced_keys=allow_unsourced_keys,
                allow_scenario_keys=allow_scenario_keys,
                status_counts=status_counts,
                unsourced_keys_used=unsourced_keys_used,
                scenario_keys_used=scenario_keys_used,
            )
        else:
            def _unit_cor(v: Any, nm: str) -> float:
                fv = _require_finite_float(v, name=nm)
                if not (0.0 <= fv <= 1.0):
                    raise PhysicsSpecError(f"restitution must be in [0, 1] at '{nm}', got {fv}", key=nm)
                return fv

            rest_val, _ = _parse_scalar_leaf(
                rf_entry,
                key=rf_key,
                parser_fn=_unit_cor,
                allowed_statuses=allowed_statuses,
                allow_unsourced_keys=allow_unsourced_keys,
                allow_scenario_keys=allow_scenario_keys,
                status_counts=status_counts,
                unsourced_keys_used=unsourced_keys_used,
                scenario_keys_used=scenario_keys_used,
            )

    spec = MaterialSpec(
        static_friction=d.get("static_friction"),
        dynamic_friction=d.get("dynamic_friction"),
        restitution=rest_val,
        friction_combine_mode=d.get("friction_combine_mode", "average"),
        restitution_combine_mode=rest_mode,
        allow_legacy_inversion=False,
    )
    _check_status(
        d.get("status"),
        key=key,
        allowed_statuses=allowed_statuses,
        allow_unsourced_keys=allow_unsourced_keys,
        allow_scenario_keys=allow_scenario_keys,
        status_counts=status_counts,
        unsourced_keys_used=unsourced_keys_used,
        scenario_keys_used=scenario_keys_used,
    )
    return spec


def _parse_collision_dict(
    d: Mapping[str, Any] | None,
    *,
    key: str,
    default_enabled: bool | None = None,
    default_approx: str = "asset",
    allowed_statuses: frozenset[str] | None = None,
    allow_unsourced_keys: frozenset[str] = frozenset(),
    status_counts: dict[str, int] | None = None,
    unsourced_keys_used: list[str] | None = None,
) -> CollisionSpec:
    if d is None:
        return CollisionSpec(enabled=default_enabled, approximation=default_approx)
    if not isinstance(d, Mapping):
        raise PhysicsSpecError(f"Expected collision mapping at '{key}'", key=key)
    allowed_c = {
        "enabled",
        "approximation",
        "contact_offset_m",
        "rest_offset_m",
        "torsional_patch_radius_m",
        "min_torsional_patch_radius_m",
        "status",
        "sources",
        "conditions",
        "notes",
    }
    unknown = sorted(set(d.keys()) - allowed_c)
    if unknown:
        raise PhysicsSpecError(f"Unknown collision key(s) {unknown} at '{key}'", key=f"{key}.{unknown[0]}")
    spec = CollisionSpec(
        enabled=d.get("enabled", default_enabled),
        approximation=d.get("approximation", default_approx),
        contact_offset_m=d.get("contact_offset_m"),
        rest_offset_m=d.get("rest_offset_m"),
        torsional_patch_radius_m=d.get("torsional_patch_radius_m"),
        min_torsional_patch_radius_m=d.get("min_torsional_patch_radius_m"),
    )
    if allowed_statuses is not None:
        _check_status(
            d.get("status"),
            key=key,
            allowed_statuses=allowed_statuses,
            allow_unsourced_keys=allow_unsourced_keys,
            status_counts=status_counts,
            unsourced_keys_used=unsourced_keys_used,
        )
    return spec


def _parse_drive_dict(
    d: Any,
    *,
    key: str,
    allowed_statuses: frozenset[str],
    allow_unsourced_keys: frozenset[str] = frozenset(),
    status_counts: dict[str, int] | None = None,
    unsourced_keys_used: list[str] | None = None,
) -> DriveSpec:
    if not isinstance(d, Mapping):
        raise PhysicsSpecError(f"Expected drive mapping at '{key}', got {d!r}", key=key)
    allowed_d = {
        "stiffness",
        "damping",
        "max_force",
        "max_velocity",
        "drive_type",
        "status",
        "sources",
        "conditions",
        "notes",
    }
    unknown = sorted(set(d.keys()) - allowed_d)
    if unknown:
        raise PhysicsSpecError(f"Unknown drive key(s) {unknown} at '{key}'", key=f"{key}.{unknown[0]}")
    spec = DriveSpec(
        stiffness=d.get("stiffness"),
        damping=d.get("damping"),
        max_force=d.get("max_force"),
        max_velocity=d.get("max_velocity"),
        drive_type=d.get("drive_type", "angular"),
    )
    _check_status(
        d.get("status"),
        key=key,
        allowed_statuses=allowed_statuses,
        allow_unsourced_keys=allow_unsourced_keys,
        status_counts=status_counts,
        unsourced_keys_used=unsourced_keys_used,
    )
    return spec


class PhysicsParamsTable:
    """Validated parameter table holding class unary materials, pair bands, per-asset rows, and robot specs."""

    def __init__(
        self,
        raw: Mapping[str, Any],
        *,
        source_path: str = "<memory>",
        table_sha256: str = "",
        allowed_statuses: Iterable[str] = ("sourced", "estimated"),
        allow_unsourced_keys: Iterable[str] = (),
        allow_scenario_keys: Iterable[str] = (),
        scenario: Mapping[str, Any] | None = None,
        scenario_opt_in: str | None = None,
    ) -> None:
        self.source_path = source_path
        self.table_sha256 = table_sha256
        self.allowed_statuses = frozenset(str(s) for s in allowed_statuses)
        unknown_status = self.allowed_statuses - VALID_STATUSES
        if unknown_status:
            raise PhysicsSpecError(f"Unknown allowed_statuses {sorted(unknown_status)}", key="allowed_statuses")
        adhoc_unsourced_set: set[str] = {str(k) for k in allow_unsourced_keys}
        adhoc_scenario_set: set[str] = {str(k) for k in allow_scenario_keys}
        profile_unsourced_set: set[str] = set()
        profile_scenario_set: set[str] = set()
        self.scenario_opt_in: str | None = str(scenario_opt_in).strip() if scenario_opt_in else None
        self.status_counts: _StatusCounter = _StatusCounter()
        self.unsourced_keys_used: list[str] = []
        self.scenario_keys_used: list[str] = []
        self.derived_unary_classes: dict[str, str] = {}

        data = _to_plain(raw)
        if not isinstance(data, Mapping):
            raise PhysicsSpecError("Parameter table root must be a JSON object", key="root")
        unknown_top = sorted(set(data.keys()) - _ALLOWED_TOP_TABLE_KEYS)
        if unknown_top:
            raise PhysicsSpecError(f"Unknown top-level parameter table key(s) {unknown_top}", key=unknown_top[0])
        self.provenance: dict[str, Any] | None = (
            dict(data["provenance"]) if isinstance(data.get("provenance"), Mapping) else None
        )

        scenarios_cfg = data.get("scenarios") or {}
        default_scen = dict(scenarios_cfg.get("default") or {}) if isinstance(scenarios_cfg, Mapping) else {}
        if self.scenario_opt_in is not None:
            profiles = scenarios_cfg.get("opt_in_profiles") if isinstance(scenarios_cfg, Mapping) else None
            if not isinstance(profiles, Mapping) or self.scenario_opt_in not in profiles:
                avail = sorted(profiles.keys()) if isinstance(profiles, Mapping) else []
                raise PhysicsSpecError(
                    f"Unknown physics_scenario_opt_in={self.scenario_opt_in!r}; available profiles={avail}",
                    key="physics_scenario_opt_in",
                )
            prof = profiles[self.scenario_opt_in]
            if not isinstance(prof, Mapping):
                raise PhysicsSpecError(
                    f"scenarios.opt_in_profiles.{self.scenario_opt_in} must be a mapping",
                    key=f"scenarios.opt_in_profiles.{self.scenario_opt_in}",
                )
            unknown_pk = sorted(
                set(prof.keys()) - {"description", "scenario", "allow_unsourced_keys", "allow_scenario_keys", "notes"}
            )
            if unknown_pk:
                raise PhysicsSpecError(
                    f"Unknown key(s) {unknown_pk} in scenarios.opt_in_profiles.{self.scenario_opt_in}",
                    key=f"scenarios.opt_in_profiles.{self.scenario_opt_in}.{unknown_pk[0]}",
                )
            if isinstance(prof.get("scenario"), Mapping):
                default_scen.update(prof["scenario"])
            for k in prof.get("allow_unsourced_keys") or ():
                profile_unsourced_set.add(str(k))
            for k in prof.get("allow_scenario_keys") or ():
                profile_scenario_set.add(str(k))

        if scenario:
            default_scen.update(_to_plain(scenario))
        self.scenario: dict[str, Any] = default_scen
        raw_fill_tasks = self.scenario.get("fill_tasks")
        if raw_fill_tasks is not None:
            if not isinstance(raw_fill_tasks, Sequence) or isinstance(raw_fill_tasks, (str, bytes)):
                raise PhysicsSpecError("scenario.fill_tasks must be a list of task names", key="scenario.fill_tasks")
            self.fill_tasks: frozenset[str] | None = frozenset(str(t) for t in raw_fill_tasks)
        elif self.scenario_opt_in == "bottles_stress_full":
            self.fill_tasks = frozenset({"put_bottles_into_dustbin"})
        else:
            self.fill_tasks = None
        self.adhoc_unsourced_keys: frozenset[str] = frozenset(adhoc_unsourced_set)
        self.profile_unsourced_keys: frozenset[str] = frozenset(profile_unsourced_set)
        self.adhoc_scenario_keys: frozenset[str] = frozenset(adhoc_scenario_set)
        self.profile_scenario_keys: frozenset[str] = frozenset(profile_scenario_set)
        self.allow_unsourced_keys: frozenset[str] = frozenset(profile_unsourced_set | adhoc_unsourced_set)
        self.allow_scenario_keys: frozenset[str] = frozenset(profile_scenario_set | adhoc_scenario_set)

        self.reviewed_tasks: dict[str, str] = dict(data.get("reviewed_tasks") or {})
        self.all_known_tasks: tuple[str, ...] = tuple(sorted(self.reviewed_tasks.keys())) or ("__unscoped__",)
        self.known_classes: set[str] = set()
        self.class_densities: dict[str, tuple[float, tuple[float, float], str]] = {}
        self.class_shell_densities: dict[str, tuple[float, tuple[float, float], str]] = {}
        self.class_restitutions: dict[str, tuple[float, str]] = {}
        self.class_full_restitutions: dict[str, tuple[float, str]] = {}
        self.class_materials: dict[str, MaterialSpec] = {}
        self.pair_bands: dict[tuple[str, str], dict[str, Any]] = {}

        self._load_classes_base(data)
        self._load_pairs_bands(data)
        self._load_unaries(data)
        self.assets: dict[str, dict[str, Any]] = {}
        self._load_and_verify_assets(data)
        self.robots: dict[str, RobotSpec] = {}
        self._load_robots(data)
        self._verify_pairs_algebra(data)

        unmatched_unsourced = sorted(self.allow_unsourced_keys - self.status_counts.checked_keys)
        if unmatched_unsourced:
            raise PhysicsSpecError(
                f"allow_unsourced_keys contains key(s) matching no active parameter leaf: {unmatched_unsourced}",
                key=unmatched_unsourced[0],
            )
        unmatched_scenario = sorted(self.allow_scenario_keys - self.status_counts.checked_keys)
        if unmatched_scenario:
            raise PhysicsSpecError(
                f"allow_scenario_keys contains key(s) matching no active parameter leaf: {unmatched_scenario}",
                key=unmatched_scenario[0],
            )

        self.unsourced_keys_used = sorted(set(self.unsourced_keys_used))
        self.scenario_keys_used = sorted(set(self.scenario_keys_used))
        self.profile_unsourced_keys_used: list[str] = [
            k for k in self.unsourced_keys_used if k in self.profile_unsourced_keys
        ]
        self.adhoc_unsourced_keys_used: list[str] = [
            k for k in self.unsourced_keys_used if k in self.adhoc_unsourced_keys
        ]
        self.realised_vs_nominal: dict[str, Any] = self._load_and_verify_realised_vs_nominal(data)

    @classmethod
    def from_path(
        cls,
        path: str,
        *,
        allowed_statuses: Iterable[str] = ("sourced", "estimated"),
        allow_unsourced_keys: Iterable[str] = (),
        allow_scenario_keys: Iterable[str] = (),
        scenario: Mapping[str, Any] | None = None,
        scenario_opt_in: str | None = None,
    ) -> PhysicsParamsTable:
        """Load a validated parameter table from a JSON file or directory.

        In directory mode, reads the five canonical D2 filenames
        ``("classes.json", "class_values.json", "pairs.json", "assets.json", "robots.json")``
        in deterministic order and deliberately ignores auxiliary report JSONs (such as
        ``alignment_84.json`` or ``feasibility.json`` in ``audit/out/63/``).
        """
        if os.path.isdir(path):
            merged: dict[str, Any] = {}
            h = hashlib.sha256()
            for fname in ("classes.json", "class_values.json", "pairs.json", "assets.json", "robots.json"):
                fpath = os.path.join(path, fname)
                if os.path.isfile(fpath):
                    with open(fpath, "rb") as bf:
                        raw_bytes = bf.read()
                    h.update(fname.encode("utf-8") + b"\0" + raw_bytes + b"\0")
                    sub = json.loads(raw_bytes.decode("utf-8"), parse_constant=cls._reject_nonfinite_json)
                    if not isinstance(sub, Mapping):
                        raise PhysicsSpecError(f"Expected JSON object in {fpath!r}", key=fname)
                    unknown_file_keys = sorted(set(sub.keys()) - _ALLOWED_TOP_TABLE_KEYS)
                    if unknown_file_keys:
                        raise PhysicsSpecError(
                            f"Unknown top-level key(s) {unknown_file_keys} in {fpath!r}",
                            key=unknown_file_keys[0],
                        )
                    for k, v in sub.items():
                        if k in merged and k not in _SHARED_DIR_META_KEYS:
                            raise PhysicsSpecError(
                                f"Duplicate top-level parameter key {k!r} across files in directory {path!r}",
                                key=k,
                            )
                        if k == "scenarios" and k in merged and isinstance(merged[k], Mapping) and isinstance(v, Mapping):
                            merged[k] = {**merged[k], **v}
                        else:
                            merged[k] = v
            if not merged:
                raise PhysicsSpecError(f"No valid parameter JSON files found in directory {path!r}", key="physics_params_path")
            return cls(
                merged,
                source_path=path,
                table_sha256=h.hexdigest(),
                allowed_statuses=allowed_statuses,
                allow_unsourced_keys=allow_unsourced_keys,
                allow_scenario_keys=allow_scenario_keys,
                scenario=scenario,
                scenario_opt_in=scenario_opt_in,
            )

        if not os.path.isfile(path):
            raise PhysicsSpecError(f"Parameter table file not found: {path!r}", key="physics_params_path")
        with open(path, "rb") as bf:
            raw_bytes = bf.read()
        sha = hashlib.sha256(raw_bytes).hexdigest()
        parsed = json.loads(raw_bytes.decode("utf-8"), parse_constant=cls._reject_nonfinite_json)
        return cls(
            parsed,
            source_path=path,
            table_sha256=sha,
            allowed_statuses=allowed_statuses,
            allow_unsourced_keys=allow_unsourced_keys,
            allow_scenario_keys=allow_scenario_keys,
            scenario=scenario,
            scenario_opt_in=scenario_opt_in,
        )

    @staticmethod
    def _reject_nonfinite_json(token: str) -> Any:
        raise PhysicsSpecError(f"Non-finite JSON constant {token!r} in parameter table", key="NaN")

    def _load_classes_base(self, data: Mapping[str, Any]) -> None:
        classes_raw = data.get("classes") or {}
        if not isinstance(classes_raw, Mapping):
            raise PhysicsSpecError("'classes' must be a mapping", key="classes")
        fill_mode = str(self.scenario.get("fill", self.scenario.get("bottle_fill", "empty")))

        for sec_name in ("object_classes", "surface_classes"):
            sec_map = data.get(sec_name)
            if sec_map is not None:
                if not isinstance(sec_map, Mapping):
                    raise PhysicsSpecError(f"'{sec_name}' must be a mapping", key=sec_name)
                self.known_classes.update(str(k) for k in sec_map.keys())

        for cname, cdict in sorted(classes_raw.items()):
            self.known_classes.add(str(cname))
            if not isinstance(cdict, Mapping):
                raise PhysicsSpecError(f"Class '{cname}' must be a mapping", key=f"classes.{cname}")
            unknown = sorted(set(cdict.keys()) - _ALLOWED_CLASS_KEYS)
            if unknown:
                raise PhysicsSpecError(f"Unknown key(s) {unknown} in class '{cname}'", key=f"classes.{cname}.{unknown[0]}")

            if "density_kg_m3" in cdict and cdict["density_kg_m3"] is not None:
                self.class_densities[cname] = _validate_band_entry(
                    cdict["density_kg_m3"],
                    key=f"classes.{cname}.density_kg_m3",
                    positive=True,
                    fill_mode=fill_mode,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )

            if "shell_density_kg_m3" in cdict and cdict["shell_density_kg_m3"] is not None:
                self.class_shell_densities[cname] = _validate_band_entry(
                    cdict["shell_density_kg_m3"],
                    key=f"classes.{cname}.shell_density_kg_m3",
                    positive=True,
                    fill_mode=fill_mode,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )

            if "restitution" in cdict and cdict["restitution"] is not None:
                # #88/#91 Note 1: base (empty) class restitution stays on the class for open/static receptacles
                # (e.g. Geometry/dustbin/00000 in polyolefin_plastic); fill_overrides.full applies ONLY to dynamic
                # sealed containers (hollow_contents_dependent without open_receptacle).
                r_entry = cdict["restitution"]
                r_val, _, r_stat = _validate_band_entry(
                    r_entry,
                    key=f"classes.{cname}.restitution",
                    positive=False,
                    max_val=1.0,
                    fill_mode=None,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )
                self.class_restitutions[cname] = (r_val, r_stat)
                if (
                    fill_mode == "full"
                    and isinstance(r_entry, Mapping)
                    and isinstance(r_entry.get("fill_overrides"), Mapping)
                    and isinstance(r_entry["fill_overrides"].get("full"), Mapping)
                ):
                    rf_val, _, rf_stat = _validate_band_entry(
                        r_entry,
                        key=f"classes.{cname}.restitution",
                        positive=False,
                        max_val=1.0,
                        fill_mode="full",
                        allowed_statuses=self.allowed_statuses,
                        allow_unsourced_keys=self.allow_unsourced_keys,
                        allow_scenario_keys=self.allow_scenario_keys,
                        status_counts=self.status_counts,
                        unsourced_keys_used=self.unsourced_keys_used,
                        scenario_keys_used=self.scenario_keys_used,
                    )
                    self.class_full_restitutions[cname] = (rf_val, rf_stat)

            if "unary_material" in cdict and cdict["unary_material"] is not None:
                self.class_materials[cname] = _parse_material_dict(
                    cdict["unary_material"],
                    key=f"classes.{cname}.unary_material",
                    fill_mode=fill_mode,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )

    def _load_pairs_bands(self, data: Mapping[str, Any]) -> None:
        pairs_raw = data.get("pairs")
        if pairs_raw is None:
            return
        pair_items: list[tuple[str, Mapping[str, Any]]] = []
        if isinstance(pairs_raw, Mapping):
            pair_items = [(str(k), v) for k, v in pairs_raw.items()]
        elif isinstance(pairs_raw, list):
            for idx, item in enumerate(pairs_raw):
                if not isinstance(item, Mapping):
                    raise PhysicsSpecError(f"pairs[{idx}] must be a mapping", key=f"pairs[{idx}]")
                pk = item.get("key") or f"{item.get('a')}__{item.get('b')}"
                pair_items.append((str(pk), item))
        else:
            raise PhysicsSpecError("'pairs' must be a list or mapping", key="pairs")

        fill_mode = str(self.scenario.get("fill", self.scenario.get("bottle_fill", "empty")))
        for pk, pdict in pair_items:
            if not isinstance(pdict, Mapping):
                raise PhysicsSpecError(f"Pair '{pk}' must be a mapping", key=f"pairs.{pk}")
            unknown = sorted(set(pdict.keys()) - _ALLOWED_PAIR_KEYS)
            if unknown:
                raise PhysicsSpecError(f"Unknown key(s) {unknown} in pair '{pk}'", key=f"pairs.{pk}.{unknown[0]}")
            if "__" in pk:
                ca, cb = pk.split("__", 1)
            else:
                ca, cb = str(pdict.get("a", "")), str(pdict.get("b", ""))
            if not ca or not cb:
                raise PhysicsSpecError(f"Pair '{pk}' missing class endpoints", key=f"pairs.{pk}")
            canon = (min(ca, cb), max(ca, cb))

            # Gate pair friction bands on range_status (what constrains unary feasibility) when present, else status.
            s_nom, s_rng, s_stat = _validate_band_entry(
                pdict.get("mu_static"),
                key=f"pairs.{pk}.mu_static",
                positive=False,
                gate_field="range_status",
                fill_mode=fill_mode,
                allowed_statuses=self.allowed_statuses,
                allow_unsourced_keys=self.allow_unsourced_keys,
                allow_scenario_keys=self.allow_scenario_keys,
                status_counts=self.status_counts,
                unsourced_keys_used=self.unsourced_keys_used,
                scenario_keys_used=self.scenario_keys_used,
            )
            k_nom, k_rng, k_stat = _validate_band_entry(
                pdict.get("mu_kinetic"),
                key=f"pairs.{pk}.mu_kinetic",
                positive=False,
                gate_field="range_status",
                fill_mode=fill_mode,
                allowed_statuses=self.allowed_statuses,
                allow_unsourced_keys=self.allow_unsourced_keys,
                allow_scenario_keys=self.allow_scenario_keys,
                status_counts=self.status_counts,
                unsourced_keys_used=self.unsourced_keys_used,
                scenario_keys_used=self.scenario_keys_used,
            )

            rest_nom: float | None = None
            rest_rng: tuple[float, float] | None = None
            rest_rng_unfilled: tuple[float, float] | None = None
            rest_check_active = False
            if pdict.get("restitution") is not None:
                p_rest = pdict["restitution"]
                if not isinstance(p_rest, Mapping):
                    raise PhysicsSpecError(f"Expected mapping for 'pairs.{pk}.restitution'", key=f"pairs.{pk}.restitution")
                # #88/#91 Note 2: scope pair COR check assertions strictly to contacts with impact speed > 0.5 m/s
                # (env/environment/base_env.py:27 bounce_threshold_velocity = 0.5 m/s).
                above_05 = bool(p_rest.get("impact_speed_above_0.5_m_s", False))
                if above_05:
                    rest_nom, rest_rng, _ = _validate_band_entry(
                        p_rest,
                        key=f"pairs.{pk}.restitution",
                        positive=False,
                        max_val=1.0,
                        gate_field="range_status",
                        fill_mode=fill_mode,
                        allowed_statuses=self.allowed_statuses,
                        allow_unsourced_keys=self.allow_unsourced_keys,
                        allow_scenario_keys=self.allow_scenario_keys,
                        status_counts=self.status_counts,
                        unsourced_keys_used=self.unsourced_keys_used,
                        scenario_keys_used=self.scenario_keys_used,
                    )
                    rest_check_active = True
                    if fill_mode != "empty":
                        # The fill-override band covers contacts of a filled sealed container; contacts where no side
                        # is one (an open cup on a table, #167) keep the empty band. Not gated or counted twice.
                        _, rest_rng_unfilled, _ = _validate_band_entry(
                            p_rest,
                            key=f"pairs.{pk}.restitution",
                            positive=False,
                            max_val=1.0,
                            gate_field="range_status",
                            fill_mode="empty",
                            allowed_statuses=VALID_STATUSES,
                        )
                else:
                    # Validate numeric range/bounds on quasi-static (<0.5 m/s) entries without gating/asserting inactive bounce.
                    rest_nom, rest_rng, _ = _validate_band_entry(
                        p_rest,
                        key=f"pairs.{pk}.restitution",
                        positive=False,
                        max_val=1.0,
                        gate_field="range_status",
                        fill_mode=fill_mode,
                        allowed_statuses=VALID_STATUSES,
                    )

            self.pair_bands[canon] = {
                "key": pk,
                "nominal_static": s_nom,
                "nominal_kinetic": k_nom,
                "nominal_restitution": rest_nom,
                "mu_static": s_rng,
                "mu_kinetic": k_rng,
                "restitution": rest_rng,
                "restitution_unfilled": rest_rng_unfilled if rest_rng_unfilled is not None else rest_rng,
                "restitution_check_active": rest_check_active,
                "status": _weakest_status((s_stat, k_stat)),
                "scenario_membership": pdict.get("scenario_membership", "all"),
            }

    def _load_unaries(self, data: Mapping[str, Any]) -> None:
        unary_map = data.get("unary_values")
        map_key = "unary_values"
        if unary_map is None and data.get("unary_values_default_scenario") is not None:
            unary_map = data.get("unary_values_default_scenario")
            map_key = "unary_values_default_scenario"
        if unary_map is None:
            return
        if not isinstance(unary_map, Mapping):
            raise PhysicsSpecError(f"'{map_key}' must be a mapping", key=map_key)
        unknown_u = sorted(set(unary_map.keys()) - _ALLOWED_UNARY_MAP_KEYS)
        if unknown_u:
            raise PhysicsSpecError(f"Unknown key(s) {unknown_u} in '{map_key}'", key=f"{map_key}.{unknown_u[0]}")

        u_static = unary_map.get("static") or {}
        u_kinetic = unary_map.get("kinetic") or {}
        u_rest = unary_map.get("restitution") or {}
        u_fmode = unary_map.get("friction_combine_mode") or {}
        u_rmode = unary_map.get("restitution_combine_mode") or {}
        u_status = unary_map.get("status")
        u_rstatus = unary_map.get("restitution_status")
        if not isinstance(u_static, Mapping) or not isinstance(u_kinetic, Mapping):
            raise PhysicsSpecError(f"'{map_key}.static' and '{map_key}.kinetic' must be mappings", key=map_key)

        unary_cnames = sorted(set(u_static.keys()) | set(u_kinetic.keys()))
        for cname in unary_cnames:
            if cname not in self.class_materials and (cname not in u_static or cname not in u_kinetic):
                raise PhysicsSpecError(f"Class '{cname}' missing static or kinetic in {map_key}", key=f"{map_key}.{cname}")
        all_checked_classes = set(self.class_materials.keys()) | {str(c) for c in unary_cnames}

        for cname in unary_cnames:
            self.known_classes.add(str(cname))
            if cname in self.class_materials:
                continue

            if cname in u_rest:
                rest_val = u_rest[cname]
            elif cname in self.class_restitutions:
                rest_val = self.class_restitutions[cname][0]
            else:
                rest_val = 0.0

            mat_spec = MaterialSpec(
                static_friction=u_static[cname],
                dynamic_friction=u_kinetic[cname],
                restitution=rest_val,
                friction_combine_mode=u_fmode.get(cname, "average"),
                restitution_combine_mode=u_rmode.get(cname, "average"),
                allow_legacy_inversion=False,
            )

            # Gate friction status: explicit status if given, else derived ONLY from bands that _verify_pairs_algebra
            # actually checks (both endpoints in all_checked_classes); with 0 checked bands -> None ('unsourced').
            if isinstance(u_status, Mapping):
                raw_st = u_status.get(cname)
            elif u_status is not None:
                raw_st = u_status
            else:
                band_statuses = [
                    pb["status"]
                    for (ca, cb), pb in self.pair_bands.items()
                    if (ca == cname or cb == cname) and (ca in all_checked_classes and cb in all_checked_classes)
                ]
                raw_st = _weakest_status(band_statuses) if band_statuses else None
                if raw_st is not None:
                    self.derived_unary_classes[str(cname)] = raw_st

            _check_status(
                raw_st,
                key=f"{map_key}.{cname}",
                allowed_statuses=self.allowed_statuses,
                allow_unsourced_keys=self.allow_unsourced_keys,
                allow_scenario_keys=self.allow_scenario_keys,
                status_counts=self.status_counts,
                unsourced_keys_used=self.unsourced_keys_used,
                scenario_keys_used=self.scenario_keys_used,
            )

            # If restitution came from u_rest (rather than an already-gated classes.<cname>.restitution band), gate it too.
            if cname in u_rest:
                raw_rst = u_rstatus.get(cname) if isinstance(u_rstatus, Mapping) else (u_rstatus if u_rstatus is not None else (
                    u_status.get(cname) if isinstance(u_status, Mapping) else u_status
                ))
                _check_status(
                    raw_rst,
                    key=f"{map_key}.restitution.{cname}",
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )

            self.class_materials[str(cname)] = mat_spec

    def _robot_carrier_materials_by_class(self, data: Mapping[str, Any] | None) -> dict[str, list[MaterialSpec]]:
        """Return dynamic robot link ``MaterialSpec`` carriers per material class across all tasks."""
        out: dict[str, list[MaterialSpec]] = {}

        def _add(cls_name: str, mat: MaterialSpec) -> None:
            bucket = out.setdefault(cls_name, [])
            if mat not in bucket:
                bucket.append(mat)

        pad_scen = self.scenario.get("finger_pad")
        if isinstance(pad_scen, str) and pad_scen in self.class_materials:
            _add(pad_scen, self.class_materials[pad_scen])

        robots_raw = data.get("robots") if isinstance(data, Mapping) else None
        if isinstance(robots_raw, Mapping):
            for rname, rdict in robots_raw.items():
                if not isinstance(rdict, Mapping):
                    continue
                rspec = self.robots.get(str(rname))
                if rdict.get("pad_material_class") or pad_scen:
                    for pcls in ("pad_hard", "pad_elastomer", rdict.get("pad_material_class"), pad_scen):
                        if isinstance(pcls, str) and pcls in self.class_materials:
                            _add(pcls, self.class_materials[pcls])
                npl = rdict.get("non_pad_link_material_classes")
                if isinstance(npl, Mapping):
                    for lk, cleaf in npl.items():
                        cval = cleaf.get("value") if isinstance(cleaf, Mapping) else cleaf
                        if isinstance(cval, str) and cval in self.class_materials:
                            lmat = (
                                rspec.link_materials.get(str(lk), self.class_materials[cval])
                                if rspec is not None
                                else self.class_materials[cval]
                            )
                            _add(cval, lmat)
        return out

    def _verify_pairs_algebra(self, data: Mapping[str, Any] | None = None) -> None:
        fill_mode = str(self.scenario.get("fill", self.scenario.get("bottle_fill", "empty")))
        robot_carriers = self._robot_carrier_materials_by_class(data)
        table_rec = self.assets.get("Scene/Table")

        def _sealed(a: Mapping[str, Any]) -> bool:
            fl = a.get("flags") or ()
            if a.get("is_static", False) or "hollow_contents_dependent" not in fl or "open_receptacle" in fl:
                return False
            if fill_mode != "empty" and self.fill_tasks is not None and not (a["tasks_set"] & self.fill_tasks):
                return False
            return True

        def _side_realizations(
            cls_name: str, cls_mat: MaterialSpec
        ) -> list[tuple[MaterialSpec, bool, bool, Mapping[str, Any] | None]]:
            mats: list[tuple[MaterialSpec, bool, bool, Mapping[str, Any] | None]] = [
                (a["material"], not a.get("is_static", False), _sealed(a), a)
                for a in self.assets.values()
                if a.get("material_class") == cls_name and a.get("material") is not None
            ]
            if (
                table_rec is not None
                and cls_name in ("table_laminate", "table_wood")
                and table_rec.get("material_class") != cls_name
            ):
                mats.append((cls_mat, not table_rec.get("is_static", False), False, table_rec))
            # Always include dynamic robot link carriers (present in every task, meeting every task's objects),
            # even when task-scoped assets also use the same material class (#175 re-review item 3 gap 1).
            for rmat in robot_carriers.get(cls_name, ()):
                mats.append((rmat, True, False, None))
            if not mats:
                rc = (
                    self.class_full_restitutions[cls_name][0]
                    if (fill_mode == "full" and cls_name in self.class_full_restitutions)
                    else cls_mat.restitution
                )
                mats = [
                    (
                        MaterialSpec(
                            cls_mat.static_friction,
                            cls_mat.dynamic_friction,
                            rc,
                            cls_mat.friction_combine_mode,
                            cls_mat.restitution_combine_mode,
                        ),
                        cls_name not in ("table_laminate", "table_wood", "ground"),
                        cls_name in self.class_full_restitutions,
                        None,
                    )
                ]
            return mats

        for (ca, cb), pb in self.pair_bands.items():
            if ca in self.class_materials and cb in self.class_materials:
                ma = self.class_materials[ca]
                mb = self.class_materials[cb]
                eff_s, mode_s = combine_values(
                    ma.static_friction, mb.static_friction, ma.friction_combine_mode, mb.friction_combine_mode
                )
                eff_k, _ = combine_values(
                    ma.dynamic_friction, mb.dynamic_friction, ma.friction_combine_mode, mb.friction_combine_mode
                )
                s_rng, k_rng = pb["mu_static"], pb["mu_kinetic"]
                pk = pb["key"]
                if not (s_rng[0] - 1e-6 <= eff_s <= s_rng[1] + 1e-6):
                    raise PhysicsSpecError(
                        f"Combined static friction {eff_s:.4f} (mode={mode_s}) for ({ca}, {cb}) outside pair band {list(s_rng)}",
                        key=f"pairs.{pk}.mu_static",
                    )
                if not (k_rng[0] - 1e-6 <= eff_k <= k_rng[1] + 1e-6):
                    raise PhysicsSpecError(
                        f"Combined dynamic friction {eff_k:.4f} (mode={mode_s}) for ({ca}, {cb}) outside pair band {list(k_rng)}",
                        key=f"pairs.{pk}.mu_kinetic",
                    )
                if pb.get("restitution_check_active") and pb.get("restitution") is not None:
                    # Check all co-occurring asset-to-asset / asset-to-surface / robot-link realizations of (ca, cb).
                    # Co-occurrence is derived directly from the reviewed tasks' spawned objects (assets.*.tasks + robots),
                    # never from an unvalidated can_form_in_task flag (#175 review item 3).
                    side_a_mats = _side_realizations(ca, ma)
                    side_b_mats = _side_realizations(cb, mb)
                    for mat_a_i, dyn_a, sealed_a, a_rec_i in side_a_mats:
                        for mat_b_i, dyn_b, sealed_b, b_rec_i in side_b_mats:
                            if not self._assets_can_cooccur(a_rec_i, b_rec_i, dyn_a=dyn_a, dyn_b=dyn_b):
                                continue
                            r_rng = pb["restitution"] if (sealed_a or sealed_b) else (pb.get("restitution_unfilled") or pb["restitution"])
                            eff_r, mode_r = combine_values(
                                mat_a_i.restitution,
                                mat_b_i.restitution,
                                mat_a_i.restitution_combine_mode,
                                mat_b_i.restitution_combine_mode,
                            )
                            if not (r_rng[0] - 1e-6 <= eff_r <= r_rng[1] + 1e-6):
                                raise PhysicsSpecError(
                                    f"Combined restitution {eff_r:.4f} (mode={mode_r}) for ({ca}, {cb}) outside >0.5 m/s pair band {list(r_rng)}",
                                    key=f"pairs.{pk}.restitution",
                                )

    @staticmethod
    def _asset_has_enabled_collider(a_rec: Mapping[str, Any] | None) -> bool:
        if a_rec is None:
            return True
        if "visual_marker_no_collider" in (a_rec.get("flags") or ()):
            return False
        cspec = a_rec.get("collision_spec")
        if cspec is not None and cspec.enabled is False:
            return False
        return True

    def _assets_can_cooccur(
        self,
        a_rec: Mapping[str, Any] | None,
        b_rec: Mapping[str, Any] | None,
        *,
        dyn_a: bool,
        dyn_b: bool,
    ) -> bool:
        """Return True iff collider realizations ``a_rec`` and ``b_rec`` can co-occur in at least one task."""
        if not (dyn_a or dyn_b):
            return False
        if not self._asset_has_enabled_collider(a_rec) or not self._asset_has_enabled_collider(b_rec):
            return False
        if a_rec is None or b_rec is None:
            return True
        shared = a_rec["tasks_set"] & b_rec["tasks_set"]
        if not shared:
            return False
        fl_a = a_rec.get("flags") or ()
        fl_b = b_rec.get("flags") or ()
        if a_rec["id"] == b_rec["id"]:
            if a_rec.get("is_static", False) or "no_self_contact" in fl_a:
                return False
            return True
        if (
            "single_instance_category" in fl_a
            and "single_instance_category" in fl_b
            and a_rec.get("category") is not None
            and a_rec.get("category") == b_rec.get("category")
            and not a_rec.get("is_static", False)
            and not b_rec.get("is_static", False)
        ):
            return False
        return True

    def _derive_task_pair_cooccurrence(self, data: Mapping[str, Any]) -> tuple[set[str], set[str]] | None:
        """Derive formable class-pair and override-pair sets conservatively from ``assets.*.tasks`` and ``robots``.

        An asset omitting ``tasks`` carries ``tasks_set = self.all_known_tasks`` and counts as possibly meeting every
        task's objects, so missing task metadata on one asset never disables ``can_form_in_task`` validation globally
        (#175 re-review item 3 gap 2). Returns ``None`` only when ``self.assets`` is empty.
        """
        if not self.assets:
            return None
        all_tasks = list(self.all_known_tasks)
        pad_cands = sorted({c for c in ("pad_hard", "pad_elastomer", self.scenario.get("finger_pad")) if isinstance(c, str) and c in self.class_materials})
        tab_cands = sorted({c for c in ("table_laminate", "table_wood", self.scenario.get("table")) if isinstance(c, str) and c in self.class_materials})
        if not pad_cands:
            pad_cands = [""]
        if not tab_cands:
            tab_cands = [""]

        robots_raw = data.get("robots") if isinstance(data, Mapping) else None
        robot_non_pad_classes: list[str] = []
        has_robot_pads = False
        if isinstance(robots_raw, Mapping):
            for rdict in robots_raw.values():
                if not isinstance(rdict, Mapping):
                    continue
                if rdict.get("pad_material_class") or self.scenario.get("finger_pad"):
                    has_robot_pads = True
                npl = rdict.get("non_pad_link_material_classes")
                if isinstance(npl, Mapping):
                    for cleaf in npl.values():
                        cval = cleaf.get("value") if isinstance(cleaf, Mapping) else cleaf
                        if isinstance(cval, str) and cval in self.class_materials:
                            robot_non_pad_classes.append(cval)

        derived_cp: set[str] = set()
        derived_op: set[str] = set()
        for t in all_tasks:
            task_assets = [
                a for a in self.assets.values()
                if t in a["tasks_set"] and self._asset_has_enabled_collider(a)
            ]
            for pad_c in pad_cands:
                for tab_c in tab_cands:
                    # Tuple: (carrier, is_dynamic, asset_rec_or_None)
                    bodies: list[tuple[tuple[str, str], bool, Mapping[str, Any] | None]] = []
                    for _arm in range(2):
                        for rcls in robot_non_pad_classes:
                            bodies.append((("class", rcls), True, None))
                        if has_robot_pads and pad_c:
                            bodies.append((("class", pad_c), True, None))
                            bodies.append((("class", pad_c), True, None))
                    for a in task_assets:
                        is_dyn = not a.get("is_static", False)
                        if a["id"] == "Scene/Table" and tab_c:
                            car = ("class", tab_c)
                        elif a.get("has_material_override"):
                            car = ("override", a["id"])
                        elif a.get("material_class") is not None:
                            car = ("class", str(a["material_class"]))
                        else:
                            continue
                        bodies.append((car, is_dyn, a))
                        if is_dyn and "no_self_contact" not in (a.get("flags") or ()):
                            bodies.append((car, True, a))
                    nb = len(bodies)
                    for i in range(nb):
                        m1, d1, rec1 = bodies[i]
                        for j in range(i + 1, nb):
                            m2, d2, rec2 = bodies[j]
                            if not self._assets_can_cooccur(rec1, rec2, dyn_a=d1, dyn_b=d2):
                                continue
                            if m1[0] == "class" and m2[0] == "class":
                                derived_cp.add(f"{min(m1[1], m2[1])}__{max(m1[1], m2[1])}")
                            elif m1[0] == "override" and m2[0] == "class":
                                derived_op.add(f"{m1[1]}__{m2[1]}")
                            elif m2[0] == "override" and m1[0] == "class":
                                derived_op.add(f"{m2[1]}__{m1[1]}")
        return derived_cp, derived_op

    def _load_and_verify_realised_vs_nominal(self, data: Mapping[str, Any]) -> dict[str, Any]:
        """Compute realised-vs-nominal pair table from loaded materials and verify any table-supplied block."""
        fill_mode = str(self.scenario.get("fill", self.scenario.get("bottle_fill", "empty")))
        computed_class_pairs: dict[str, dict[str, Any]] = {}
        for (ca, cb), pb in sorted(self.pair_bands.items(), key=lambda kv: kv[1]["key"]):
            if ca not in self.class_materials or cb not in self.class_materials:
                continue
            ma = self.class_materials[ca]
            mb = self.class_materials[cb]
            ra = self.class_full_restitutions[ca][0] if (fill_mode == "full" and ca in self.class_full_restitutions) else ma.restitution
            rb = self.class_full_restitutions[cb][0] if (fill_mode == "full" and cb in self.class_full_restitutions) else mb.restitution
            eff_s, mode_s = combine_values(ma.static_friction, mb.static_friction, ma.friction_combine_mode, mb.friction_combine_mode)
            eff_k, _ = combine_values(ma.dynamic_friction, mb.dynamic_friction, ma.friction_combine_mode, mb.friction_combine_mode)
            eff_r, mode_r = combine_values(ra, rb, ma.restitution_combine_mode, mb.restitution_combine_mode)
            s_nom, k_nom = float(pb["nominal_static"]), float(pb["nominal_kinetic"])
            s_rng, k_rng = pb["mu_static"], pb["mu_kinetic"]
            pk = pb["key"]
            row: dict[str, Any] = {
                "a": ca,
                "b": cb,
                "scenario_membership": pb["scenario_membership"],
                "friction_combine_mode": mode_s,
                "mu_static": {
                    "nominal": round(s_nom, 6),
                    "realised": round(eff_s, 6),
                    "delta": round(eff_s - s_nom, 6),
                    "band": [round(s_rng[0], 6), round(s_rng[1], 6)],
                    "in_band": bool(s_rng[0] - 1e-6 <= eff_s <= s_rng[1] + 1e-6),
                },
                "mu_kinetic": {
                    "nominal": round(k_nom, 6),
                    "realised": round(eff_k, 6),
                    "delta": round(eff_k - k_nom, 6),
                    "band": [round(k_rng[0], 6), round(k_rng[1], 6)],
                    "in_band": bool(k_rng[0] - 1e-6 <= eff_k <= k_rng[1] + 1e-6),
                },
            }
            if pb.get("restitution") is not None and pb.get("nominal_restitution") is not None:
                r_nom = float(pb["nominal_restitution"])
                r_rng = pb["restitution"]
                row["restitution"] = {
                    "combine_mode": mode_r,
                    "nominal": round(r_nom, 6),
                    "realised": round(eff_r, 6),
                    "delta": round(eff_r - r_nom, 6),
                    "band": [round(r_rng[0], 6), round(r_rng[1], 6)],
                    "check_active": bool(pb.get("restitution_check_active")),
                    "in_band": bool(r_rng[0] - 1e-6 <= eff_r <= r_rng[1] + 1e-6),
                }
            computed_class_pairs[pk] = row

        computed_override_pairs: dict[str, dict[str, Any]] = {}
        for aid, aparsed in sorted(self.assets.items()):
            if not aparsed.get("has_material_override") or aparsed.get("material") is None:
                continue
            acls = aparsed.get("material_class")
            amat: MaterialSpec = aparsed["material"]
            no_self = "no_self_contact" in (aparsed.get("flags") or ())
            for partner_cls, pmat in sorted(self.class_materials.items()):
                if acls is not None and partner_cls == acls and no_self:
                    continue
                canon = (min(str(acls), partner_cls), max(str(acls), partner_cls)) if acls else None
                pb = self.pair_bands.get(canon) if canon is not None else None
                if pb is None:
                    continue
                pr = self.class_full_restitutions[partner_cls][0] if (fill_mode == "full" and partner_cls in self.class_full_restitutions) else pmat.restitution
                eff_s, mode_s = combine_values(amat.static_friction, pmat.static_friction, amat.friction_combine_mode, pmat.friction_combine_mode)
                eff_k, _ = combine_values(amat.dynamic_friction, pmat.dynamic_friction, amat.friction_combine_mode, pmat.friction_combine_mode)
                eff_r, mode_r = combine_values(amat.restitution, pr, amat.restitution_combine_mode, pmat.restitution_combine_mode)
                s_rng, k_rng = pb["mu_static"], pb["mu_kinetic"]
                if not (s_rng[0] - 1e-6 <= eff_s <= s_rng[1] + 1e-6):
                    raise PhysicsSpecError(
                        f"Override static friction {eff_s:.4f} for ({aid}, {partner_cls}) outside band {list(s_rng)}",
                        key=f"assets.{aid}.material_override",
                    )
                if not (k_rng[0] - 1e-6 <= eff_k <= k_rng[1] + 1e-6):
                    raise PhysicsSpecError(
                        f"Override dynamic friction {eff_k:.4f} for ({aid}, {partner_cls}) outside band {list(k_rng)}",
                        key=f"assets.{aid}.material_override",
                    )
                opk = f"{aid}__{partner_cls}"
                s_nom, k_nom = float(pb["nominal_static"]), float(pb["nominal_kinetic"])
                orow: dict[str, Any] = {
                    "asset_id": aid,
                    "asset_class": acls,
                    "partner_class": partner_cls,
                    "band_pair_key": pb["key"],
                    "scenario_membership": pb["scenario_membership"],
                    "friction_combine_mode": mode_s,
                    "mu_static": {
                        "class_pair_nominal": round(s_nom, 6),
                        "realised": round(eff_s, 6),
                        "delta_vs_class_nominal": round(eff_s - s_nom, 6),
                        "band": [round(s_rng[0], 6), round(s_rng[1], 6)],
                        "in_band": True,
                    },
                    "mu_kinetic": {
                        "class_pair_nominal": round(k_nom, 6),
                        "realised": round(eff_k, 6),
                        "delta_vs_class_nominal": round(eff_k - k_nom, 6),
                        "band": [round(k_rng[0], 6), round(k_rng[1], 6)],
                        "in_band": True,
                    },
                }
                if pb.get("restitution") is not None and pb.get("nominal_restitution") is not None:
                    r_nom = float(pb["nominal_restitution"])
                    r_rng = pb["restitution"]
                    orow["restitution"] = {
                        "combine_mode": mode_r,
                        "class_pair_nominal": round(r_nom, 6),
                        "realised": round(eff_r, 6),
                        "delta_vs_class_nominal": round(eff_r - r_nom, 6),
                        "band": [round(r_rng[0], 6), round(r_rng[1], 6)],
                        "check_active": bool(pb.get("restitution_check_active")),
                        "in_band": bool(r_rng[0] - 1e-6 <= eff_r <= r_rng[1] + 1e-6),
                    }
                computed_override_pairs[opk] = orow

        derived_coocc = self._derive_task_pair_cooccurrence(data)
        if derived_coocc is not None:
            for pk_c, crow in computed_class_pairs.items():
                crow["can_form_in_task"] = pk_c in derived_coocc[0]
            for opk_c, corow in computed_override_pairs.items():
                corow["can_form_in_task"] = opk_c in derived_coocc[1]

        supplied = data.get("realised_vs_nominal")
        if supplied is not None:
            if not isinstance(supplied, Mapping) or not isinstance(supplied.get("class_pairs"), Mapping):
                raise PhysicsSpecError(
                    "'realised_vs_nominal' must be a mapping containing 'class_pairs'",
                    key="realised_vs_nominal",
                )
            sup_cp = supplied["class_pairs"]
            if set(sup_cp.keys()) != set(computed_class_pairs.keys()):
                raise PhysicsSpecError(
                    f"realised_vs_nominal.class_pairs keys mismatch loaded pairs: "
                    f"missing={sorted(set(computed_class_pairs) - set(sup_cp))}, extra={sorted(set(sup_cp) - set(computed_class_pairs))}",
                    key="realised_vs_nominal.class_pairs",
                )
            for pk, comp_row in computed_class_pairs.items():
                srow = sup_cp[pk]
                for fk in ("mu_static", "mu_kinetic"):
                    if abs(float(srow[fk]["realised"]) - float(comp_row[fk]["realised"])) > 1e-5:
                        raise PhysicsSpecError(
                            f"realised_vs_nominal.class_pairs.{pk}.{fk}.realised={srow[fk]['realised']} != computed {comp_row[fk]['realised']}",
                            key=f"realised_vs_nominal.class_pairs.{pk}.{fk}",
                        )
                if "can_form_in_task" in srow:
                    cf_key = f"realised_vs_nominal.class_pairs.{pk}.can_form_in_task"
                    if derived_coocc is None:
                        raise PhysicsSpecError(
                            f"{cf_key} cannot be verified because 'assets' is empty",
                            key=cf_key,
                        )
                    cf_val = _require_bool(srow["can_form_in_task"], name=cf_key)
                    exp_cf = pk in derived_coocc[0]
                    if cf_val != exp_cf:
                        raise PhysicsSpecError(
                            f"{cf_key}={srow['can_form_in_task']!r} contradicts task-object co-occurrence ({exp_cf})",
                            key=cf_key,
                        )
            sup_op = supplied.get("asset_override_pairs")
            if sup_op is not None:
                if not isinstance(sup_op, Mapping):
                    raise PhysicsSpecError(
                        "'realised_vs_nominal.asset_override_pairs' must be a mapping",
                        key="realised_vs_nominal.asset_override_pairs",
                    )
                for opk, sorow in sup_op.items():
                    if not isinstance(sorow, Mapping):
                        raise PhysicsSpecError(
                            f"realised_vs_nominal.asset_override_pairs.{opk} must be a mapping",
                            key=f"realised_vs_nominal.asset_override_pairs.{opk}",
                        )
                    if "can_form_in_task" in sorow:
                        ocf_key = f"realised_vs_nominal.asset_override_pairs.{opk}.can_form_in_task"
                        if derived_coocc is None:
                            raise PhysicsSpecError(
                                f"{ocf_key} cannot be verified because 'assets' is empty",
                                key=ocf_key,
                            )
                        ocf_val = _require_bool(sorow["can_form_in_task"], name=ocf_key)
                        exp_ocf = str(opk) in derived_coocc[1]
                        if ocf_val != exp_ocf:
                            raise PhysicsSpecError(
                                f"{ocf_key}={sorow['can_form_in_task']!r} contradicts task-object co-occurrence ({exp_ocf})",
                                key=ocf_key,
                            )
            if fill_mode == str(supplied.get("fill", "empty")):
                return dict(supplied)

        return {
            "fill": fill_mode,
            "class_pairs": computed_class_pairs,
            "asset_override_pairs": computed_override_pairs,
        }

    def _load_models_base(self, data: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
        models_raw = data.get("models")
        if models_raw is None:
            return {}
        model_items: list[Mapping[str, Any]] = []
        if isinstance(models_raw, list):
            for idx, m in enumerate(models_raw):
                if not isinstance(m, Mapping):
                    raise PhysicsSpecError(f"models[{idx}] must be a mapping", key=f"models[{idx}]")
                model_items.append(m)
        elif isinstance(models_raw, Mapping):
            for mid, m in models_raw.items():
                if not isinstance(m, Mapping):
                    raise PhysicsSpecError(f"models.{mid} must be a mapping", key=f"models.{mid}")
                model_items.append({"id": mid, **m})
        else:
            raise PhysicsSpecError("'models' must be a list or mapping", key="models")

        out: dict[str, dict[str, Any]] = {}
        for idx, m in enumerate(model_items):
            mid = m.get("id")
            if not isinstance(mid, str) or not mid.strip():
                raise PhysicsSpecError(f"models[{idx}] missing non-empty string 'id'", key=f"models[{idx}].id")
            unknown_m = sorted(set(m.keys()) - _ALLOWED_MODEL_KEYS)
            if unknown_m:
                raise PhysicsSpecError(
                    f"Unknown key(s) {unknown_m} in model '{mid}'",
                    key=f"models.{mid}.{unknown_m[0]}",
                )
            base_entry: dict[str, Any] = {
                "id": mid,
                "type": m.get("type"),
                "category": m.get("category"),
                "material_class": m.get("material_class"),
                "alternative_material_classes": list(m.get("alternative_material_classes") or []),
                "confidence": m.get("confidence"),
                "identity": m.get("identity"),
                "flags": list(m.get("flags") or []),
            }
            if m.get("bbox_extents_m") is not None:
                ext = _require_vec3_pos(m["bbox_extents_m"], name=f"models.{mid}.bbox_extents_m")
                base_entry["size_m"] = {
                    "value": list(ext),
                    "unit": "m",
                    "status": m.get("size_status", "unsourced"),
                }
            out[mid] = base_entry
        return out

    def _load_and_verify_assets(self, data: Mapping[str, Any]) -> None:
        base_models = self._load_models_base(data)
        assets_raw = data.get("assets") or {}
        if not isinstance(assets_raw, Mapping):
            raise PhysicsSpecError("'assets' must be a mapping", key="assets")
        fill_mode = str(self.scenario.get("fill", self.scenario.get("bottle_fill", "empty")))

        combined_assets: dict[str, Mapping[str, Any]] = dict(base_models)
        for aid, adict in assets_raw.items():
            if not isinstance(adict, Mapping):
                raise PhysicsSpecError(f"Asset '{aid}' must be a mapping", key=f"assets.{aid}")
            unknown = sorted(set(adict.keys()) - _ALLOWED_ASSET_KEYS)
            if unknown:
                raise PhysicsSpecError(f"Unknown key(s) {unknown} in asset '{aid}'", key=f"assets.{aid}.{unknown[0]}")
            if str(aid) in combined_assets:
                merged_a = dict(combined_assets[str(aid)])
                merged_a.update(adict)
                combined_assets[str(aid)] = merged_a
            else:
                combined_assets[str(aid)] = adict

        for aid, adict in combined_assets.items():
            mcls = adict.get("material_class")
            atype_prefix = str(aid).split("/", 1)[0]
            is_out_of_scope_model = atype_prefix in ("Fluid", "Garment") or adict.get("type") in ("Fluid", "Garment")
            if mcls is not None and not is_out_of_scope_model:
                if mcls not in self.class_materials and "material_override" not in adict:
                    raise PhysicsSpecError(
                        f"Asset '{aid}' references unknown material_class={mcls!r}",
                        key=f"assets.{aid}.material_class",
                    )
            elif mcls is not None and is_out_of_scope_model and self.known_classes and mcls not in self.known_classes:
                raise PhysicsSpecError(
                    f"Asset '{aid}' references unknown material_class={mcls!r}",
                    key=f"assets.{aid}.material_class",
                )

            is_static_asset = str(aid).startswith(("Geometry/", "Scene/Ground", "Scene/Room")) or (
                str(aid) == "Scene/Table" and adict.get("type", "static") == "static"
            )
            tasks_raw = adict.get("tasks")
            if tasks_raw is not None:
                if (
                    not isinstance(tasks_raw, list)
                    or not tasks_raw
                    or not all(isinstance(t, str) and t.strip() for t in tasks_raw)
                ):
                    raise PhysicsSpecError(
                        f"assets.{aid}.tasks must be a non-empty list of task names",
                        key=f"assets.{aid}.tasks",
                    )
                tasks_explicit: frozenset[str] | None = frozenset(t.strip() for t in tasks_raw)
                if self.reviewed_tasks:
                    unknown_t = sorted(tasks_explicit - set(self.reviewed_tasks))
                    if unknown_t:
                        raise PhysicsSpecError(
                            f"assets.{aid}.tasks references unreviewed task(s) {unknown_t}",
                            key=f"assets.{aid}.tasks",
                        )
                tasks_set = tasks_explicit
            else:
                tasks_explicit = None
                tasks_set = frozenset(self.reviewed_tasks) if self.reviewed_tasks else frozenset({"__unscoped__"})

            parsed: dict[str, Any] = {
                "id": str(aid),
                "category": adict.get("category"),
                "material_class": mcls,
                "flags": list(adict.get("flags") or []),
                "is_static": is_static_asset,
                "tasks_explicit": tasks_explicit,
                "tasks_set": tasks_set,
            }

            if "size_m" in adict and adict["size_m"] is not None:
                ext, _ = _parse_scalar_leaf(
                    adict["size_m"],
                    key=f"assets.{aid}.size_m",
                    parser_fn=lambda v, nm: _require_vec3_pos(v, name=nm),
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )
                parsed["size_m"] = ext

            if "collider_extents_m" in adict and adict["collider_extents_m"] is not None:
                col_ext, _ = _parse_scalar_leaf(
                    adict["collider_extents_m"],
                    key=f"assets.{aid}.collider_extents_m",
                    parser_fn=lambda v, nm: _require_vec3_pos(v, name=nm),
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )
                parsed["collider_extents_m"] = col_ext

            sdec = adict.get("size_decision", "asset")
            if sdec not in ("asset", "declaration_wrong", "scale"):
                raise PhysicsSpecError(f"Invalid size_decision={sdec!r} on '{aid}'", key=f"assets.{aid}.size_decision")
            parsed["size_decision"] = sdec

            if is_static_asset and ("mass_kg" in adict or "mass_by_fill" in adict):
                raise PhysicsSpecError(f"mass_kg is forbidden on static asset '{aid}'", key=f"assets.{aid}.mass_kg")

            asset_fill = (
                "empty"
                if (fill_mode != "empty" and self.fill_tasks is not None and not (tasks_set & self.fill_tasks))
                else fill_mode
            )

            mass_val: float | None = None
            if "mass_kg" in adict and adict["mass_kg"] is not None:
                mass_val, _, _ = _validate_band_entry(
                    adict["mass_kg"],
                    key=f"assets.{aid}.mass_kg",
                    positive=True,
                    fill_mode=asset_fill,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )
            elif "mass_by_fill" in adict and adict["mass_by_fill"] is not None:
                mbf = adict["mass_by_fill"]
                if not isinstance(mbf, Mapping) or asset_fill not in mbf:
                    raise PhysicsSpecError(f"Asset '{aid}' mass_by_fill missing fill={asset_fill!r}", key=f"assets.{aid}.mass_by_fill")
                mass_val, _, _ = _validate_band_entry(
                    mbf[asset_fill],
                    key=f"assets.{aid}.mass_by_fill.{asset_fill}",
                    positive=True,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )
            parsed["mass_kg"] = mass_val

            if "inertia_over_mass_by_fill" in adict and adict["inertia_over_mass_by_fill"] is not None:
                if is_static_asset:
                    raise PhysicsSpecError(f"inertia_over_mass_by_fill is forbidden on static asset '{aid}'", key=f"assets.{aid}.inertia_over_mass_by_fill")
                ibf = adict["inertia_over_mass_by_fill"]
                if not isinstance(ibf, Mapping) or asset_fill not in ibf:
                    raise PhysicsSpecError(
                        f"Asset '{aid}' inertia_over_mass_by_fill missing fill={asset_fill!r}",
                        key=f"assets.{aid}.inertia_over_mass_by_fill",
                    )
                iom, _ = _parse_scalar_leaf(
                    ibf[asset_fill],
                    key=f"assets.{aid}.inertia_over_mass_by_fill.{asset_fill}",
                    parser_fn=lambda v, nm: _require_vec3_pos(v, name=nm),
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )
                parsed["inertia_over_mass_m2"] = iom
            else:
                parsed["inertia_over_mass_m2"] = None

            if "com_offset_by_fill" in adict and adict["com_offset_by_fill"] is not None:
                if is_static_asset:
                    raise PhysicsSpecError(f"com_offset_by_fill is forbidden on static asset '{aid}'", key=f"assets.{aid}.com_offset_by_fill")
                cbf = adict["com_offset_by_fill"]
                if not isinstance(cbf, Mapping) or asset_fill not in cbf:
                    raise PhysicsSpecError(
                        f"Asset '{aid}' com_offset_by_fill missing fill={asset_fill!r}",
                        key=f"assets.{aid}.com_offset_by_fill",
                    )

                def _vec3_com(v: Any, nm: str) -> tuple[float, float, float]:
                    if not isinstance(v, Sequence) or isinstance(v, (str, bytes)) or len(v) != 3:
                        raise PhysicsSpecError(f"Expected 3-element vector for '{nm}', got {v!r}", key=nm)
                    t = tuple(_require_finite_float(x, name=nm) for x in v)
                    return (t[0], t[1], t[2])

                com_v, _ = _parse_scalar_leaf(
                    cbf[asset_fill],
                    key=f"assets.{aid}.com_offset_by_fill.{asset_fill}",
                    parser_fn=_vec3_com,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )
                parsed["com_offset_m"] = com_v
            else:
                parsed["com_offset_m"] = None

            if "solid_collider_inertia_over_mass_m2" in adict and adict["solid_collider_inertia_over_mass_m2"] is not None:
                sci, _ = _parse_scalar_leaf(
                    adict["solid_collider_inertia_over_mass_m2"],
                    key=f"assets.{aid}.solid_collider_inertia_over_mass_m2",
                    parser_fn=lambda v, nm: _require_vec3_pos(v, name=nm),
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )
                parsed["solid_collider_inertia_over_mass_m2"] = sci
            else:
                parsed["solid_collider_inertia_over_mass_m2"] = None

            asset_shell_band: tuple[float, float] | None = None
            if "shell_density_kg_m3" in adict and adict["shell_density_kg_m3"] is not None:
                _, asset_shell_band, _ = _validate_band_entry(
                    adict["shell_density_kg_m3"],
                    key=f"assets.{aid}.shell_density_kg_m3",
                    positive=True,
                    fill_mode=fill_mode,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )

            # Density checks use collider_extents_m when present, else size_m (what PhysX integrates).
            dens_ext = parsed.get("collider_extents_m") or parsed.get("size_m")
            if (
                mass_val is not None
                and dens_ext is not None
                and mcls in self.class_densities
                and "density_exempt" not in parsed["flags"]
            ):
                sx, sy, sz = dens_ext
                vol = sx * sy * sz
                implied_rho = mass_val / vol
                _, (rho_lo, rho_hi), _ = self.class_densities[mcls]
                if "hollow_contents_dependent" in parsed["flags"]:
                    if asset_shell_band is not None:
                        sh_lo, sh_hi = asset_shell_band
                    elif mcls in self.class_shell_densities:
                        _, (sh_lo, sh_hi), _ = self.class_shell_densities[mcls]
                    else:
                        # Thin-wall empty shell (>= 5 kg/m^3 effective bbox density) up to liquid-filled vessel (<= max(rho_hi, 1200)).
                        sh_lo, sh_hi = 5.0, max(rho_hi, 1200.0)
                    if implied_rho > sh_hi * 1.001 or implied_rho < sh_lo * 0.999:
                        raise PhysicsSpecError(
                            f"Hollow asset '{aid}' implied bbox density {implied_rho:.1f} kg/m^3 outside shell band [{sh_lo}, {sh_hi}] kg/m^3",
                            key=f"assets.{aid}.mass_kg",
                        )
                else:
                    if implied_rho > rho_hi * 1.001:
                        raise PhysicsSpecError(
                            f"Asset '{aid}' implied bbox density {implied_rho:.1f} kg/m^3 exceeds solid '{mcls}' upper bound {rho_hi} kg/m^3",
                            key=f"assets.{aid}.mass_kg",
                        )
                    if implied_rho < rho_lo * 0.05:
                        raise PhysicsSpecError(
                            f"Asset '{aid}' implied bbox density {implied_rho:.1f} kg/m^3 below solid '{mcls}' lower bound (0.05 * {rho_lo} = {0.05 * rho_lo:.1f} kg/m^3)",
                            key=f"assets.{aid}.mass_kg",
                        )

            is_sealed_container = (
                (not is_static_asset)
                and ("hollow_contents_dependent" in parsed["flags"])
                and ("open_receptacle" not in parsed["flags"])
            )

            if "material_override" in adict and adict["material_override"] is not None:
                mat = _parse_material_dict(
                    adict["material_override"],
                    key=f"assets.{aid}.material_override",
                    fill_mode=asset_fill if is_sealed_container else "empty",
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )
                parsed["material"] = mat
                parsed["has_material_override"] = True
                if mcls is not None:
                    for (ca, cb), bands in self.pair_bands.items():
                        if ca == mcls or cb == mcls:
                            other = cb if ca == mcls else ca
                            if other == mcls and "no_self_contact" in parsed["flags"]:
                                continue
                            if other in self.class_materials:
                                mo = self.class_materials[other]
                                es, _ = combine_values(
                                    mat.static_friction, mo.static_friction, mat.friction_combine_mode, mo.friction_combine_mode
                                )
                                ek, _ = combine_values(
                                    mat.dynamic_friction, mo.dynamic_friction, mat.friction_combine_mode, mo.friction_combine_mode
                                )
                                sr, kr = bands["mu_static"], bands["mu_kinetic"]
                                if not (sr[0] - 1e-6 <= es <= sr[1] + 1e-6) or not (kr[0] - 1e-6 <= ek <= kr[1] + 1e-6):
                                    raise PhysicsSpecError(
                                        f"Asset '{aid}' material_override violates pair band ({mcls}, {other})",
                                        key=f"assets.{aid}.material_override",
                                    )
            elif str(aid) == "Scene/Table" and mcls is None:
                tcls = str(self.scenario.get("table", "table_laminate"))
                if tcls in self.class_materials:
                    parsed["material"] = self.class_materials[tcls]
                    parsed["material_class"] = tcls
            elif mcls is not None:
                effective_mcls = str(self.scenario.get("table", mcls)) if str(aid) == "Scene/Table" else str(mcls)
                if effective_mcls in self.class_materials:
                    base_mat = self.class_materials[effective_mcls]
                    if asset_fill == "full" and is_sealed_container and effective_mcls in self.class_full_restitutions:
                        base_mat = MaterialSpec(
                            static_friction=base_mat.static_friction,
                            dynamic_friction=base_mat.dynamic_friction,
                            restitution=self.class_full_restitutions[effective_mcls][0],
                            friction_combine_mode=base_mat.friction_combine_mode,
                            restitution_combine_mode=base_mat.restitution_combine_mode,
                        )
                    parsed["material"] = base_mat
                    parsed["material_class"] = effective_mcls

            if adict.get("collision") is not None:
                default_app = "none" if str(aid) == "Scene/Table" else "asset"
                default_en = True if str(aid) == "Scene/Table" else None
                parsed["collision_spec"] = _parse_collision_dict(
                    adict["collision"],
                    key=f"assets.{aid}.collision",
                    default_enabled=default_en,
                    default_approx=default_app,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                )
            else:
                parsed["collision_spec"] = None

            if adict.get("fixed_base") is not None:
                fb_val, _ = _parse_scalar_leaf(
                    adict["fixed_base"],
                    key=f"assets.{aid}.fixed_base",
                    parser_fn=lambda v, nm: _require_bool(v, name=nm),
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                )
                parsed["fixed_base"] = fb_val
            else:
                parsed["fixed_base"] = None

            if adict.get("reset_drive") is not None:
                rd_val, _ = _parse_scalar_leaf(
                    adict["reset_drive"],
                    key=f"assets.{aid}.reset_drive",
                    parser_fn=lambda v, nm: _require_bool(v, name=nm),
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                )
                parsed["reset_drive"] = rd_val
            else:
                parsed["reset_drive"] = None

            lm_raw = adict.get("link_masses_kg") or {}
            if not isinstance(lm_raw, Mapping):
                raise PhysicsSpecError(f"assets.{aid}.link_masses_kg must be a mapping", key=f"assets.{aid}.link_masses_kg")
            parsed_link_masses: dict[str, float] = {}
            for lk, lv in lm_raw.items():
                lkey = f"assets.{aid}.link_masses_kg.{lk}"
                if isinstance(lv, Mapping) and "range" in lv:
                    m_lk, _, _ = _validate_band_entry(
                        lv,
                        key=lkey,
                        positive=True,
                        allowed_statuses=self.allowed_statuses,
                        allow_unsourced_keys=self.allow_unsourced_keys,
                        status_counts=self.status_counts,
                        unsourced_keys_used=self.unsourced_keys_used,
                    )
                else:
                    def _pos_mass(v: Any, nm: str) -> float:
                        fv = _require_finite_float(v, name=nm)
                        if fv <= 0.0:
                            raise PhysicsSpecError(f"Expected positive link mass for '{nm}', got {fv}", key=nm)
                        return fv

                    m_lk, _ = _parse_scalar_leaf(
                        lv,
                        key=lkey,
                        parser_fn=_pos_mass,
                        allowed_statuses=self.allowed_statuses,
                        allow_unsourced_keys=self.allow_unsourced_keys,
                        status_counts=self.status_counts,
                        unsourced_keys_used=self.unsourced_keys_used,
                    )
                parsed_link_masses[str(lk)] = m_lk
            parsed["link_masses_kg"] = parsed_link_masses

            lmat_raw = adict.get("link_materials") or {}
            if not isinstance(lmat_raw, Mapping):
                raise PhysicsSpecError(f"assets.{aid}.link_materials must be a mapping", key=f"assets.{aid}.link_materials")
            parsed["link_materials"] = {
                str(lk): _parse_material_dict(
                    mv,
                    key=f"assets.{aid}.link_materials.{lk}",
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                )
                for lk, mv in lmat_raw.items()
            }

            jdrv_raw = adict.get("joint_drives") or {}
            if not isinstance(jdrv_raw, Mapping):
                raise PhysicsSpecError(f"assets.{aid}.joint_drives must be a mapping", key=f"assets.{aid}.joint_drives")
            parsed["joint_drives"] = {
                str(jk): _parse_drive_dict(
                    dv,
                    key=f"assets.{aid}.joint_drives.{jk}",
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                )
                for jk, dv in jdrv_raw.items()
            }

            if adict.get("belt_velocity_m_s") is not None:
                def _vec3_finite(v: Any, nm: str) -> tuple[float, float, float]:
                    if not isinstance(v, Sequence) or isinstance(v, (str, bytes)) or len(v) != 3:
                        raise PhysicsSpecError(f"Expected 3-element vector for '{nm}', got {v!r}", key=nm)
                    t = tuple(_require_finite_float(x, name=nm) for x in v)
                    return (t[0], t[1], t[2])

                bv_val, _ = _parse_scalar_leaf(
                    adict["belt_velocity_m_s"],
                    key=f"assets.{aid}.belt_velocity_m_s",
                    parser_fn=_vec3_finite,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                )
                parsed["belt_velocity_m_s"] = bv_val
            else:
                parsed["belt_velocity_m_s"] = None

            variants: dict[str, dict[str, Any]] = {}
            tv_raw = adict.get("task_variants") or {}
            if not isinstance(tv_raw, Mapping):
                raise PhysicsSpecError(f"assets.{aid}.task_variants must be a mapping", key=f"assets.{aid}.task_variants")
            for tname, tv in tv_raw.items():
                if not isinstance(tv, Mapping) or not str(tv.get("reason", "")).strip():
                    raise PhysicsSpecError(
                        f"Asset '{aid}' task_variants.{tname} requires a non-empty 'reason'",
                        key=f"assets.{aid}.task_variants.{tname}",
                    )
                unknown_tv = sorted(set(tv.keys()) - {"reason", "mass_kg", "size_m", "notes"})
                if unknown_tv:
                    raise PhysicsSpecError(
                        f"Unknown key(s) {unknown_tv} in assets.{aid}.task_variants.{tname}",
                        key=f"assets.{aid}.task_variants.{tname}.{unknown_tv[0]}",
                    )
                vt: dict[str, Any] = {"reason": str(tv["reason"])}
                if "mass_kg" in tv:
                    vt["mass_kg"], _, _ = _validate_band_entry(
                        tv["mass_kg"],
                        key=f"assets.{aid}.task_variants.{tname}.mass_kg",
                        positive=True,
                        allowed_statuses=self.allowed_statuses,
                        allow_unsourced_keys=self.allow_unsourced_keys,
                        status_counts=self.status_counts,
                        unsourced_keys_used=self.unsourced_keys_used,
                    )
                if "size_m" in tv:
                    # Per-task box size (e.g. the table of a task that uses its own layout scale, #167); recorded and
                    # status-gated like the base size_m, which no resolver reads for authoring.
                    vt["size_m"], _ = _parse_scalar_leaf(
                        tv["size_m"],
                        key=f"assets.{aid}.task_variants.{tname}.size_m",
                        parser_fn=lambda v, nm: _require_vec3_pos(v, name=nm),
                        allowed_statuses=self.allowed_statuses,
                        allow_unsourced_keys=self.allow_unsourced_keys,
                        allow_scenario_keys=self.allow_scenario_keys,
                        status_counts=self.status_counts,
                        unsourced_keys_used=self.unsourced_keys_used,
                        scenario_keys_used=self.scenario_keys_used,
                    )
                variants[str(tname)] = vt
            parsed["task_variants"] = variants
            self.assets[str(aid)] = parsed

        known_t: set[str] = set(self.reviewed_tasks.keys())
        for ap in self.assets.values():
            if ap["tasks_explicit"] is not None:
                known_t.update(ap["tasks_explicit"])
        if not known_t:
            known_t.add("__unscoped__")
        self.all_known_tasks = tuple(sorted(known_t))
        all_tasks_frozenset = frozenset(self.all_known_tasks)
        for ap in self.assets.values():
            if ap["tasks_explicit"] is None:
                ap["tasks_set"] = all_tasks_frozenset

    def _load_robots(self, data: Mapping[str, Any]) -> None:
        robots_raw = data.get("robots") or {}
        if not isinstance(robots_raw, Mapping):
            raise PhysicsSpecError("'robots' must be a mapping", key="robots")
        pad_cls_scen = self.scenario.get("finger_pad")

        for rname, rdict in robots_raw.items():
            if not isinstance(rdict, Mapping):
                raise PhysicsSpecError(f"robots.{rname} must be a mapping", key=f"robots.{rname}")
            unauthored = sorted(set(rdict.keys()) & _UNAUTHORED_ROBOT_KEYS)
            if unauthored:
                raise PhysicsSpecError(
                    f"robots.{rname}.{unauthored[0]} is not authored by #62 PR 1 and is rejected until authored",
                    key=f"robots.{rname}.{unauthored[0]}",
                )
            unknown = sorted(set(rdict.keys()) - _ALLOWED_ROBOT_KEYS)
            if unknown:
                raise PhysicsSpecError(f"Unknown key(s) {unknown} in robots.{rname}", key=f"robots.{rname}.{unknown[0]}")

            default_finger_links = ("link7", "link8") if rname == "x5" else ("panda_leftfinger", "panda_rightfinger")
            link_mats: dict[str, MaterialSpec] = {}
            pad_cls = pad_cls_scen or rdict.get("pad_material_class")
            if pad_cls is not None:
                if pad_cls not in self.class_materials:
                    raise PhysicsSpecError(
                        f"robots.{rname} references unknown pad class {pad_cls!r}",
                        key=f"robots.{rname}.pad_material_class",
                    )
                for fl in default_finger_links:
                    link_mats[fl] = self.class_materials[pad_cls]

            npl_raw = rdict.get("non_pad_link_material_classes") or {}
            if not isinstance(npl_raw, Mapping):
                raise PhysicsSpecError(
                    f"robots.{rname}.non_pad_link_material_classes must be a mapping",
                    key=f"robots.{rname}.non_pad_link_material_classes",
                )
            for lk, c_leaf in npl_raw.items():
                lkey = f"robots.{rname}.non_pad_link_material_classes.{lk}"
                if str(lk) in default_finger_links:
                    raise PhysicsSpecError(
                        f"Pad link {lk!r} belongs in pad_material_class / link_materials, not non_pad_link_material_classes",
                        key=lkey,
                    )

                def _req_cls_str(v: Any, nm: str) -> str:
                    if not isinstance(v, str) or not v.strip():
                        raise PhysicsSpecError(f"Expected non-empty class name at '{nm}', got {v!r}", key=nm)
                    return v.strip()

                cls_name, _ = _parse_scalar_leaf(
                    c_leaf,
                    key=lkey,
                    parser_fn=_req_cls_str,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )
                if cls_name not in self.class_materials:
                    raise PhysicsSpecError(
                        f"robots.{rname}.non_pad_link_material_classes.{lk} references unknown material class {cls_name!r}",
                        key=lkey,
                    )
                link_mats[str(lk)] = self.class_materials[cls_name]

            lm_raw = rdict.get("link_materials") or {}
            if not isinstance(lm_raw, Mapping):
                raise PhysicsSpecError(f"robots.{rname}.link_materials must be a mapping", key=f"robots.{rname}.link_materials")
            for lk, mv in lm_raw.items():
                link_mats[str(lk)] = _parse_material_dict(
                    mv,
                    key=f"robots.{rname}.link_materials.{lk}",
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                    scenario_keys_used=self.scenario_keys_used,
                )

            link_inert: dict[str, InertialSpec] = {}
            li_raw = rdict.get("link_inertials") or {}
            if not isinstance(li_raw, Mapping):
                raise PhysicsSpecError(f"robots.{rname}.link_inertials must be a mapping", key=f"robots.{rname}.link_inertials")
            allowed_li = {"mass_kg", "com_offset_m", "inertia_kg_m2", "status", "sources", "conditions", "notes"}
            for lk, iv in li_raw.items():
                likey = f"robots.{rname}.link_inertials.{lk}"
                if not isinstance(iv, Mapping):
                    raise PhysicsSpecError(f"{likey} must be a mapping", key=likey)
                unknown_li = sorted(set(iv.keys()) - allowed_li)
                if unknown_li:
                    raise PhysicsSpecError(f"Unknown key(s) {unknown_li} in {likey}", key=f"{likey}.{unknown_li[0]}")
                ispec = InertialSpec(
                    mass_kg=iv.get("mass_kg"),
                    com_offset_m=iv.get("com_offset_m"),
                    inertia_kg_m2=iv.get("inertia_kg_m2"),
                )
                _check_status(
                    iv.get("status"),
                    key=likey,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                )
                link_inert[str(lk)] = ispec

            def _nonneg_radius(v: Any, nm: str) -> float:
                fv = _require_finite_float(v, name=nm)
                if fv < 0.0:
                    raise PhysicsSpecError(f"Expected non-negative radius for '{nm}', got {fv}", key=nm)
                return fv

            pad_tors: float | None = None
            if rdict.get("pad_torsional_radius_m") is not None:
                tr_raw = rdict["pad_torsional_radius_m"]
                tr_key = f"robots.{rname}.pad_torsional_radius_m"
                if isinstance(tr_raw, Mapping) and "range" in tr_raw:
                    pad_tors, _, _ = _validate_band_entry(
                        tr_raw,
                        key=tr_key,
                        positive=False,
                        allowed_statuses=self.allowed_statuses,
                        allow_unsourced_keys=self.allow_unsourced_keys,
                        status_counts=self.status_counts,
                        unsourced_keys_used=self.unsourced_keys_used,
                    )
                else:
                    pad_tors, _ = _parse_scalar_leaf(
                        tr_raw,
                        key=tr_key,
                        parser_fn=_nonneg_radius,
                        allowed_statuses=self.allowed_statuses,
                        allow_unsourced_keys=self.allow_unsourced_keys,
                        status_counts=self.status_counts,
                        unsourced_keys_used=self.unsourced_keys_used,
                    )

            pad_min_tors: float | None = None
            if rdict.get("pad_min_torsional_radius_m") is not None:
                mtr_raw = rdict["pad_min_torsional_radius_m"]
                mtr_key = f"robots.{rname}.pad_min_torsional_radius_m"
                if isinstance(mtr_raw, Mapping) and "range" in mtr_raw:
                    pad_min_tors, _, _ = _validate_band_entry(
                        mtr_raw,
                        key=mtr_key,
                        positive=False,
                        allowed_statuses=self.allowed_statuses,
                        allow_unsourced_keys=self.allow_unsourced_keys,
                        status_counts=self.status_counts,
                        unsourced_keys_used=self.unsourced_keys_used,
                    )
                else:
                    pad_min_tors, _ = _parse_scalar_leaf(
                        mtr_raw,
                        key=mtr_key,
                        parser_fn=_nonneg_radius,
                        allowed_statuses=self.allowed_statuses,
                        allow_unsourced_keys=self.allow_unsourced_keys,
                        status_counts=self.status_counts,
                        unsourced_keys_used=self.unsourced_keys_used,
                    )

            sanitize_cam = False
            if rdict.get("sanitize_camera_inertials") is not None:
                sanitize_cam, _ = _parse_scalar_leaf(
                    rdict["sanitize_camera_inertials"],
                    key=f"robots.{rname}.sanitize_camera_inertials",
                    parser_fn=lambda v, nm: _require_bool(v, name=nm),
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    status_counts=self.status_counts,
                    unsourced_keys_used=self.unsourced_keys_used,
                )

            gripper_spec = (
                self._parse_gripper(str(rname), rdict["gripper"]) if rdict.get("gripper") is not None else None
            )
            arm_limits_spec = (
                self._parse_arm_limits(str(rname), rdict["arm_limits"]) if rdict.get("arm_limits") is not None else None
            )

            self.robots[str(rname)] = RobotSpec(
                robot_name=str(rname),
                deinstance_fingers=True,
                sanitize_camera_inertials=sanitize_cam,
                pad_links=default_finger_links,
                link_materials=link_mats,
                link_inertials=link_inert,
                pad_torsional_radius_m=pad_tors,
                pad_min_torsional_radius_m=pad_min_tors,
                provenance=f"{self.source_path}:robots.{rname}",
                gripper=gripper_spec,
                arm_limits=arm_limits_spec,
            )

    def _parse_arm_limits(self, rname: str, araw: Any) -> ArmLimitsSpec:
        """Parse ``robots.<rname>.arm_limits`` (#11); scenario key ``arm_effort`` selects nominal or shipped."""
        akey = f"robots.{rname}.arm_limits"
        if not isinstance(araw, Mapping):
            raise PhysicsSpecError(f"{akey} must be a mapping", key=akey)
        unknown = sorted(set(araw.keys()) - _ALLOWED_ARM_LIMITS_KEYS)
        if unknown:
            raise PhysicsSpecError(f"Unknown key(s) {unknown} in {akey}", key=f"{akey}.{unknown[0]}")
        joints = araw.get("joints")
        if not isinstance(joints, Sequence) or isinstance(joints, (str, bytes)):
            raise PhysicsSpecError(f"{akey}.joints must be a list of joint names", key=f"{akey}.joints")
        gate = {
            "allowed_statuses": self.allowed_statuses,
            "allow_unsourced_keys": self.allow_unsourced_keys,
            "allow_scenario_keys": self.allow_scenario_keys,
            "status_counts": self.status_counts,
            "unsourced_keys_used": self.unsourced_keys_used,
            "scenario_keys_used": self.scenario_keys_used,
        }

        def vec(v: Any, nm: str) -> tuple[float, ...]:
            if not isinstance(v, Sequence) or isinstance(v, (str, bytes)) or len(v) != len(joints):
                raise PhysicsSpecError(f"{nm} must have {len(joints)} entries, got {v!r}", key=nm)
            return tuple(_require_finite_float(x, name=nm) for x in v)

        def leaf(name: str) -> tuple[float, ...]:
            if name not in araw:
                raise PhysicsSpecError(f"Missing {akey}.{name}", key=f"{akey}.{name}")
            val, _ = _parse_scalar_leaf(araw[name], key=f"{akey}.{name}", parser_fn=vec, **gate)
            return val

        effort_case = str(self.scenario.get("arm_effort", "nominal"))
        if effort_case not in ARM_EFFORT_CASES:
            raise PhysicsSpecError(
                f"Scenario arm_effort={effort_case!r}; expected one of {list(ARM_EFFORT_CASES)}", key="scenario.arm_effort"
            )
        if effort_case == "nominal":
            effort = leaf("effort_nm")
        else:
            # Validated and gated, but not counted: RoboDojo's actuator effort stays.
            _parse_scalar_leaf(
                araw.get("effort_nm"), key=f"{akey}.effort_nm", parser_fn=vec,
                allowed_statuses=self.allowed_statuses, allow_unsourced_keys=self.allow_unsourced_keys,
                allow_scenario_keys=self.allow_scenario_keys,
            )
            effort = None
        return ArmLimitsSpec(
            robot_name=rname,
            joints=tuple(str(j) for j in joints),
            lower_rad=leaf("lower_rad"),
            upper_rad=leaf("upper_rad"),
            velocity_rad_s=leaf("velocity_rad_s"),
            effort_nm=effort,
            effort_case=effort_case,
            provenance=f"{self.source_path}:{akey}",
        )

    def _parse_gripper(self, rname: str, graw: Any) -> GripperSpec:
        """Parse ``robots.<rname>.gripper`` (D3, #90); scenario keys ``gripper_effort`` (nominal/low/high), ``gripper_armature``
        (nominal/none) and ``gripper_drive_limit`` (trip/motor_peak, #160)."""
        gkey = f"robots.{rname}.gripper"
        if not isinstance(graw, Mapping):
            raise PhysicsSpecError(f"{gkey} must be a mapping", key=gkey)
        unknown = sorted(set(graw.keys()) - _ALLOWED_GRIPPER_KEYS)
        if unknown:
            raise PhysicsSpecError(f"Unknown key(s) {unknown} in {gkey}", key=f"{gkey}.{unknown[0]}")
        gate = {
            "allowed_statuses": self.allowed_statuses,
            "allow_unsourced_keys": self.allow_unsourced_keys,
            "allow_scenario_keys": self.allow_scenario_keys,
            "status_counts": self.status_counts,
            "unsourced_keys_used": self.unsourced_keys_used,
            "scenario_keys_used": self.scenario_keys_used,
        }

        def band(name: str) -> float:
            if name not in graw:
                raise PhysicsSpecError(f"Missing {gkey}.{name}", key=f"{gkey}.{name}")
            val, _, _ = _validate_band_entry(graw[name], key=f"{gkey}.{name}", positive=True, **gate)
            return val

        effort_case = str(self.scenario.get("gripper_effort", "nominal"))
        if effort_case not in GRIPPER_EFFORT_CASES:
            raise PhysicsSpecError(
                f"Scenario gripper_effort={effort_case!r}; expected one of {list(GRIPPER_EFFORT_CASES)}",
                key="scenario.gripper_effort",
            )
        eff_raw = graw.get("finger_effort_limit_n")
        if not isinstance(eff_raw, Mapping) or set(eff_raw.keys()) != set(GRIPPER_EFFORT_CASES):
            raise PhysicsSpecError(
                f"{gkey}.finger_effort_limit_n must map exactly {list(GRIPPER_EFFORT_CASES)} to entries",
                key=f"{gkey}.finger_effort_limit_n",
            )
        effort = None
        for case in GRIPPER_EFFORT_CASES:
            ckey = f"{gkey}.finger_effort_limit_n.{case}"
            if case == effort_case:
                effort, _, _ = _validate_band_entry(eff_raw[case], key=ckey, positive=True, **gate)
            else:
                # Validated and gated, but not counted: only the selected case reaches the engine.
                _validate_band_entry(
                    eff_raw[case], key=ckey, positive=True,
                    allowed_statuses=self.allowed_statuses,
                    allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                )

        mraw = graw.get("mimic")
        if not isinstance(mraw, Mapping):
            raise PhysicsSpecError(f"{gkey}.mimic must be a mapping", key=f"{gkey}.mimic")
        unknown_m = sorted(set(mraw.keys()) - _ALLOWED_GRIPPER_MIMIC_KEYS)
        if unknown_m:
            raise PhysicsSpecError(f"Unknown key(s) {unknown_m} in {gkey}.mimic", key=f"{gkey}.mimic.{unknown_m[0]}")
        gearing, _ = _parse_scalar_leaf(
            mraw.get("gearing"), key=f"{gkey}.mimic.gearing",
            parser_fn=lambda v, nm: _require_finite_float(v, name=nm), **gate,
        )
        offset, _ = _parse_scalar_leaf(
            mraw.get("offset"), key=f"{gkey}.mimic.offset",
            parser_fn=lambda v, nm: _require_finite_float(v, name=nm), **gate,
        )
        armature_case = str(self.scenario.get("gripper_armature", "nominal"))
        if armature_case not in GRIPPER_ARMATURE_CASES:
            raise PhysicsSpecError(
                f"Scenario gripper_armature={armature_case!r}; expected one of {list(GRIPPER_ARMATURE_CASES)}",
                key="scenario.gripper_armature",
            )
        if armature_case == "nominal":
            finger_armature = band("finger_armature_kg")
        else:
            # Validated and gated, but not counted: no armature reaches the engine.
            _validate_band_entry(
                graw.get("finger_armature_kg"), key=f"{gkey}.finger_armature_kg", positive=True,
                allowed_statuses=self.allowed_statuses, allow_unsourced_keys=self.allow_unsourced_keys,
                allow_scenario_keys=self.allow_scenario_keys,
            )
            finger_armature = 0.0
        # #160: both drive limits are required; the selected one reaches the engine and is counted.
        drive_case = str(self.scenario.get("gripper_drive_limit", "trip"))
        if drive_case not in GRIPPER_DRIVE_LIMIT_CASES:
            raise PhysicsSpecError(
                f"Scenario gripper_drive_limit={drive_case!r}; expected one of {list(GRIPPER_DRIVE_LIMIT_CASES)}",
                key="scenario.gripper_drive_limit",
            )
        limits = {}
        for case, name in (("trip", "finger_trip_n"), ("motor_peak", "finger_motor_peak_n")):
            if case == drive_case:
                limits[name] = band(name)
            else:
                if name not in graw:
                    raise PhysicsSpecError(f"Missing {gkey}.{name}", key=f"{gkey}.{name}")
                limits[name], _, _ = _validate_band_entry(
                    graw[name], key=f"{gkey}.{name}", positive=True,
                    allowed_statuses=self.allowed_statuses, allow_unsourced_keys=self.allow_unsourced_keys,
                    allow_scenario_keys=self.allow_scenario_keys,
                )
        return GripperSpec(
            finger_armature_kg=finger_armature,
            armature_case=armature_case,
            robot_name=rname,
            driven_joint=graw.get("driven_joint"),
            mimic_joint=graw.get("mimic_joint"),
            finger_stiffness_n_m=band("finger_stiffness_n_per_m"),
            finger_damping_n_s_m=band("finger_damping_n_s_per_m"),
            finger_effort_n=effort,
            effort_case=effort_case,
            drive_limit_case=drive_case,
            **limits,
            slew_m_s=band("slew_m_per_s"),
            mimic_gearing=gearing,
            mimic_offset=offset,
            mimic_axis=str(mraw.get("axis", "rotX")),
            provenance=f"{self.source_path}:{gkey}",
        )


# --- Resolver & dormant-value diffing ------------------------------------------------------------------------------


class PhysicsResolver:
    """Turns ``(params_table, layout_declaration, preset/fixes)`` into frozen per-object specs."""

    def __init__(
        self,
        *,
        preset: str = "shipped",
        fixes: Iterable[str] | str | None = None,
        params_table: PhysicsParamsTable | None = None,
        task_name: str | None = None,
        device: str = "cpu",
        require_reviewed_diff: bool = False,
    ) -> None:
        self.preset, self.fixes = resolve_fixes(preset, fixes)
        self.params_table = params_table
        self.task_name = task_name
        self.device = str(device or "cpu")
        self.require_reviewed_diff = require_reviewed_diff
        if self.table_values and self.params_table is None:
            raise UnsourcedParameterError(
                "physics fix 'values' (or a part of it) is enabled, but no parameter table was provided (PR 1 quarantines dormant layout values)",
                task=task_name,
                key="physics_params_path",
            )
        if "gripper" in self.fixes and self.params_table is None:
            raise UnsourcedParameterError(
                "physics fix 'gripper' is enabled, but no parameter table was provided (robots.<name>.gripper holds its values)",
                task=task_name,
                key="physics_params_path",
            )
        if (
            self.table_values
            and self.require_reviewed_diff
            and self.task_name
            and self.params_table is not None
            and self.task_name not in self.params_table.reviewed_tasks
        ):
            raise UnsourcedParameterError(
                f"Task {self.task_name!r} has no reviewed diff hash in parameter table",
                task=self.task_name,
                key="reviewed_tasks",
            )
        self.declarations_checked: int = 0
        self.unaudited_out_of_scope: list[dict[str, Any]] = []
        self.code_defaults_applied: list[str] = []
        self.diff_records: list[dict[str, Any]] = []

    @property
    def is_shipped(self) -> bool:
        return len(self.fixes) == 0

    @property
    def materials_values(self) -> bool:
        """Friction, restitution and combine modes come from the parameter table."""
        return bool(self.fixes & {"values", "materials"})

    @property
    def masses_values(self) -> bool:
        """Rigid masses, inertias and centres of mass come from the parameter table."""
        return bool(self.fixes & {"values", "masses"})

    @property
    def table_values(self) -> bool:
        return self.materials_values or self.masses_values

    @property
    def split_values(self) -> bool:
        """Exactly one of the two value parts is active (a G4 attribution arm)."""
        return self.materials_values != self.masses_values

    def _refuse_split(self, asset_id: str, prim_path: str) -> None:
        if self.split_values:
            raise PhysicsSpecError(
                f"Value parts cannot be split for {asset_id!r} (articulation and dynamic rows mix masses, materials and drives); use 'values'",
                task=self.task_name,
                prim_path=prim_path,
                key="physics_fixes",
            )

    @property
    def dormant_values_active(self) -> bool:
        return ("order" in self.fixes) and not self.materials_values

    @property
    def values_undelivered_on_cpu(self) -> bool:
        return self.materials_values and ("order" not in self.fixes) and self.device.startswith("cpu")

    @property
    def mass_delivery(self) -> str:
        if not self.masses_values:
            return "shipped_set_mass_after_parse"
        # Delivery is a function of the union, so ``values`` and ``materials,masses`` deliver identically; ``masses``
        # alone (no materials) authors before the parse in every combination (#113).
        if "order" in self.fixes or not self.materials_values:
            return "usd_mass_api_before_parse"
        return "table_set_mass_after_parse"

    @staticmethod
    def _get_physics_subkey(config: Any, section: str, key: str) -> tuple[bool, Any]:
        """Read a single leaf ``config.<section>.<key>`` without resolving unrelated OmegaConf subtrees."""
        if config is None:
            return False, None
        sec = config.get(section) if isinstance(config, Mapping) else getattr(config, section, None)
        if sec is None or not isinstance(sec, Mapping):
            return False, None
        if key not in sec:
            return False, None
        return True, _to_plain(sec[key])

    @classmethod
    def from_config(cls, config: Any, *, task_name: str | None = None) -> PhysicsResolver:
        def _first_present(keys_sections: Sequence[tuple[str, str]]) -> tuple[bool, Any]:
            for sec, k in keys_sections:
                found, val = cls._get_physics_subkey(config, sec, k)
                if found and val is not None:
                    return True, val
            return False, None

        _, cfg_tname = cls._get_physics_subkey(config, "eval_cfg", "task_name")
        tname = task_name or cfg_tname
        _, cfg_dev = cls._get_physics_subkey(config, "sim", "device")
        device = str(cfg_dev or "cpu")

        _, cfg_preset = _first_present(
            (("eval_cfg", "physics_preset"), ("sim", "physics_preset"), ("task_env", "physics_preset"))
        )
        preset = cfg_preset or os.environ.get("ROBODOJO_PHYSICS_PRESET") or "shipped"

        has_fixes, cfg_fixes = _first_present(
            (("eval_cfg", "physics_fixes"), ("sim", "physics_fixes"), ("task_env", "physics_fixes"))
        )
        fixes = cfg_fixes if has_fixes else os.environ.get("ROBODOJO_PHYSICS_FIXES")

        _, cfg_params = _first_present(
            (("eval_cfg", "physics_params_path"), ("sim", "physics_params_path"), ("task_env", "physics_params_path"))
        )
        _, cfg_statuses = _first_present((("eval_cfg", "physics_allowed_statuses"), ("sim", "physics_allowed_statuses")))
        allowed_statuses = cfg_statuses or ("sourced", "estimated")
        _, cfg_unsourced_keys = _first_present(
            (("eval_cfg", "physics_allow_unsourced"), ("sim", "physics_allow_unsourced"))
        )
        env_unsourced = os.environ.get("ROBODOJO_PHYSICS_ALLOW_UNSOURCED")
        allow_unsourced_keys = (
            cfg_unsourced_keys
            if cfg_unsourced_keys is not None
            else ([k.strip() for k in env_unsourced.split(",") if k.strip()] if env_unsourced else ())
        )
        _, cfg_scenario_keys = _first_present(
            (("eval_cfg", "physics_allow_scenario"), ("sim", "physics_allow_scenario"))
        )
        allow_scenario_keys = cfg_scenario_keys or ()
        _, cfg_opt_in = _first_present(
            (("eval_cfg", "physics_scenario_opt_in"), ("sim", "physics_scenario_opt_in"), ("task_env", "physics_scenario_opt_in"))
        )
        scenario_opt_in = cfg_opt_in or os.environ.get("ROBODOJO_PHYSICS_SCENARIO_OPT_IN")
        _, scenario = _first_present((("eval_cfg", "physics_scenario"), ("sim", "physics_scenario")))
        _, req_rev = _first_present(
            (("eval_cfg", "physics_require_reviewed_diff"), ("sim", "physics_require_reviewed_diff"))
        )
        require_reviewed = bool(req_rev) if req_rev is not None else False

        _, resolved_fixes = resolve_fixes(preset, fixes)
        # Under shipped (no active fixes), do not let ROBODOJO_PHYSICS_PARAMS trigger a table load.
        params_path = cfg_params or (os.environ.get("ROBODOJO_PHYSICS_PARAMS") if resolved_fixes else None)
        table = None
        if params_path and resolved_fixes:
            table = PhysicsParamsTable.from_path(
                str(params_path),
                allowed_statuses=allowed_statuses,
                allow_unsourced_keys=allow_unsourced_keys,
                allow_scenario_keys=allow_scenario_keys,
                scenario=scenario,
                scenario_opt_in=scenario_opt_in,
            )
        elif resolved_fixes & ({"values", "gripper"} | VALUE_PARTS):
            fix_name = next(f for f in ("values", "materials", "masses", "gripper") if f in resolved_fixes)
            raise UnsourcedParameterError(
                f"physics fix {fix_name!r} requires physics_params_path (dormant layout values are never applied directly)",
                task=tname,
                key="physics_params_path",
            )
        return cls(
            preset=preset,
            fixes=resolved_fixes,
            params_table=table,
            task_name=tname,
            device=device,
            require_reviewed_diff=require_reviewed,
        )

    def reset_episode_records(self) -> None:
        self.declarations_checked = 0
        self.unaudited_out_of_scope.clear()
        self.code_defaults_applied.clear()
        self.diff_records.clear()

    def validate_saved_layout(self, env_config: Mapping[str, Any] | None, *, env_id: int, layout_id: int | None = None) -> None:
        if env_config is None:
            return
        for sec in ("Rigid", "Dynamic", "Geometry", "Articulation", "Garment", "Fluid"):
            sec_val = env_config.get(sec)
            if not isinstance(sec_val, Mapping):
                continue
            for cat, inst_list in sec_val.items():
                if not isinstance(inst_list, Sequence) or isinstance(inst_list, (str, bytes)):
                    continue
                for idx, inst in enumerate(inst_list):
                    if not isinstance(inst, Mapping):
                        continue
                    p_dict = inst.get("physics") or {}
                    checked = validate_declaration(
                        p_dict,
                        section=sec,
                        task=self.task_name,
                        layout=layout_id,
                        env_id=env_id,
                        prim_path=f"{sec}/{cat}[{idx}]",
                        scale=inst.get("scale"),
                    )
                    self.declarations_checked += 1
                    if checked["type"] in ("garment", "fluid"):
                        self.unaudited_out_of_scope.append(
                            {"env_id": env_id, "section": sec, "category": cat, "type": checked["type"]}
                        )
        for fk in ("Table", "Ground", "Room"):
            if fk in env_config and env_config[fk] is not None:
                validate_fixture_declaration(fk, env_config[fk], task=self.task_name, layout=layout_id, env_id=env_id)
                self.declarations_checked += 1

    @staticmethod
    def canonical_asset_id(section: str, category: str, category_idx: int | None, is_cluttered: bool = False) -> str:
        prefix = "Clutter" if is_cluttered else section.capitalize()
        if category_idx is None:
            return f"{prefix}/{category}"
        return f"{prefix}/{category}/{int(category_idx):05d}"

    def _lookup_asset(self, asset_id: str, *, prim_path: str | None = None) -> dict[str, Any]:
        assert self.params_table is not None
        if asset_id not in self.params_table.assets:
            raise UnsourcedParameterError(
                f"Missing asset entry {asset_id!r} in parameter table {self.params_table.source_path}",
                task=self.task_name,
                prim_path=prim_path,
                key=asset_id,
            )
        row = self.params_table.assets[asset_id]
        tasks_explicit = row.get("tasks_explicit")
        if (
            tasks_explicit is not None
            and self.task_name in self.params_table.reviewed_tasks
            and self.task_name not in tasks_explicit
        ):
            raise PhysicsSpecError(
                f"Asset {asset_id!r} spawned in reviewed task {self.task_name!r}, which is not in assets.{asset_id}.tasks={sorted(tasks_explicit)}",
                task=self.task_name,
                prim_path=prim_path,
                key=f"assets.{asset_id}.tasks",
            )
        return row

    @staticmethod
    def _dormant_flags(raw_phys: Mapping[str, Any], bbox_extents: tuple[float, float, float] | None) -> list[str]:
        flags: list[str] = []
        if "friction" in raw_phys:
            flags.append("dead_key_friction_#1")
            if float(raw_phys["friction"]) == 0.0:
                flags.append("dormant_friction_zero_#8")
        if raw_phys.get("static_friction") == 10000.0 or raw_phys.get("dynamic_friction") == 10000.0:
            flags.append("dormant_friction_10000_#3")
        if raw_phys.get("static_friction") == 2.0 or raw_phys.get("dynamic_friction") == 2.0:
            flags.append("dormant_friction_2.0_#7")
        sf = raw_phys.get("static_friction", 0.6)
        df = raw_phys.get("dynamic_friction", 1.5)
        if float(sf) < float(df):
            flags.append("static_lt_dynamic_#2_#6")
        if "mass" in raw_phys:
            m = float(raw_phys["mass"])
            if m > 0.5:
                flags.append("mass_capped_0.5kg_#4_#5")
            elif m <= 0.0:
                flags.append("mass_floored_0.05kg_#5")
        else:
            flags.append("mass_default_0.5kg_#5")
        if "size" in raw_phys:
            flags.append("dead_key_size_#46")
            if bbox_extents is not None:
                sz = [float(x) for x in raw_phys["size"]]
                if any(abs(sz[i] - bbox_extents[i]) / max(bbox_extents[i], 1e-9) > 0.05 for i in range(3)):
                    flags.append("size_mismatch_gt_5pct_#46")
        return flags

    def resolve_rigid(
        self,
        *,
        prim_path: str,
        asset_id: str,
        inst_config: Mapping[str, Any],
        scale: Sequence[float],
        metadata_bbox_extents: Sequence[float] | None = None,
        env_id: int | None = None,
    ) -> RigidSpec:
        plain_inst = _to_plain(inst_config or {})
        raw_phys = validate_declaration(
            plain_inst.get("physics"),
            section="Rigid",
            task=self.task_name,
            env_id=env_id,
            prim_path=prim_path,
            scale=scale,
        )
        base_scale = _require_vec3_pos(scale, name="scale", prim_path=prim_path)
        meta_ext = _require_vec3_pos(metadata_bbox_extents, name="aligned_bbox") if metadata_bbox_extents is not None else None

        # Shipped values (for shipped preset, partial ablations, and diff recording)
        raw_mass = float(raw_phys.get("mass", 0.5))
        shipped_mass = min(raw_mass, 0.5)
        if shipped_mass <= 0.0:
            shipped_mass = 0.05
        shipped_sf = float(raw_phys.get("static_friction", 0.6))
        shipped_df = float(raw_phys.get("dynamic_friction", 1.5))
        shipped_rest = float(raw_phys.get("restitution", 0.0))

        row = self._lookup_asset(asset_id, prim_path=prim_path) if self.table_values else None
        if self.masses_values:
            tv = (row.get("task_variants") or {}).get(self.task_name or "", {})
            mass_kg = tv.get("mass_kg", row.get("mass_kg"))
            if mass_kg is None:
                raise UnsourcedParameterError(f"Asset {asset_id!r} has no mass_kg", task=self.task_name, prim_path=prim_path, key="mass_kg")
            iom = row.get("inertia_over_mass_m2")
            inertia_kg_m2 = (mass_kg * iom[0], mass_kg * iom[1], mass_kg * iom[2]) if iom is not None else None
            com_offset_m = row.get("com_offset_m")
            solid_iom = row.get("solid_collider_inertia_over_mass_m2")
        else:
            mass_kg = shipped_mass
            inertia_kg_m2 = None
            com_offset_m = None
            solid_iom = None
        if self.materials_values:
            mat = row.get("material")
            if mat is None:
                raise UnsourcedParameterError(f"Asset {asset_id!r} has no resolved material", task=self.task_name, prim_path=prim_path, key="material")
        else:
            mat = MaterialSpec(
                static_friction=shipped_sf,
                dynamic_friction=shipped_df,
                restitution=shipped_rest,
                friction_combine_mode=None,
                restitution_combine_mode=None,
                allow_legacy_inversion=True,
            )
        if row is not None:
            asset_ext = row.get("collider_extents_m") or row.get("size_m") or meta_ext
            s_decision = row.get("size_decision", "asset")
            col_pre = row.get("collision_spec")
            prov = f"{self.params_table.source_path}:{asset_id}"
        else:
            asset_ext = meta_ext
            s_decision = "asset"
            col_pre = None
            prov = "shipped" if "order" not in self.fixes else "declared_order_only"

        if "size" in self.fixes:
            if "size" in raw_phys and asset_ext is not None:
                decl_sz = _require_vec3_pos(raw_phys["size"], name="size", prim_path=prim_path)
                mismatch = any(abs(decl_sz[i] - asset_ext[i]) / max(asset_ext[i], 1e-9) > 0.05 for i in range(3))
                if mismatch and row is None:
                    raise PhysicsSpecError(
                        f"Declared size {decl_sz} differs >5% from asset bbox {asset_ext} without size_decision (#46)",
                        task=self.task_name,
                        prim_path=prim_path,
                        key="size",
                    )
            if row is not None and s_decision == "scale" and row.get("size_m") is not None and meta_ext is not None:
                tgt = row["size_m"]
                res_scale = (tgt[0] / meta_ext[0], tgt[1] / meta_ext[1], tgt[2] / meta_ext[2])
                col_base = row.get("collider_extents_m") or meta_ext
                exp_ext = (col_base[0] * res_scale[0], col_base[1] * res_scale[1], col_base[2] * res_scale[2])
            else:
                res_scale = base_scale
                exp_ext = (
                    (asset_ext[0] * res_scale[0], asset_ext[1] * res_scale[1], asset_ext[2] * res_scale[2])
                    if asset_ext is not None
                    else None
                )
        else:
            res_scale = base_scale
            exp_ext = (
                (asset_ext[0] * res_scale[0], asset_ext[1] * res_scale[1], asset_ext[2] * res_scale[2])
                if asset_ext is not None
                else None
            )

        col_spec = (
            (col_pre if col_pre is not None else CollisionSpec(enabled=None, approximation="asset"))
            if "collision" in self.fixes
            else CollisionSpec(enabled=None, approximation="asset")
        )

        spec = RigidSpec(
            asset_id=asset_id,
            mass_kg=mass_kg,
            scale=res_scale,
            expected_extents_m=exp_ext,
            material=mat,
            collision=col_spec,
            com_offset_m=com_offset_m,
            inertia_kg_m2=inertia_kg_m2,
            solid_collider_inertia_over_mass_m2=solid_iom,
            provenance=prov,
            author_pre_parse=("order" in self.fixes),
            author_usd_mass=(self.mass_delivery == "usd_mass_api_before_parse"),
            author_collision=("collision" in self.fixes),
        )
        self.diff_records.append(
            {
                "prim_path": prim_path,
                "asset_id": asset_id,
                "class": "rigid",
                "declared": raw_phys,
                "shipped_effective": {
                    "mass_kg": shipped_mass,
                    "engine_material_cpu": [0.5, 0.5, 0.0],
                    "authored_material": [shipped_sf, shipped_df, shipped_rest],
                    "effective_static_if_delivered": max(shipped_sf, shipped_df),
                },
                "resolved": {
                    "mass_kg": spec.mass_kg,
                    "com_offset_m": list(spec.com_offset_m) if spec.com_offset_m is not None else None,
                    "inertia_kg_m2": list(spec.inertia_kg_m2) if spec.inertia_kg_m2 is not None else None,
                    "material": [spec.material.static_friction, spec.material.dynamic_friction, spec.material.restitution],
                    "combine_modes": [spec.material.friction_combine_mode, spec.material.restitution_combine_mode],
                    "scale": list(spec.scale),
                    "author_pre_parse": spec.author_pre_parse,
                },
                "dormant_flags": self._dormant_flags(raw_phys, asset_ext),
            }
        )
        return spec

    def resolve_geometry(
        self,
        *,
        prim_path: str,
        asset_id: str,
        inst_config: Mapping[str, Any],
        scale: Sequence[float],
        metadata_bbox_extents: Sequence[float] | None = None,
        env_id: int | None = None,
    ) -> StaticSpec:
        plain_inst = _to_plain(inst_config or {})
        raw_phys = validate_declaration(
            plain_inst.get("physics"),
            section="Geometry",
            task=self.task_name,
            env_id=env_id,
            prim_path=prim_path,
            scale=scale,
        )
        base_scale = _require_vec3_pos(scale, name="scale", prim_path=prim_path)
        meta_ext = _require_vec3_pos(metadata_bbox_extents, name="aligned_bbox") if metadata_bbox_extents is not None else None

        if self.table_values:
            row = self._lookup_asset(asset_id, prim_path=prim_path)
            asset_ext = row.get("collider_extents_m") or row.get("size_m") or meta_ext
            col_pre = row.get("collision_spec")
            prov = f"{self.params_table.source_path}:{asset_id}"
        else:
            asset_ext = meta_ext
            col_pre = None
            prov = "shipped"
        if self.materials_values:
            mat = row.get("material")
            if mat is None:
                raise UnsourcedParameterError(f"Asset {asset_id!r} has no resolved material", task=self.task_name, prim_path=prim_path, key="material")
            bind_mat = "order" in self.fixes
        else:
            mat = MaterialSpec(
                static_friction=float(raw_phys.get("static_friction", 0.0)),
                dynamic_friction=float(raw_phys.get("dynamic_friction", 0.0)),
                restitution=float(raw_phys.get("restitution", 0.5)),
                friction_combine_mode=None,
                restitution_combine_mode=None,
                allow_legacy_inversion=True,
            )
            bind_mat = False  # Upstream geometry.py:85-90 builds 0/0/0.5 and never binds (#23)

        exp_ext = (
            (asset_ext[0] * base_scale[0], asset_ext[1] * base_scale[1], asset_ext[2] * base_scale[2])
            if asset_ext is not None
            else None
        )
        col_spec = (
            (col_pre if col_pre is not None else CollisionSpec(enabled=None, approximation="asset"))
            if "collision" in self.fixes
            else CollisionSpec(enabled=bool(raw_phys.get("collision", False)), approximation="asset")
        )
        spec = StaticSpec(
            asset_id=asset_id,
            scale=base_scale,
            expected_extents_m=exp_ext,
            material=mat,
            collision=col_spec,
            bind_material=bind_mat,
            author_collision=("collision" in self.fixes),
            provenance=prov,
        )
        self.diff_records.append(
            {
                "prim_path": prim_path,
                "asset_id": asset_id,
                "class": "geometry",
                "declared": raw_phys,
                "resolved": {
                    "bind_material": spec.bind_material,
                    "material": [spec.material.static_friction, spec.material.dynamic_friction, spec.material.restitution],
                },
                "dormant_flags": [k for k in ("mass", "friction", "size") if k in raw_phys],
            }
        )
        return spec

    def resolve_room(self, *, prim_path: str, room_cfg: Mapping[str, Any] | None, env_id: int | None = None) -> StaticSpec:
        raw = validate_fixture_declaration("Room", room_cfg, task=self.task_name, env_id=env_id)
        scale = _require_vec3_pos(raw.get("scale", (1.0, 1.0, 1.0)), name="Room.scale", prim_path=prim_path)
        asset_id = "Scene/Room"
        col_pre = None
        if self.materials_values:  # fixtures carry only materials; they follow the `materials` part
            assert self.params_table is not None
            if asset_id in self.params_table.assets:
                row = self.params_table.assets[asset_id]
                mat = row["material"]
                col_pre = row.get("collision_spec")
                bind_mat = "order" in self.fixes
                prov = f"{self.params_table.source_path}:{asset_id}"
            elif "ground" in self.params_table.class_materials:
                mat = self.params_table.class_materials["ground"]
                bind_mat = "order" in self.fixes
                prov = f"{self.params_table.source_path}:classes.ground"
            else:
                raise UnsourcedParameterError(
                    "Missing Scene/Room or class 'ground' in parameter table",
                    task=self.task_name,
                    prim_path=prim_path,
                    key=asset_id,
                )
        else:
            mat = MaterialSpec(0.5, 0.5, 0.0, friction_combine_mode="average", restitution_combine_mode="average")
            bind_mat = False
            prov = "shipped"
        col_spec = (
            (col_pre if col_pre is not None else CollisionSpec(enabled=None, approximation="asset"))
            if "collision" in self.fixes
            else CollisionSpec(enabled=bool(raw.get("collision", True)), approximation="asset")
        )
        return StaticSpec(
            asset_id=asset_id,
            scale=scale,
            expected_extents_m=None,
            material=mat,
            collision=col_spec,
            bind_material=bind_mat,
            author_collision=("collision" in self.fixes),
            provenance=prov,
        )

    def resolve_table(self, *, prim_path: str, table_cfg: Mapping[str, Any] | None, env_id: int | None = None) -> TableSpec:
        raw = validate_fixture_declaration("Table", table_cfg, task=self.task_name, env_id=env_id)
        scale = _require_vec3_pos(raw.get("scale", (1.0, 1.0, 1.0)), name="Table.scale", prim_path=prim_path)
        is_static = bool(raw.get("static", True))
        asset_id = "Scene/Table"
        if self.split_values and not is_static:
            raise PhysicsSpecError(
                "A non-static table's mass and material cannot be split; use 'values' (G4 split covers static tables only)",
                task=self.task_name,
                prim_path=prim_path,
                key="physics_fixes",
            )

        if self.materials_values:  # a static table carries only a material; it follows the `materials` part
            assert self.params_table is not None
            if asset_id in self.params_table.assets:
                row = self.params_table.assets[asset_id]
                mat = row["material"]
                mass_kg = None if is_static else row.get("mass_kg", raw.get("mass"))
                col_pre = row.get("collision_spec")
                prov = f"{self.params_table.source_path}:{asset_id}"
            else:
                tcls = str(self.params_table.scenario.get("table", "table_laminate"))
                if tcls not in self.params_table.class_materials:
                    raise UnsourcedParameterError(
                        f"Missing Scene/Table or class {tcls!r} in parameter table",
                        task=self.task_name,
                        prim_path=prim_path,
                        key=asset_id,
                    )
                mat = self.params_table.class_materials[tcls]
                mass_kg = None if is_static else raw.get("mass")
                col_pre = None
                prov = f"{self.params_table.source_path}:classes.{tcls}"
            bind_pre = "order" in self.fixes
        else:
            mat = MaterialSpec(
                static_friction=0.8,
                dynamic_friction=0.8,
                restitution=0.0,
                friction_combine_mode=None,
                restitution_combine_mode=None,
                allow_legacy_inversion=True,
            )
            mass_kg = None if is_static else float(raw.get("mass", 10.0))
            col_pre = None
            # Upstream table.py:200 never binds when is_static=True (#14); only non-static flushes at :137 (#58)
            bind_pre = ("order" in self.fixes) and (not is_static)
            prov = "shipped"

        if "collision" in self.fixes and col_pre is None:
            note = "Table.collision: code default approximation='none' (triangle mesh) applied in place of shipped 'convexDecomposition'"
            if note not in self.code_defaults_applied:
                self.code_defaults_applied.append(note)
        col_spec = (
            (col_pre if col_pre is not None else CollisionSpec(enabled=True, approximation="none"))
            if "collision" in self.fixes
            else CollisionSpec(enabled=True, approximation="convexDecomposition")
        )
        # CreateMeshPrimCommand Cube has local extents 1.0 x 1.0 x 1.0 (size = scale)
        return TableSpec(
            asset_id=asset_id,
            is_static=is_static,
            scale=scale,
            expected_extents_m=scale,
            material=mat,
            collision=col_spec,
            mass_kg=mass_kg,
            bind_pre_parse=bind_pre,
            author_collision=("collision" in self.fixes),
            provenance=prov,
        )

    def resolve_ground(
        self,
        *,
        prim_path: str,
        ground_cfg: Mapping[str, Any] | None,
        env_spacing: float,
        env_id: int | None = None,
    ) -> StaticSpec:
        raw = validate_fixture_declaration("Ground", ground_cfg, task=self.task_name, env_id=env_id)
        thickness = float(raw.get("thickness", 0.1))
        asset_id = "Scene/Ground"

        col_pre = None
        if self.materials_values:  # fixtures carry only materials; they follow the `materials` part
            assert self.params_table is not None
            if asset_id in self.params_table.assets:
                row = self.params_table.assets[asset_id]
                mat = row["material"]
                col_pre = row.get("collision_spec")
                prov = f"{self.params_table.source_path}:{asset_id}"
            elif "ground" in self.params_table.class_materials:
                mat = self.params_table.class_materials["ground"]
                prov = f"{self.params_table.source_path}:classes.ground"
            else:
                raise UnsourcedParameterError(
                    "Missing Scene/Ground or class 'ground' in parameter table",
                    task=self.task_name,
                    prim_path=prim_path,
                    key=asset_id,
                )
        else:
            pm = raw.get("physics_material") or {}
            mat = MaterialSpec(
                static_friction=float(pm.get("static_friction", 0.5)),
                dynamic_friction=float(pm.get("dynamic_friction", 0.5)),
                restitution=float(pm.get("restitution", 0.0)),
                friction_combine_mode=pm.get("friction_combine_mode", "average"),
                restitution_combine_mode=pm.get("restitution_combine_mode", "average"),
                compliant_contact_stiffness=pm.get("compliant_contact_stiffness", 0.0),
                compliant_contact_damping=pm.get("compliant_contact_damping", 0.0),
            )
            prov = "shipped"

        col_spec = (
            (col_pre if col_pre is not None else CollisionSpec(enabled=True, approximation="none"))
            if "collision" in self.fixes
            else CollisionSpec(enabled=True, approximation="none")
        )
        return StaticSpec(
            asset_id=asset_id,
            scale=(1.0, 1.0, 1.0),
            expected_extents_m=(float(env_spacing), float(env_spacing), thickness),
            material=mat,
            collision=col_spec,
            bind_material=("order" in self.fixes),
            author_collision=("collision" in self.fixes),
            provenance=prov,
        )

    def resolve_articulation(
        self,
        *,
        prim_path: str,
        asset_id: str,
        inst_config: Mapping[str, Any],
        scale: Sequence[float],
        env_id: int | None = None,
    ) -> ArticulationSpec:
        plain_inst = _to_plain(inst_config or {})
        raw_phys = validate_declaration(
            plain_inst.get("physics"),
            section="Articulation",
            task=self.task_name,
            env_id=env_id,
            prim_path=prim_path,
            scale=scale,
        )
        base_scale = _require_vec3_pos(scale, name="scale", prim_path=prim_path)
        self._refuse_split(asset_id, prim_path)
        if self.table_values:
            row = self._lookup_asset(asset_id, prim_path=prim_path)
            if row.get("fixed_base") is None:
                raise PhysicsSpecError(
                    f"Articulation {asset_id!r} requires explicit fixed_base in parameter table (#49)",
                    task=self.task_name,
                    prim_path=prim_path,
                    key="fixed_base",
                )
            col_pre = row.get("collision_spec")
            return ArticulationSpec(
                asset_id=asset_id,
                fixed_base=bool(row["fixed_base"]),
                reset_drive=bool(row["reset_drive"]) if row.get("reset_drive") is not None else bool(raw_phys.get("reset_drive", False)),
                scale=base_scale,
                link_masses_kg=dict(row.get("link_masses_kg") or {}),
                link_materials=dict(row.get("link_materials") or {}),
                joint_drives=dict(row.get("joint_drives") or {}),
                default_material=row.get("material"),
                collision=col_pre if col_pre is not None else CollisionSpec(enabled=None, approximation="asset"),
                author_pre_parse=("order" in self.fixes),
                author_collision=("collision" in self.fixes),
                provenance=f"{self.params_table.source_path}:{asset_id}",
            )
        return ArticulationSpec(
            asset_id=asset_id,
            fixed_base=bool(raw_phys.get("fixed_base", False)),
            reset_drive=bool(raw_phys.get("reset_drive", False)),
            scale=base_scale,
            author_pre_parse=False,
            author_collision=False,
            provenance="shipped",
        )

    def resolve_dynamic(
        self,
        *,
        prim_path: str,
        asset_id: str,
        inst_config: Mapping[str, Any],
        scale: Sequence[float],
        metadata_bbox_extents: Sequence[float] | None = None,
        env_id: int | None = None,
    ) -> DynamicSpec:
        plain_inst = _to_plain(inst_config or {})
        validate_declaration(
            plain_inst.get("physics"),
            section="Dynamic",
            task=self.task_name,
            env_id=env_id,
            prim_path=prim_path,
            scale=scale,
        )
        base_scale = _require_vec3_pos(scale, name="scale", prim_path=prim_path)
        meta_ext = _require_vec3_pos(metadata_bbox_extents, name="aligned_bbox") if metadata_bbox_extents is not None else None
        self._refuse_split(asset_id, prim_path)
        if self.table_values:
            row = self._lookup_asset(asset_id, prim_path=prim_path)
            col_pre = row.get("collision_spec")
            return DynamicSpec(
                asset_id=asset_id,
                scale=base_scale,
                expected_extents_m=row.get("collider_extents_m") or row.get("size_m") or meta_ext,
                belt_velocity_m_s=row.get("belt_velocity_m_s"),
                material=row.get("material"),
                collision=col_pre if col_pre is not None else CollisionSpec(enabled=None, approximation="asset"),
                author_pre_parse=("order" in self.fixes),
                author_collision=("collision" in self.fixes),
                provenance=f"{self.params_table.source_path}:{asset_id}",
            )
        return DynamicSpec(
            asset_id=asset_id,
            scale=base_scale,
            expected_extents_m=meta_ext,
            belt_velocity_m_s=None,
            material=None,
            author_pre_parse=False,
            author_collision=False,
            provenance="shipped",
        )

    def resolve_gripper(self, robot_name: str) -> GripperSpec | None:
        """D3 gripper spec for ``robot_name`` under the ``gripper`` fix, else ``None`` (RoboDojo's clamp path)."""
        if "gripper" not in self.fixes:
            return None
        rspec = self.params_table.robots.get(robot_name) if self.params_table is not None else None
        if rspec is None or rspec.gripper is None:
            raise UnsourcedParameterError(
                f"physics fix 'gripper' is enabled, but the parameter table has no robots.{robot_name}.gripper entry",
                task=self.task_name,
                key=f"robots.{robot_name}.gripper",
            )
        return rspec.gripper

    def resolve_scene_physx(self) -> dict[str, Any]:
        """PhysX scene settings under the ``solver`` fix, else ``{}`` (RoboDojo's). The caller sets them on the simulation
        config before launch, so they are authored on the physics scene before any body is parsed."""
        if "solver" not in self.fixes:
            return {}
        return {
            "enable_stabilization": False,
            "min_position_iteration_count": SOLVER_POSITION_ITERATION_FACTOR * SOLVER_SHIPPED_POSITION_ITERATIONS,
        }

    def resolve_arm_limits(self, robot_name: str) -> ArmLimitsSpec | None:
        """#11 arm limits for ``robot_name`` under the ``robot_limits`` fix, else ``None`` (RoboDojo's ranges and caps)."""
        if "robot_limits" not in self.fixes:
            return None
        rspec = self.params_table.robots.get(robot_name) if self.params_table is not None else None
        if rspec is None or rspec.arm_limits is None:
            raise UnsourcedParameterError(
                f"physics fix 'robot_limits' is enabled, but the parameter table has no robots.{robot_name}.arm_limits entry",
                task=self.task_name,
                key=f"robots.{robot_name}.arm_limits",
            )
        return rspec.arm_limits

    def resolve_robot(self, robot_name: str) -> RobotSpec:
        spec = self._resolve_robot_base(robot_name)
        extra = {"gripper": self.resolve_gripper(robot_name), "arm_limits": self.resolve_arm_limits(robot_name)}
        extra = {k: v for k, v in extra.items() if v is not None}
        if not extra:
            return spec
        return RobotSpec(**{**{f: getattr(spec, f) for f in spec.__dataclass_fields__}, **extra})

    def _resolve_robot_base(self, robot_name: str) -> RobotSpec:
        if self.table_values and self.params_table is not None:
            if robot_name not in self.params_table.robots:
                raise UnsourcedParameterError(
                    f"Missing robot entry {robot_name!r} in parameter table",
                    task=self.task_name,
                    key=f"robots.{robot_name}",
                )
            base = self.params_table.robots[robot_name]
            return RobotSpec(
                robot_name=robot_name,
                deinstance_fingers=("order" in self.fixes) or ("collision" in self.fixes),
                sanitize_camera_inertials=("collision" in self.fixes) and base.sanitize_camera_inertials,
                pad_links=base.pad_links,
                link_materials=dict(base.link_materials) if (self.materials_values and "order" in self.fixes) else {},
                link_inertials=dict(base.link_inertials) if (self.masses_values and "order" in self.fixes) else {},
                pad_torsional_radius_m=base.pad_torsional_radius_m if "collision" in self.fixes else None,
                pad_min_torsional_radius_m=base.pad_min_torsional_radius_m if "collision" in self.fixes else None,
                provenance=base.provenance,
            )
        return RobotSpec(
            robot_name=robot_name,
            deinstance_fingers=("collision" in self.fixes),
            sanitize_camera_inertials=("collision" in self.fixes),
            provenance="declared_order_only" if not self.is_shipped else "shipped",
        )

    def manifest_summary(self) -> dict[str, Any]:
        diff_json = json.dumps(self.diff_records, sort_keys=True, separators=(",", ":"))
        diff_sha = hashlib.sha256(diff_json.encode("utf-8")).hexdigest()
        if self.materials_values and self.device.startswith("cpu"):
            values_delivered: bool | None = not self.values_undelivered_on_cpu
        else:
            values_delivered = None
        delivery_note = "table values authored, not delivered on cpu" if self.values_undelivered_on_cpu else None
        admitted = sorted(self.params_table.allowed_statuses) if self.params_table is not None else None
        unsourced_used = list(self.params_table.unsourced_keys_used) if self.params_table is not None else []
        profile_unsourced_used = list(self.params_table.profile_unsourced_keys_used) if self.params_table is not None else []
        adhoc_unsourced = sorted(self.params_table.adhoc_unsourced_keys) if self.params_table is not None else []
        adhoc_unsourced_used = list(self.params_table.adhoc_unsourced_keys_used) if self.params_table is not None else []
        scenario_used = list(self.params_table.scenario_keys_used) if self.params_table is not None else []
        below_g1 = bool(
            unsourced_used
            or scenario_used
            or (
                self.params_table is not None
                and bool(self.params_table.allowed_statuses & {"unsourced", "scenario", "test_only"})
            )
        )
        summary: dict[str, Any] = {
            "preset": self.preset,
            "fixes": sorted(self.fixes),
            "device": self.device,
            "dormant_values_active": self.dormant_values_active,
            "values_delivered": values_delivered,
            "delivery_note": delivery_note,
            "table_source": self.params_table.source_path if self.params_table is not None else None,
            "table_sha256": self.params_table.table_sha256 if self.params_table is not None else None,
            "table_provenance": (
                dict(self.params_table.provenance)
                if (self.params_table is not None and self.params_table.provenance is not None)
                else None
            ),
            "admitted_statuses": admitted,
            "scenario_opt_in": self.params_table.scenario_opt_in if self.params_table is not None else None,
            "scenario": dict(self.params_table.scenario) if self.params_table is not None else None,
            "status_counts": dict(self.params_table.status_counts) if self.params_table is not None else None,
            "derived_unary_classes": dict(self.params_table.derived_unary_classes) if self.params_table is not None else None,
            "unsourced_keys_used": unsourced_used,
            "profile_unsourced_keys_used": profile_unsourced_used,
            "adhoc_unsourced_keys": adhoc_unsourced,
            "adhoc_unsourced_keys_used": adhoc_unsourced_used,
            "scenario_keys_used": scenario_used,
            "below_g1_unsourced_opt_in": below_g1,
            "realised_vs_nominal": (
                dict(self.params_table.realised_vs_nominal)
                if (self.params_table is not None and self.params_table.realised_vs_nominal)
                else None
            ),
            "code_defaults_applied": list(self.code_defaults_applied),
            "dynamic_spawn_timing_note": (
                "order adds 'dynamic' to spawnable_object_types (episode reload spawn); real conveyor witness open"
                if "order" in self.fixes
                else None
            ),
            "declarations_checked": self.declarations_checked,
            "unaudited_out_of_scope": list(self.unaudited_out_of_scope),
            "diff_count": len(self.diff_records),
            "diff_sha256": diff_sha,
            "diffs": list(self.diff_records),
            **(
                {
                    "value_parts": sorted(p for p, on in (("materials", self.materials_values), ("masses", self.masses_values)) if on),
                    "mass_delivery": self.mass_delivery,
                }
                if self.fixes & VALUE_PARTS
                else {}
            ),
        }
        if "gripper" in self.fixes and self.params_table is not None:
            # Only under the fix, so that shipped manifests stay field-for-field equal to RoboDojo's.
            summary["gripper"] = {
                rname: rs.gripper.manifest()
                for rname, rs in sorted(self.params_table.robots.items())
                if rs.gripper is not None
            }
        if "solver" in self.fixes:
            summary["solver"] = {
                "scene_physx": self.resolve_scene_physx(),
                "position_iteration_factor": SOLVER_POSITION_ITERATION_FACTOR,
                "shipped_position_iterations": SOLVER_SHIPPED_POSITION_ITERATIONS,
            }
        if "robot_limits" in self.fixes and self.params_table is not None:
            summary["arm_limits"] = {
                rname: self.resolve_arm_limits(rname).manifest()
                for rname, rs in sorted(self.params_table.robots.items())
                if rs.arm_limits is not None
            }
        return summary


def build_eval_result_physics_block(ms: Mapping[str, Any]) -> dict[str, Any]:
    """Build the compact ``eval_result['physics']`` record from ``PhysicsResolver.manifest_summary()``."""
    phys_meta: dict[str, Any] = {
        "preset": ms["preset"],
        "fixes": list(ms["fixes"]),
        "device": ms["device"],
        "dormant_values_active": bool(ms["dormant_values_active"]),
        "values_delivered": ms["values_delivered"],
        "table_sha256": ms["table_sha256"],
        "admitted_statuses": list(ms["admitted_statuses"]) if ms.get("admitted_statuses") is not None else None,
        "scenario_opt_in": ms.get("scenario_opt_in"),
        "scenario": dict(ms["scenario"]) if isinstance(ms.get("scenario"), Mapping) else ms.get("scenario"),
        "status_counts": dict(ms["status_counts"]) if isinstance(ms.get("status_counts"), Mapping) else ms.get("status_counts"),
        "below_g1_unsourced_opt_in": bool(ms["below_g1_unsourced_opt_in"]),
    }
    if "value_parts" in ms:
        phys_meta["value_parts"] = list(ms["value_parts"])
        phys_meta["mass_delivery"] = ms["mass_delivery"]
    if ms.get("dormant_values_active"):
        phys_meta["status_note"] = "dormant values active"
    if ms.get("delivery_note"):
        phys_meta["delivery_note"] = ms["delivery_note"]
        if "status_note" not in phys_meta:
            phys_meta["status_note"] = ms["delivery_note"]
    if ms.get("unsourced_keys_used"):
        phys_meta["unsourced_keys_used"] = list(ms["unsourced_keys_used"])
    if ms.get("profile_unsourced_keys_used"):
        phys_meta["profile_unsourced_keys_used"] = list(ms["profile_unsourced_keys_used"])
    if ms.get("adhoc_unsourced_keys"):
        phys_meta["adhoc_unsourced_keys"] = list(ms["adhoc_unsourced_keys"])
    if ms.get("adhoc_unsourced_keys_used"):
        phys_meta["adhoc_unsourced_keys_used"] = list(ms["adhoc_unsourced_keys_used"])
    if ms.get("scenario_keys_used"):
        phys_meta["scenario_keys_used"] = list(ms["scenario_keys_used"])
    if ms.get("gripper"):
        phys_meta["gripper"] = {
            rname: {
                "effort_case": g["effort_case"],
                "finger_effort_n": g["finger_effort_n"],
                "finger_stiffness_n_m": g["finger_stiffness_n_m"],
                "finger_damping_n_s_m": g["finger_damping_n_s_m"],
                "finger_armature_kg": g["finger_armature_kg"],
                "drive_limit_case": g["drive_limit_case"],
                "finger_trip_n": g["finger_trip_n"],
                "finger_motor_peak_n": g["finger_motor_peak_n"],
                "slew_m_s": g["slew_m_s"],
            }
            for rname, g in ms["gripper"].items()
        }
    if ms.get("arm_limits"):
        phys_meta["arm_limits"] = {
            rname: {k: a[k] for k in ("effort_case", "lower_rad", "upper_rad", "velocity_rad_s", "effort_nm")}
            for rname, a in ms["arm_limits"].items()
        }
    return phys_meta


# --- Active-resolver context (allows thin, zero-mandatory-arg hooks) -----------------------------------------------

_ACTIVE_RESOLVER: PhysicsResolver | None = None


def set_active_resolver(resolver: PhysicsResolver | None) -> None:
    global _ACTIVE_RESOLVER
    _ACTIVE_RESOLVER = resolver


def get_active_resolver() -> PhysicsResolver | None:
    return _ACTIVE_RESOLVER


# --- Pure-USD pre-parse authoring helpers --------------------------------------------------------------------------


def author_usd_material(stage: Any, mat_path: str, mat_spec: MaterialSpec) -> Any:
    """Author a USD ``PhysicsMaterial`` prim with optional ``PhysxMaterialAPI`` combine modes."""
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    prim = stage.GetPrimAtPath(mat_path)
    mat = UsdShade.Material(prim) if (prim and prim.IsValid()) else UsdShade.Material.Define(stage, mat_path)
    prim = mat.GetPrim()
    mat_api = UsdPhysics.MaterialAPI(prim) if prim.HasAPI(UsdPhysics.MaterialAPI) else UsdPhysics.MaterialAPI.Apply(prim)
    mat_api.CreateStaticFrictionAttr().Set(float(mat_spec.static_friction))
    mat_api.CreateDynamicFrictionAttr().Set(float(mat_spec.dynamic_friction))
    mat_api.CreateRestitutionAttr().Set(float(mat_spec.restitution))

    if (
        mat_spec.friction_combine_mode is not None
        or mat_spec.restitution_combine_mode is not None
        or mat_spec.compliant_contact_stiffness is not None
        or mat_spec.compliant_contact_damping is not None
    ):
        pm_api = (
            PhysxSchema.PhysxMaterialAPI(prim)
            if prim.HasAPI(PhysxSchema.PhysxMaterialAPI)
            else PhysxSchema.PhysxMaterialAPI.Apply(prim)
        )
        if mat_spec.friction_combine_mode is not None:
            pm_api.CreateFrictionCombineModeAttr().Set(str(mat_spec.friction_combine_mode))
        if mat_spec.restitution_combine_mode is not None:
            pm_api.CreateRestitutionCombineModeAttr().Set(str(mat_spec.restitution_combine_mode))
        if mat_spec.compliant_contact_stiffness is not None:
            pm_api.CreateCompliantContactStiffnessAttr().Set(float(mat_spec.compliant_contact_stiffness))
        if mat_spec.compliant_contact_damping is not None:
            pm_api.CreateCompliantContactDampingAttr().Set(float(mat_spec.compliant_contact_damping))
    return mat


def bind_usd_physics_material(prim: Any, mat: Any) -> None:
    """Bind ``mat`` to ``prim`` for ``purpose='physics'`` stronger than descendants."""
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics")


def _deinstance_hierarchy_for_colliders(root_prim: Any) -> None:
    from pxr import Usd

    # De-instance any instanceable prims under root_prim so collider attributes/bindings can be authored cleanly.
    changed = True
    while changed:
        changed = False
        for p in Usd.PrimRange(root_prim):
            if p.IsValid() and p.IsInstanceable():
                p.SetInstanceable(False)
                changed = True
                break


_VISUAL_BRANCH_TOKENS: frozenset[str] = frozenset({"visual", "visuals"})
_COLLISION_BRANCH_TOKENS: frozenset[str] = frozenset({"collision", "collisions"})


def classify_duplicate_colliders(
    root_path: str,
    gprim_colliders: Sequence[tuple[str, bool]],
    *,
    strip_pre_disabled: bool = True,
) -> dict[str, list[str]]:
    """Classify duplicate Gprim colliders under ``root_path`` generically from USD hierarchy structure (#24, #175).

    Given ``gprim_colliders`` as ``(prim_path, collision_enabled)`` pairs for all ``UsdGeom.Gprim`` descendants
    carrying ``UsdPhysics.CollisionAPI``:
    1. ``visual_duplicates``: any Gprim collider under a ``visual``/``visuals`` ancestor (case-insensitive, below
       ``root_path``) whose parent also has a sibling ``collision``/``collisions`` branch containing >= 1 enabled
       Gprim collider. Does not touch render-mesh colliders in assemblies without a sibling collision branch (e.g.
       ``Geometry/camera_stand/00000`` ``RSD455/Visual/*``) or intentional compound colliders under ``collision``/``Scope``.
    2. ``disabled_duplicates``: when ``strip_pre_disabled`` is True and >= 1 non-duplicate Gprim collider remains
       enabled under ``root_path``, any remaining Gprim collider authored with ``physics:collisionEnabled = False``
       in USD (removing ``UsdPhysics.CollisionAPI`` prevents PhysX from creating a disabled ``PxShape`` that still
       contributes 2x automatic density mass/inertia when ``MassAPI`` is not authored).
    """
    prefix = root_path.rstrip("/") + "/"
    parsed: list[tuple[str, tuple[str, ...], bool]] = []
    for p_str, is_en in gprim_colliders:
        rel = p_str[len(prefix) :] if p_str.startswith(prefix) else p_str.lstrip("/")
        parts = tuple(seg for seg in rel.split("/") if seg)
        parsed.append((p_str, parts, bool(is_en)))

    collision_branch_parents: set[tuple[str, ...]] = set()
    for _, parts, is_en in parsed:
        if not is_en:
            continue
        for idx, seg in enumerate(parts):
            if seg.lower() in _COLLISION_BRANCH_TOKENS:
                collision_branch_parents.add(parts[:idx])

    visual_dups: list[str] = []
    visual_dup_set: set[str] = set()
    for p_str, parts, _ in parsed:
        for idx, seg in enumerate(parts):
            if seg.lower() in _VISUAL_BRANCH_TOKENS and parts[:idx] in collision_branch_parents:
                visual_dups.append(p_str)
                visual_dup_set.add(p_str)
                break

    remaining_enabled = sum(1 for p_str, _, is_en in parsed if p_str not in visual_dup_set and is_en)
    disabled_dups: list[str] = []
    if strip_pre_disabled and remaining_enabled >= 1:
        for p_str, _, is_en in parsed:
            if p_str not in visual_dup_set and not is_en:
                disabled_dups.append(p_str)

    return {
        "visual_duplicates": visual_dups,
        "disabled_duplicates": disabled_dups,
    }


def _is_usd_collision_enabled(prim: Any) -> bool:
    en_attr = prim.GetAttribute("physics:collisionEnabled")
    return not (en_attr and en_attr.IsValid() and en_attr.Get() is False)


def author_usd_collision(root_prim: Any, col_spec: CollisionSpec) -> None:
    """Author ``UsdPhysics.CollisionAPI`` / ``PhysxSchema.PhysxCollisionAPI`` attributes on Gprim colliders under ``root_prim``."""
    from pxr import Usd, UsdGeom, UsdPhysics

    needs_descendant_edits = (
        col_spec.approximation != "asset"
        or col_spec.contact_offset_m is not None
        or col_spec.rest_offset_m is not None
        or col_spec.torsional_patch_radius_m is not None
        or col_spec.min_torsional_patch_radius_m is not None
        or col_spec.enabled is not None
    )

    # Read-only scan (traversing instance proxies) to detect duplicate colliders (#24, #175) without de-instancing
    # clean assets when col_spec carries default values.
    root_path_str = str(root_prim.GetPath())
    probe_entries = [
        (str(p.GetPath()), _is_usd_collision_enabled(p))
        for p in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
        if p.HasAPI(UsdPhysics.CollisionAPI) and bool(UsdGeom.Gprim(p))
    ]
    dups = classify_duplicate_colliders(
        root_path_str,
        probe_entries,
        strip_pre_disabled=(col_spec.enabled is not False),
    )
    dup_paths: set[str] = set(dups["visual_duplicates"]) | set(dups["disabled_duplicates"])

    if not needs_descendant_edits and not dup_paths:
        return
    _deinstance_hierarchy_for_colliders(root_prim)

    all_gprim_cols = [
        p
        for p in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
        if p.HasAPI(UsdPhysics.CollisionAPI) and bool(UsdGeom.Gprim(p)) and not p.IsInstanceProxy()
    ]
    if dup_paths:
        for cprim in all_gprim_cols:
            if str(cprim.GetPath()) in dup_paths:
                UsdPhysics.CollisionAPI(cprim).CreateCollisionEnabledAttr().Set(False)
                cprim.RemoveAPI(UsdPhysics.CollisionAPI)

    if not needs_descendant_edits:
        return

    cols = [cprim for cprim in all_gprim_cols if str(cprim.GetPath()) not in dup_paths]
    if not cols and root_prim.HasAPI(UsdPhysics.CollisionAPI) and bool(UsdGeom.Gprim(root_prim)):
        cols = [root_prim]

    for cprim in cols:
        c_api = UsdPhysics.CollisionAPI(cprim)
        if col_spec.enabled is not None:
            c_api.CreateCollisionEnabledAttr().Set(bool(col_spec.enabled))
        if col_spec.approximation != "asset":
            m_api = (
                UsdPhysics.MeshCollisionAPI(cprim)
                if cprim.HasAPI(UsdPhysics.MeshCollisionAPI)
                else UsdPhysics.MeshCollisionAPI.Apply(cprim)
            )
            m_api.CreateApproximationAttr().Set(str(col_spec.approximation))
        if (
            col_spec.contact_offset_m is not None
            or col_spec.rest_offset_m is not None
            or col_spec.torsional_patch_radius_m is not None
            or col_spec.min_torsional_patch_radius_m is not None
        ):
            from pxr import PhysxSchema

            pc_api = (
                PhysxSchema.PhysxCollisionAPI(cprim)
                if cprim.HasAPI(PhysxSchema.PhysxCollisionAPI)
                else PhysxSchema.PhysxCollisionAPI.Apply(cprim)
            )
            if col_spec.contact_offset_m is not None:
                pc_api.CreateContactOffsetAttr().Set(float(col_spec.contact_offset_m))
            if col_spec.rest_offset_m is not None:
                pc_api.CreateRestOffsetAttr().Set(float(col_spec.rest_offset_m))
            if col_spec.torsional_patch_radius_m is not None:
                pc_api.CreateTorsionalPatchRadiusAttr().Set(float(col_spec.torsional_patch_radius_m))
            if col_spec.min_torsional_patch_radius_m is not None:
                pc_api.CreateMinTorsionalPatchRadiusAttr().Set(float(col_spec.min_torsional_patch_radius_m))


def author_usd_mass(prim: Any, mass_kg: float, com_offset_m: tuple[float, float, float] | None = None, inertia_kg_m2: tuple[float, float, float] | None = None) -> None:
    """Author ``UsdPhysics.MassAPI`` on ``prim`` prior to body parsing."""
    from pxr import Gf, UsdPhysics

    m_api = UsdPhysics.MassAPI(prim) if prim.HasAPI(UsdPhysics.MassAPI) else UsdPhysics.MassAPI.Apply(prim)
    m_api.CreateMassAttr().Set(float(mass_kg))
    if com_offset_m is not None:
        m_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*[float(x) for x in com_offset_m]))
    if inertia_kg_m2 is not None:
        m_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*[float(x) for x in inertia_kg_m2]))
        m_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def author_rigid_physics(prim: Any, mat_path: str, spec: RigidSpec) -> Any:
    """Author USD material, binding, mass, and collision on ``RigidObject`` BEFORE ``SingleRigidPrim.__init__``."""
    stage = prim.GetStage()
    mat = author_usd_material(stage, mat_path, spec.material)
    if spec.author_pre_parse:
        bind_usd_physics_material(prim, mat)
    if spec.author_usd_mass:
        author_usd_mass(prim, spec.mass_kg, spec.com_offset_m, spec.inertia_kg_m2)
    if spec.author_collision:
        author_usd_collision(prim, spec.collision)
    return mat


def author_static_physics(prim: Any, mat_path: str, spec: StaticSpec) -> Any | None:
    """Author USD material and collision on ``GeometryObject`` / ``Room`` BEFORE ``SingleGeometryPrim.__init__``."""
    stage = prim.GetStage()
    mat = None
    if spec.bind_material:
        mat = author_usd_material(stage, mat_path, spec.material)
        bind_usd_physics_material(prim, mat)
    if spec.author_collision:
        author_usd_collision(prim, spec.collision)
    return mat


def author_table_physics(prim: Any, mat_path: str, spec: TableSpec) -> Any:
    """Author USD material (and mass if dynamic) on ``Table`` BEFORE ``SingleRigidPrim.__init__`` / MDL return."""
    from pxr import UsdPhysics

    stage = prim.GetStage()
    mat = author_usd_material(stage, mat_path, spec.material)
    if spec.bind_pre_parse:
        bind_usd_physics_material(prim, mat)
    if not spec.is_static and spec.mass_kg is not None and spec.bind_pre_parse:
        author_usd_mass(prim, spec.mass_kg)
    if spec.author_collision:
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
        author_usd_collision(prim, spec.collision)
    return mat


def author_articulation_physics(prim: Any, spec: ArticulationSpec) -> None:
    """Author per-link masses, materials, and joint drives on ``ArticulationObject`` BEFORE ``SingleArticulation.__init__``."""
    from pxr import PhysxSchema, Usd, UsdPhysics

    if not spec.author_pre_parse:
        return
    stage = prim.GetStage()
    prim_path = str(prim.GetPath())
    _deinstance_hierarchy_for_colliders(prim)

    if spec.default_material is not None:
        dmat = author_usd_material(stage, f"{prim_path}/physics_material", spec.default_material)
        bind_usd_physics_material(prim, dmat)

    for p in Usd.PrimRange(prim):
        lname = p.GetName()
        if lname in spec.link_masses_kg and p.HasAPI(UsdPhysics.RigidBodyAPI):
            author_usd_mass(p, spec.link_masses_kg[lname])
        if lname in spec.link_materials:
            lmat = author_usd_material(stage, f"{p.GetPath()}/physics_material", spec.link_materials[lname])
            bind_usd_physics_material(p, lmat)
        if lname in spec.joint_drives:
            drv = spec.joint_drives[lname]
            d_api = (
                UsdPhysics.DriveAPI(p, drv.drive_type)
                if p.HasAPI(UsdPhysics.DriveAPI, drv.drive_type)
                else UsdPhysics.DriveAPI.Apply(p, drv.drive_type)
            )
            unit_scale = (math.pi / 180.0) if drv.drive_type == "angular" else 1.0
            d_api.CreateStiffnessAttr().Set(float(drv.stiffness * unit_scale))
            d_api.CreateDampingAttr().Set(float(drv.damping * unit_scale))
            if drv.max_force is not None:
                d_api.CreateMaxForceAttr().Set(float(drv.max_force))
            if drv.max_velocity is not None:
                vel_scale = (180.0 / math.pi) if drv.drive_type == "angular" else 1.0
                pj = PhysxSchema.PhysxJointAPI(p) if p.HasAPI(PhysxSchema.PhysxJointAPI) else PhysxSchema.PhysxJointAPI.Apply(p)
                pj.CreateMaxJointVelocityAttr().Set(float(drv.max_velocity * vel_scale))

    if spec.author_collision:
        author_usd_collision(prim, spec.collision)


def author_dynamic_physics(prim: Any, mat_path: str, spec: DynamicSpec) -> None:
    """Author USD material and optional surface velocity on ``DynamicObject`` BEFORE ``XFormPrim``."""
    from pxr import Gf, PhysxSchema, Usd, UsdPhysics

    if not spec.author_pre_parse:
        return
    stage = prim.GetStage()
    if spec.material is not None:
        mat = author_usd_material(stage, mat_path, spec.material)
        bind_usd_physics_material(prim, mat)
    if spec.belt_velocity_m_s is not None:
        _deinstance_hierarchy_for_colliders(prim)
        for p in Usd.PrimRange(prim):
            if p.HasAPI(UsdPhysics.RigidBodyAPI) or p.HasAPI(UsdPhysics.CollisionAPI):
                sv = (
                    PhysxSchema.PhysxSurfaceVelocityAPI(p)
                    if p.HasAPI(PhysxSchema.PhysxSurfaceVelocityAPI)
                    else PhysxSchema.PhysxSurfaceVelocityAPI.Apply(p)
                )
                sv.CreateSurfaceVelocityEnabledAttr().Set(True)
                sv.CreateSurfaceVelocityAttr().Set(Gf.Vec3f(*[float(x) for x in spec.belt_velocity_m_s]))
                break
    if spec.author_collision:
        author_usd_collision(prim, spec.collision)


def author_robot_physics(stage: Any, env_root: str, robot_idx: int, spec: RobotSpec) -> None:
    """Author finger de-instancing, pad material bindings, torsional radii, the D3 gripper mimic, and camera inertials BEFORE ``clone_environments``."""
    robot_root_path = f"{env_root}/robot{robot_idx}"
    robot_prim = stage.GetPrimAtPath(robot_root_path) if stage is not None else None
    if not robot_prim or not robot_prim.IsValid():
        if spec.provenance == "shipped":
            return
        raise PhysicsSpecError(
            f"Missing robot prim at {robot_root_path!r} when authoring non-shipped robot physics",
            prim_path=robot_root_path,
            key=f"robots.{spec.robot_name}",
        )

    from pxr import Gf, PhysxSchema, Usd, UsdPhysics

    pad_links = spec.pad_links or (
        ("link7", "link8") if spec.robot_name == "x5" else ("panda_leftfinger", "panda_rightfinger")
    )
    deinstance_links = tuple(dict.fromkeys((*pad_links, *spec.link_materials.keys())))
    if spec.deinstance_fingers:
        for link in deinstance_links:
            col_group = stage.GetPrimAtPath(f"{robot_root_path}/{link}/collisions")
            if col_group and col_group.IsValid() and col_group.IsInstanceable():
                col_group.SetInstanceable(False)
            link_prim = stage.GetPrimAtPath(f"{robot_root_path}/{link}")
            if link_prim and link_prim.IsValid():
                _deinstance_hierarchy_for_colliders(link_prim)

    for link, mat_spec in spec.link_materials.items():
        link_prim = stage.GetPrimAtPath(f"{robot_root_path}/{link}")
        if not link_prim or not link_prim.IsValid():
            raise PhysicsSpecError(
                f"Missing robot link prim {robot_root_path}/{link!r}",
                prim_path=f"{robot_root_path}/{link}",
                key=f"robots.{spec.robot_name}.link_materials.{link}",
            )
        mat_leaf = "pad_physics_material" if link in pad_links else "link_physics_material"
        mat_path = f"{robot_root_path}/{link}/{mat_leaf}"
        mat = author_usd_material(stage, mat_path, mat_spec)
        bind_usd_physics_material(link_prim, mat)
        for p in Usd.PrimRange(link_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if p.HasAPI(UsdPhysics.CollisionAPI) and not p.IsInstanceProxy():
                bind_usd_physics_material(p, mat)
        proto = stage.GetPrimAtPath(f"{robot_root_path}/colliders/{link}")
        if proto and proto.IsValid():
            bind_usd_physics_material(proto, mat)

    if spec.gripper is not None:
        # D3 (#90): joint8 mimics joint7 (q8 + gearing * q7 + offset = 0). The instance token must be rot*; Omniverse's
        # parser silently ignores linear/transX tokens on prismatic joints (#86). joint8's drive gains are zeroed by
        # split_gripper_actuators before the articulation is created.
        g = spec.gripper
        mimic_prim = stage.GetPrimAtPath(f"{robot_root_path}/joints/{g.mimic_joint}")
        ref_prim = stage.GetPrimAtPath(f"{robot_root_path}/joints/{g.driven_joint}")
        if not mimic_prim or not mimic_prim.IsValid() or not ref_prim or not ref_prim.IsValid():
            raise PhysicsSpecError(
                f"Missing gripper joint prims {g.mimic_joint!r}/{g.driven_joint!r} under {robot_root_path}/joints",
                prim_path=f"{robot_root_path}/joints",
                key=f"robots.{spec.robot_name}.gripper",
            )
        mim = PhysxSchema.PhysxMimicJointAPI.Apply(mimic_prim, g.mimic_axis)
        mim.CreateReferenceJointRel().SetTargets([ref_prim.GetPath()])
        mim.CreateGearingAttr(float(g.mimic_gearing))
        mim.CreateOffsetAttr(float(g.mimic_offset))

    if spec.arm_limits is not None:
        # #11: joint ranges in USD (degrees for revolute joints) before PhysX parses the articulation; velocity and
        # effort limits go through the actuator config (apply_arm_limits_to_actuators), which IsaacLab writes at init.
        al = spec.arm_limits
        for j, lo, hi in zip(al.joints, al.lower_rad, al.upper_rad):
            jp = stage.GetPrimAtPath(f"{robot_root_path}/joints/{j}")
            if not jp or not jp.IsValid() or not jp.IsA(UsdPhysics.RevoluteJoint):
                raise PhysicsSpecError(
                    f"Missing revolute joint prim {robot_root_path}/joints/{j!r}",
                    prim_path=f"{robot_root_path}/joints/{j}",
                    key=f"robots.{spec.robot_name}.arm_limits",
                )
            rj = UsdPhysics.RevoluteJoint(jp)
            rj.CreateLowerLimitAttr().Set(float(math.degrees(lo)))
            rj.CreateUpperLimitAttr().Set(float(math.degrees(hi)))

    if spec.pad_torsional_radius_m is not None or spec.pad_min_torsional_radius_m is not None:
        for link in pad_links:
            link_prim = stage.GetPrimAtPath(f"{robot_root_path}/{link}")
            if not link_prim or not link_prim.IsValid():
                raise PhysicsSpecError(
                    f"Missing robot finger link prim {robot_root_path}/{link!r}",
                    prim_path=f"{robot_root_path}/{link}",
                    key=f"robots.{spec.robot_name}.pad_torsional_radius_m",
                )
            for p in Usd.PrimRange(link_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
                if p.HasAPI(UsdPhysics.CollisionAPI) and not p.IsInstanceProxy():
                    pc = (
                        PhysxSchema.PhysxCollisionAPI(p)
                        if p.HasAPI(PhysxSchema.PhysxCollisionAPI)
                        else PhysxSchema.PhysxCollisionAPI.Apply(p)
                    )
                    if spec.pad_torsional_radius_m is not None:
                        pc.CreateTorsionalPatchRadiusAttr().Set(float(spec.pad_torsional_radius_m))
                    if spec.pad_min_torsional_radius_m is not None:
                        pc.CreateMinTorsionalPatchRadiusAttr().Set(float(spec.pad_min_torsional_radius_m))

    for link, inert in spec.link_inertials.items():
        lp = stage.GetPrimAtPath(f"{robot_root_path}/{link}")
        if not lp or not lp.IsValid():
            raise PhysicsSpecError(
                f"Missing robot link prim {robot_root_path}/{link!r}",
                prim_path=f"{robot_root_path}/{link}",
                key=f"robots.{spec.robot_name}.link_inertials.{link}",
            )
        author_usd_mass(lp, inert.mass_kg, inert.com_offset_m, inert.inertia_kg_m2)

    if spec.sanitize_camera_inertials:
        # #48: ARX_physics.usd authors mass 0 and centerOfMass (-inf, -inf, -inf) on camera_base/camera
        for cam_link in ("camera_base", "camera"):
            if cam_link in spec.link_inertials:
                continue
            cp = stage.GetPrimAtPath(f"{robot_root_path}/{cam_link}")
            if not cp or not cp.IsValid() or not cp.HasAPI(UsdPhysics.MassAPI):
                continue
            m_api = UsdPhysics.MassAPI(cp)
            com_attr = m_api.GetCenterOfMassAttr()
            if com_attr and com_attr.IsValid():
                com_val = com_attr.Get()
                if com_val is not None and any(not math.isfinite(float(c)) for c in com_val):
                    com_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
            mass_attr = m_api.GetMassAttr()
            if mass_attr and mass_attr.IsValid():
                mv = mass_attr.Get()
                if mv is not None and (not math.isfinite(float(mv)) or float(mv) <= 0.0):
                    mass_attr.Set(1e-4)
