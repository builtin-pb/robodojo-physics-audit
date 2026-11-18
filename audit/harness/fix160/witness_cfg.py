"""#160 disturbance witness: configuration checks and override readback (standard library only, unit-tested on CPU).

The #163 review (problem 5) found that a misspelt variant key (``joint7_effort_limit_N``) silently ran the base limit
under the requested variant's name. Every key is now checked before the run, and every runtime override is compared
with the engine's readback after it is applied.
"""
from __future__ import annotations

import math

CFG_KEYS = frozenset({"bottles", "inputs", "variants", "protocol", "ik_seed_current"})
PROTOCOL_KEYS = frozenset(
    {"quiet_actions", "tap_N", "tap_steps", "after_actions", "jerk_after_actions", "release_gap_mm", "support_close"}
)
VARIANT_KEYS = frozenset({"name", "bottles", "armature_kg", "joint7_effort_limit_n", "retarget_frac", "latch_at_freeze_frac"})


class WitnessConfigError(ValueError):
    pass


def _pos(v, key):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0.0:
        raise WitnessConfigError(f"{key} must be a finite number > 0, got {v!r}")


def validate_cfg(cfg, input_keys=None):
    """Raise ``WitnessConfigError`` on any unknown key, missing name, duplicate name or malformed override."""
    if not isinstance(cfg, dict):
        raise WitnessConfigError("disturb config must be a JSON object")
    unknown = sorted(set(cfg) - CFG_KEYS)
    if unknown:
        raise WitnessConfigError(f"unknown disturb config key(s) {unknown}; allowed {sorted(CFG_KEYS)}")
    unknown = sorted(set(cfg.get("protocol") or {}) - PROTOCOL_KEYS)
    if unknown:
        raise WitnessConfigError(f"unknown protocol key(s) {unknown}; allowed {sorted(PROTOCOL_KEYS)}")
    variants = cfg.get("variants")
    if not isinstance(variants, list) or not variants:
        raise WitnessConfigError("variants must be a non-empty list")
    names = set()
    for i, v in enumerate(variants):
        if not isinstance(v, dict):
            raise WitnessConfigError(f"variants[{i}] must be an object")
        unknown = sorted(set(v) - VARIANT_KEYS)
        if unknown:
            raise WitnessConfigError(f"variants[{i}] has unknown key(s) {unknown}; allowed {sorted(VARIANT_KEYS)}")
        name = v.get("name")
        if not isinstance(name, str) or not name or name in names:
            raise WitnessConfigError(f"variants[{i}] needs a unique non-empty name, got {name!r}")
        names.add(name)
        if "armature_kg" in v:
            a = v["armature_kg"]
            if not isinstance(a, list) or len(a) != 2 or any(
                isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or x < 0.0 for x in a
            ):
                raise WitnessConfigError(f"variants[{i}].armature_kg must be [joint7, joint8] >= 0, got {a!r}")
        for key in ("joint7_effort_limit_n", "retarget_frac", "latch_at_freeze_frac"):
            if key in v:
                _pos(v[key], f"variants[{i}].{key}")
        if "retarget_frac" in v and "latch_at_freeze_frac" in v:
            raise WitnessConfigError(f"variants[{i}]: retarget_frac and latch_at_freeze_frac are exclusive")
    for where, bottles in [("bottles", cfg.get("bottles"))] + [(f"variants[{i}].bottles", v["bottles"]) for i, v in enumerate(variants) if "bottles" in v]:
        if not isinstance(bottles, list) or not bottles or not all(isinstance(b, str) for b in bottles):
            raise WitnessConfigError(f"{where} must be a non-empty list of input keys")
        if input_keys is not None and set(bottles) - set(input_keys):
            raise WitnessConfigError(f"{where} names unknown input(s) {sorted(set(bottles) - set(input_keys))}")


def expected_drive(v, base_armature, base_fmax):
    """The joint7/joint8 armature and joint7 effort limit the engine should read back after ``v`` is applied."""
    a7, a8 = v.get("armature_kg", base_armature)
    return {"armature_eng": [float(a7), float(a8)], "Fmax7_eng": float(v.get("joint7_effort_limit_n", base_fmax))}


def readback_mismatches(v, readback, base_armature, base_fmax, rtol=1e-4):
    """Compare a variant's requested overrides with ``p32_gripper.read_drive_and_pos`` output; empty when they match."""
    exp = expected_drive(v, base_armature, base_fmax)
    got = {"armature_eng": [float(x) for x in readback["armature_eng"]], "Fmax7_eng": float(readback["Fmax_eng"][0])}
    out = []
    for e, g, label in zip(exp["armature_eng"] + [exp["Fmax7_eng"]], got["armature_eng"] + [got["Fmax7_eng"]],
                           ("joint7 armature", "joint8 armature", "joint7 effort limit")):
        if abs(g - e) > rtol * max(1.0, abs(e)):
            out.append(f"{label}: engine {g} != requested {e}")
    return out
