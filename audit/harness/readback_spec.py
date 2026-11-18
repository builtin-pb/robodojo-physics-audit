"""Engine readback, positive-control canary touches, and spec assertions (#62 PR 1).

Implements the union readback from #62 D2:
- All collider shapes (no [:2] truncation) on rigid objects, articulations, and robots
  across all parallel environments ``0 .. num_envs - 1``, including per-link shape slices
  on robot finger links (``link7`` and ``link8`` asserted individually on each robot).
- Engine masses, finite centres of mass, positive diagonal inertias (plus ``I / m`` shape
  factors), and robot + articulation drive gains/limits from PhysX tensor views.
- Collider counts, enabled flags, approximations, offsets, and local-frame scaled
  collider AABB extents (via ``ComputeRelativeBound(c, root)`` over enabled ``UsdGeom.Gprim``
  colliders) vs resolved ``expected_extents_m`` (<= 2% enforced on rigid, geometry, dynamic,
  and fixtures when ``size`` is active).
- Static colliders (Geometry, Room, Table, Ground, Dynamic) and de-instanced robot
  finger pads verified via USD ``ComputeBoundMaterial('physics')``, active PhysX
  ``get_contact_report()`` canary touches (checking BOTH touched collider paths and
  material-path-to-USD numeric values), and 100-step Coulomb slide witnesses on Table
  and Ground.
- Explicit contact-evidence accounting: on ``cpu``, empty resting contacts when rigid
  objects are present fail the assertion; on ``cuda:0`` (Direct GPU mode, where CPU
  ``get_contact_report()`` is unsupported), contact evidence is explicitly marked
  ``absent_direct_gpu_contact_report_unsupported`` rather than passing on ``[]``.
"""

from __future__ import annotations

import math
import os
import tempfile
from typing import Any

try:
    import numpy as np
except ImportError:  # pragma: no cover - allows stdlib-only unit tests to import pure helpers
    np = None  # type: ignore[assignment]

try:
    import torch
except ImportError:  # pragma: no cover - allows stdlib-only unit tests to import pure helpers
    torch = None  # type: ignore[assignment]


# Witness-side collider width compensations: mask #95 (V-HACD voxelization erosion) and #25 (authored minThickness=0.02
# / 16-vertex hull floor on thin plates, discs, and small pods under size_decision='asset', #175).
VHACD_EROSION_ALLOWANCE_ASSETS: frozenset[str] = frozenset(
    {
        "Clutter/balance/00000",
        "Clutter/ball/00001",
        "Clutter/baseball_bat/00000",
        "Clutter/body_lotion/00000",
        "Clutter/bottle/00000",
        "Clutter/bridge/00000",
        "Clutter/cake/00000",
        "Clutter/cap/00000",
        "Clutter/comb/00000",
        "Clutter/cup/00000",
        "Clutter/dog_toy/00000",
        "Clutter/french_fries/00000",
        "Clutter/game_machine/00000",
        "Clutter/gamepad/00000",
        "Clutter/garage/00003",
        "Clutter/garage/00004",
        "Clutter/garage/00005",
        "Clutter/garage/00006",
        "Clutter/garage/00007",
        "Clutter/garage/00008",
        "Clutter/garage/00009",
        "Clutter/garage/00010",
        "Clutter/garage/00011",
        "Clutter/garage/00014",
        "Clutter/garage/00015",
        "Clutter/garage/00016",
        "Clutter/garage/00017",
        "Clutter/hammer/00000",
        "Clutter/helmets/00000",
        "Clutter/helmets/00001",
        "Clutter/jeans/00000",
        "Clutter/kettle/00000",
        "Clutter/lamp/00000",
        "Clutter/magic_potion/00000",
        "Clutter/microscope/00000",
        "Clutter/mini_fan/00000",
        "Clutter/mini_fan/00001",
        "Clutter/mini_fan/00002",
        "Clutter/palette/00000",
        "Clutter/piano/00000",
        "Clutter/plant/00000",
        "Clutter/plate/00000",
        "Clutter/plier/00000",
        "Clutter/pokeball/00001",
        "Clutter/pokeball/00003",
        "Clutter/racket/00000",
        "Clutter/racket/00001",
        "Clutter/racket/00003",
        "Clutter/racket/00004",
        "Clutter/sandwich/00000",
        "Clutter/screwdriver/00000",
        "Clutter/sculpture/00001",
        "Clutter/sculpture/00002",
        "Clutter/shampoo/00000",
        "Clutter/stapler/00001",
        "Clutter/stapler/00003",
        "Clutter/tissue/00002",
        "Clutter/toothpaste/00000",
        "Clutter/toothpaste/00001",
        "Clutter/toy/00002",
        "Clutter/toy_car/00000",
        "Clutter/toy_plane/00000",
        "Clutter/toy_plane/00001",
        "Clutter/toy_plane/00002",
        "Clutter/walkie_talkie/00000",
        "Clutter/wrench/00000",
        "Geometry/monitor/00000",
        "Geometry/monitor/00001",
        "Geometry/vase/00000",
        "Geometry/vase/00001",
        "Rigid/bottle/00000",
        "Rigid/bottle/00002",
        "Rigid/bottle/00050",
        "Rigid/bottle/00059",
        "Rigid/bottle/00060",
        "Rigid/bottle_opener/00000",
        "Rigid/broom/00000",
        "Rigid/cactus/00000",
        "Rigid/can/00000",
        "Rigid/car/00000",
        "Rigid/car/00002",
        "Rigid/car/00004",
        "Rigid/car/00005",
        "Rigid/car/00007",
        "Rigid/car/00008",
        "Rigid/car/00009",
        "Rigid/car/00011",
        "Rigid/car/00012",
        "Rigid/car/00014",
        "Rigid/chocolate_bag/00000",
        "Rigid/drawing_compass/00000",
        "Rigid/electric_toothbrush/00000",
        "Rigid/electric_toothbrush/00001",
        "Rigid/electric_toothbrush/00002",
        "Rigid/electric_toothbrush/00003",
        "Rigid/electric_toothbrush/00004",
        "Rigid/electric_toothbrush/00005",
        "Rigid/electric_toothbrush/00006",
        "Rigid/figurine/00000",
        "Rigid/garage/00005",
        "Rigid/garage/00006",
        "Rigid/garage/00007",
        "Rigid/garage/00008",
        "Rigid/garage/00011",
        "Rigid/garage/00015",
        "Rigid/grapes/00000",
        "Rigid/hammer/00000",
        "Rigid/hammer/00001",
        "Rigid/hammer/00002",
        "Rigid/hammer/00004",
        "Rigid/hammer/00005",
        "Rigid/hammer/00008",
        "Rigid/handled_paint/00000",
        "Rigid/headphone/00000",
        "Rigid/keyboard/00000",
        "Rigid/keyboard/00001",
        "Rigid/keyboard/00002",
        "Rigid/mallet_stand/00000",
        "Rigid/matryoshka_dolls/00000",
        "Rigid/matryoshka_dolls/00001",
        "Rigid/matryoshka_dolls/00002",
        "Rigid/matryoshka_dolls/00003",
        "Rigid/matryoshka_dolls/00004",
        "Rigid/matryoshka_dolls/00006",
        "Rigid/matryoshka_dolls/00009",
        "Rigid/matryoshka_dolls/00010",
        "Rigid/matryoshka_dolls/00011",
        "Rigid/matryoshka_dolls/00012",
        "Rigid/matryoshka_dolls/00013",
        "Rigid/matryoshka_dolls/00017",
        "Rigid/matryoshka_dolls/00018",
        "Rigid/matryoshka_dolls/00019",
        "Rigid/matryoshka_dolls/00020",
        "Rigid/matryoshka_dolls/00021",
        "Rigid/matryoshka_dolls/00024",
        "Rigid/mouse/00004",
        "Rigid/mouse/00006",
        "Rigid/mouse/00007",
        "Rigid/mug/00011",
        "Rigid/mug/00012",
        "Rigid/mug/00013",
        "Rigid/number/00039",
        "Rigid/owl/00000",
        "Rigid/paper_ball/00000",
        "Rigid/paper_ball/00001",
        "Rigid/paper_ball/00002",
        "Rigid/pen/00000",
        "Rigid/pen/00001",
        "Rigid/pen/00004",
        "Rigid/pen/00010",
        "Rigid/pen/00011",
        "Rigid/pepper/00001",
        "Rigid/pepper/00002",
        "Rigid/piler/00000",
        "Rigid/pliers/00000",
        "Rigid/pot/00001",
        "Rigid/prayer_bead/00000",
        "Rigid/saw/00000",
        "Rigid/scan/00000",
        "Rigid/scissor/00000",
        "Rigid/shoe/00001",
        "Rigid/shoe/00002",
        "Rigid/shoe/00003",
        "Rigid/shoe/00004",
        "Rigid/shoe/00005",
        "Rigid/shoe/00006",
        "Rigid/shoe/00007",
        "Rigid/shoe/00008",
        "Rigid/shoe/00010",
        "Rigid/shoe/00011",
        "Rigid/shoe/00013",
        "Rigid/shoe/00014",
        "Rigid/shoe/00015",
        "Rigid/shoe/00017",
        "Rigid/shovel/00002",
        "Rigid/teddy_bear/00000",
        "Rigid/toy/00000",
        "Rigid/toy/00002",
        "Rigid/toy_car/00002",
        "Rigid/toy_car/00003",
        "Rigid/waffle/00000",
        "Rigid/watch/00003",
        "Rigid/watch/00007",
        "Rigid/whisk/00000",
        "Rigid/wooden_toy/00001",
        "Rigid/wooden_toy/00003",
        "Rigid/wooden_toy/00004",
        "Rigid/wrench/00000",
    }
)
VHACD_LARGE_SPAN_5MM_ASSETS: frozenset[str] = frozenset(
    {
        "Clutter/dog_toy/00000",
        "Clutter/magic_potion/00000",
        "Geometry/monitor/00000",
        "Geometry/monitor/00001",
        "Rigid/pot/00001",
        "Rigid/shoe/00015",
        "Rigid/toy/00000",
    }
)
VHACD_MIN_THICKNESS_ALLOWANCE_ASSETS: frozenset[str] = frozenset(
    {
        "Clutter/audio/00000",
        "Clutter/audio/00001",
        "Clutter/audio/00002",
        "Clutter/bag/00000",
        "Clutter/balance/00000",
        "Clutter/ball/00000",
        "Clutter/ball/00001",
        "Clutter/baseball_bat/00000",
        "Clutter/body_lotion/00000",
        "Clutter/book/00000",
        "Clutter/book/00001",
        "Clutter/book/00002",
        "Clutter/book/00003",
        "Clutter/book/00004",
        "Clutter/book_shelf/00000",
        "Clutter/bottle/00000",
        "Clutter/bowl/00000",
        "Clutter/bowl/00001",
        "Clutter/box/00000",
        "Clutter/box/00001",
        "Clutter/box/00002",
        "Clutter/box/00003",
        "Clutter/box/00004",
        "Clutter/bridge/00000",
        "Clutter/cake/00000",
        "Clutter/cake/00001",
        "Clutter/cake/00002",
        "Clutter/calculator/00000",
        "Clutter/calculator/00001",
        "Clutter/calendar/00000",
        "Clutter/can/00000",
        "Clutter/candle/00000",
        "Clutter/cap/00000",
        "Clutter/cap/00001",
        "Clutter/card/00000",
        "Clutter/card/00001",
        "Clutter/cereal/00000",
        "Clutter/chinese_pavillion/00000",
        "Clutter/chocolate/00000",
        "Clutter/clip/00000",
        "Clutter/coffee_bag/00000",
        "Clutter/coin/00000",
        "Clutter/coin/00001",
        "Clutter/comb/00000",
        "Clutter/croissant/00000",
        "Clutter/cup/00000",
        "Clutter/cup/00001",
        "Clutter/cup/00002",
        "Clutter/cup/00003",
        "Clutter/cutter/00000",
        "Clutter/darts_board/00000",
        "Clutter/dice/00000",
        "Clutter/earbuds/00000",
        "Clutter/earth_globe/00000",
        "Clutter/eraser/00000",
        "Clutter/foosball_table/00000",
        "Clutter/french_fries/00000",
        "Clutter/game_machine/00000",
        "Clutter/gamepad/00000",
        "Clutter/garage/00000",
        "Clutter/garage/00001",
        "Clutter/garage/00002",
        "Clutter/garage/00003",
        "Clutter/garage/00004",
        "Clutter/garage/00005",
        "Clutter/garage/00006",
        "Clutter/garage/00007",
        "Clutter/garage/00008",
        "Clutter/garage/00009",
        "Clutter/garage/00010",
        "Clutter/garage/00011",
        "Clutter/garage/00012",
        "Clutter/garage/00013",
        "Clutter/garage/00014",
        "Clutter/garage/00015",
        "Clutter/garage/00016",
        "Clutter/garage/00017",
        "Clutter/glasses/00000",
        "Clutter/glasses/00001",
        "Clutter/graphics_card/00000",
        "Clutter/graphics_card/00001",
        "Clutter/hammer/00000",
        "Clutter/headphones/00000",
        "Clutter/helmets/00000",
        "Clutter/helmets/00001",
        "Clutter/house/00000",
        "Clutter/house/00001",
        "Clutter/house/00002",
        "Clutter/ice_cream/00000",
        "Clutter/ice_cream/00001",
        "Clutter/iron/00000",
        "Clutter/jeans/00000",
        "Clutter/kettle/00000",
        "Clutter/kettle/00001",
        "Clutter/key/00000",
        "Clutter/keyboard/00000",
        "Clutter/keyboard/00001",
        "Clutter/lamp/00000",
        "Clutter/mask/00000",
        "Clutter/meta_quest/00000",
        "Clutter/microscope/00000",
        "Clutter/minecraft_blocks/00000",
        "Clutter/minecraft_blocks/00001",
        "Clutter/minecraft_blocks/00002",
        "Clutter/minecraft_blocks/00003",
        "Clutter/minecraft_blocks/00004",
        "Clutter/minecraft_blocks/00005",
        "Clutter/minecraft_blocks/00006",
        "Clutter/minecraft_blocks/00007",
        "Clutter/minecraft_blocks/00008",
        "Clutter/minecraft_blocks/00009",
        "Clutter/minecraft_blocks/00010",
        "Clutter/minecraft_blocks/00011",
        "Clutter/minecraft_blocks/00012",
        "Clutter/minecraft_blocks/00013",
        "Clutter/minecraft_blocks/00014",
        "Clutter/minecraft_blocks/00015",
        "Clutter/mini_fan/00000",
        "Clutter/mini_fan/00001",
        "Clutter/mini_fan/00002",
        "Clutter/money_stack/00000",
        "Clutter/monitor/00000",
        "Clutter/notes_holder/00000",
        "Clutter/nvlink/00000",
        "Clutter/office_mail/00000",
        "Clutter/office_mail/00001",
        "Clutter/oscilloscope/00000",
        "Clutter/palette/00000",
        "Clutter/pen/00000",
        "Clutter/pen/00001",
        "Clutter/pen/00002",
        "Clutter/pencil_case/00000",
        "Clutter/phone_stand/00000",
        "Clutter/phone_stand/00001",
        "Clutter/piano/00000",
        "Clutter/plane/00000",
        "Clutter/plant/00000",
        "Clutter/plant/00001",
        "Clutter/plate/00000",
        "Clutter/plate/00001",
        "Clutter/plier/00000",
        "Clutter/pokeball/00000",
        "Clutter/pokeball/00001",
        "Clutter/pokeball/00002",
        "Clutter/pokeball/00003",
        "Clutter/pokeball/00004",
        "Clutter/pokeball/00005",
        "Clutter/pokeball/00006",
        "Clutter/pokeball/00007",
        "Clutter/pokeball/00008",
        "Clutter/power_bank/00000",
        "Clutter/puzzle_pieces/00000",
        "Clutter/racket/00000",
        "Clutter/racket/00001",
        "Clutter/racket/00002",
        "Clutter/racket/00003",
        "Clutter/racket/00004",
        "Clutter/sandwich/00000",
        "Clutter/scale/00000",
        "Clutter/scale/00001",
        "Clutter/screwdriver/00000",
        "Clutter/sculpture/00000",
        "Clutter/sculpture/00001",
        "Clutter/sculpture/00002",
        "Clutter/shampoo/00000",
        "Clutter/sound_card/00000",
        "Clutter/stapler/00000",
        "Clutter/stapler/00001",
        "Clutter/stapler/00002",
        "Clutter/stapler/00003",
        "Clutter/stapler/00004",
        "Clutter/tape_measure/00000",
        "Clutter/tissue/00000",
        "Clutter/tissue/00001",
        "Clutter/tissue/00002",
        "Clutter/tissue/00003",
        "Clutter/toothpaste/00000",
        "Clutter/toothpaste/00001",
        "Clutter/toy/00000",
        "Clutter/toy/00001",
        "Clutter/toy/00002",
        "Clutter/toy_car/00000",
        "Clutter/toy_car/00001",
        "Clutter/toy_plane/00000",
        "Clutter/toy_plane/00001",
        "Clutter/toy_plane/00002",
        "Clutter/trophy/00000",
        "Clutter/trophy/00001",
        "Clutter/walkie_talkie/00000",
        "Clutter/wrench/00000",
        "Geometry/cube_cushion/00001",
        "Geometry/cube_cushion/00002",
        "Rigid/action_camera/00000",
        "Rigid/action_camera/00001",
        "Rigid/coin/00000",
        "Rigid/corkscrew/00000",
        "Rigid/earbuds/00000",
        "Rigid/earbuds/00001",
        "Rigid/earbuds/00002",
        "Rigid/matryoshka_dolls/00007",
        "Rigid/pen_holder/00000",
        "Rigid/shovel/00001",
        "Rigid/tape_measure/00000",
        "Rigid/watch/00002",
        "Rigid/wrench/00001",
    }
)
VHACD_MIN_THICKNESS_WIDENING_CAP_M: float = 0.0012
# Clutter/card/00001 (66.6x91.9x14.8 mm, diagonal 114.5 mm) authors minThickness=0.02 (0.02 * diag = 2.29 mm floor),
# widening z from 14.79 mm to 16.68 mm (1.892 mm <= 2.0 mm, #25).
VHACD_MIN_THICKNESS_2MM_ASSETS: frozenset[str] = frozenset({"Clutter/card/00001"})
VHACD_MIN_THICKNESS_2MM_WIDENING_CAP_M: float = 0.0020
# Verified in isaac-sim_5.1.0.sif:
# - PhysxConvexHullCollisionAPI declares `int physxConvexHullCollision:hullVertexLimit = 64`
#   (/isaac-sim/extscache/omni.usd.schema.physx-107.3.26+107.3.3.lx64.r.cp311.u353/plugins/PhysxSchema/resources/schema.usda:858,
#   usdrt/scenegraph/usd/physxSchema/physxConvexHullCollisionAPI.h:124).
# - omni.kit.property.physx database.py:954 sets `"physxConvexHullCollision:hullVertexLimit": InfoData(8, 64, 1)` (min=8, max=64).
CONVEX_HULL_VERTEX_LIMIT_ALLOWANCE_ASSETS: frozenset[str] = frozenset(
    {
        "Rigid/bottle/00005",
        "Rigid/car/00001",
        "Rigid/car/00003",
        "Rigid/car/00006",
        "Rigid/oil_pen/00000",
        "Rigid/oil_pen/00001",
    }
)
COLLIDER_NARROWING_ALLOWANCE_M: float = 0.004
VHACD_LARGE_SPAN_ALLOWANCE_M: float = 0.0052
# PhysX `sq.overlap_box` takes float32 `carb.Float3`/`carb.Float4` (`ulp(1.7 m) = 1.2e-7 m`, ~1-2 um across rotated poses);
# un-eroded axes (e.g. flat z=0.174 m caps of 00005, rel_err ~ 6e-6) read +/-1e-6 m. Any genuine widening (>= 6 um)
# or any widening on an axis exceeding 2% is rejected unless scoped under VHACD_MIN_THICKNESS_ALLOWANCE_ASSETS (#25).
OVERLAP_BOX_FLOAT32_EPS_M: float = 5e-6


def effective_collider_narrowing_allowance_m(
    asset_id: str | None,
    allowance_m: float = COLLIDER_NARROWING_ALLOWANCE_M,
) -> float:
    """Effective narrowing cap (m) enforced and logged for ``asset_id``."""
    return (
        max(allowance_m, VHACD_LARGE_SPAN_ALLOWANCE_M)
        if (asset_id in VHACD_LARGE_SPAN_5MM_ASSETS and allowance_m == COLLIDER_NARROWING_ALLOWANCE_M)
        else allowance_m
    )


def effective_min_thickness_widening_cap_m(asset_id: str | None) -> float:
    """Effective #25 ``minThickness`` widening cap (m) enforced and logged for ``asset_id``."""
    return (
        VHACD_MIN_THICKNESS_2MM_WIDENING_CAP_M
        if asset_id in VHACD_MIN_THICKNESS_2MM_ASSETS
        else VHACD_MIN_THICKNESS_WIDENING_CAP_M
    )


def collider_compensation_logged_limits(
    asset_id: str | None,
    comp_kind: str,
    allowance_m: float = COLLIDER_NARROWING_ALLOWANCE_M,
) -> tuple[float, str]:
    """Return ``(allowance_m, one_sided)`` from the exact constants enforced by ``evaluate_collider_narrowing_allowance``."""
    eff_allow_m = effective_collider_narrowing_allowance_m(asset_id, allowance_m)
    if comp_kind == "vhacd_min_thickness_allowance":
        widen_cap = effective_min_thickness_widening_cap_m(asset_id)
        one_sided = f"min_thickness_widening_le_{widen_cap}m"
    else:
        one_sided = "engine_narrower_than_usd_only"
    return eff_allow_m, one_sided


def evaluate_collider_narrowing_allowance(
    *,
    asset_id: str | None,
    approximations: list[str],
    usd_aabb_extents_m: list[float] | None,
    engine_aabb_extents_m: list[float] | None,
    expected_extents_m: list[float] | None,
    allowance_m: float = COLLIDER_NARROWING_ALLOWANCE_M,
    probe_eps_m: float = OVERLAP_BOX_FLOAT32_EPS_M,
) -> tuple[bool, str | None, list[float] | None]:
    """Check witness-side collider AABB allowances against USD for named #95 and #25 assets."""
    if usd_aabb_extents_m is None or engine_aabb_extents_m is None or expected_extents_m is None:
        return False, None, None
    if len(usd_aabb_extents_m) != 3 or len(engine_aabb_extents_m) != 3 or len(expected_extents_m) != 3:
        return False, None, None

    narrowing_vs_usd_m = [round(float(usd_aabb_extents_m[k]) - float(engine_aabb_extents_m[k]), 6) for k in range(3)]

    kind: str | None = None
    eff_allowance_m = effective_collider_narrowing_allowance_m(asset_id, allowance_m)
    if asset_id in VHACD_MIN_THICKNESS_ALLOWANCE_ASSETS and "convexDecomposition" in approximations:
        kind = "vhacd_min_thickness_allowance"
        widen_cap = effective_min_thickness_widening_cap_m(asset_id)
        for k in range(3):
            u_k = float(usd_aabb_extents_m[k])
            e_k = float(engine_aabb_extents_m[k])
            m_k = float(expected_extents_m[k])
            nar_u = round(u_k - e_k, 9)
            nar_m = round(m_k - e_k, 9)
            if nar_u < -widen_cap or nar_u > eff_allowance_m:
                return False, kind, narrowing_vs_usd_m
            if nar_m < -widen_cap or nar_m > eff_allowance_m:
                return False, kind, narrowing_vs_usd_m
        return True, kind, narrowing_vs_usd_m
    elif asset_id in VHACD_EROSION_ALLOWANCE_ASSETS and approximations == ["convexDecomposition"]:
        kind = "vhacd_voxelization_erosion_allowance"
    elif asset_id in CONVEX_HULL_VERTEX_LIMIT_ALLOWANCE_ASSETS and approximations == ["convexHull"]:
        kind = "convex_hull_vertex_limit_allowance"
    else:
        return False, None, narrowing_vs_usd_m

    for k in range(3):
        u_k = float(usd_aabb_extents_m[k])
        e_k = float(engine_aabb_extents_m[k])
        m_k = float(expected_extents_m[k])
        nar_u = round(u_k - e_k, 9)
        nar_m = round(m_k - e_k, 9)
        # One-sided against USD: engine collider may never be wider than USD (up to 5 um float32 overlap_box quantum
        # on un-eroded axes), and at most eff_allowance_m narrower.
        if nar_u < -probe_eps_m or nar_u > eff_allowance_m:
            return False, kind, narrowing_vs_usd_m
        rel_k = abs(nar_m) / max(m_k, 1e-9)
        if rel_k > 0.02:
            if nar_u <= 0.0 or nar_m <= 0.0 or nar_m > eff_allowance_m:
                return False, kind, narrowing_vs_usd_m

    return True, kind, narrowing_vs_usd_m


def is_sdf_trimesh_face_index_report_null(
    *,
    c0_approx: str | None,
    c1_approx: str | None,
    mpair: list[str] | tuple[str, str],
    c0_usd_mat: dict[str, Any],
    c1_usd_mat: dict[str, Any],
) -> bool:
    """Return True iff ``/physicsScene/defaultMaterial`` in a CPU contact report is strictly PhysX ``NpShape.cpp:566``'s
    ``PxShape::getMaterialFromInternalFaceIndex(0xFFFFffff) -> NULL`` serialization artifact on an ``sdf`` x ``none``
    (triangle-mesh) contact whose triangle-mesh collider is explicitly bound in USD (verified in solver by Coulomb slide:
    ``arrange_largest_number`` ``digit_0`` decelerates at ``-0.01094 m/s`` per 4 ms step at ``table_laminate=0.199829``
    vs ``-0.01683 m/s`` and ``28.71 mm`` at ``table_laminate=0.500000``).
    """
    if len(mpair) != 2 or "/physicsScene/defaultMaterial" not in mpair:
        return False
    approxs = (str(c0_approx or ""), str(c1_approx or ""))
    umats = (c0_usd_mat, c1_usd_mat)
    for side in (0, 1):
        if mpair[side] != "/physicsScene/defaultMaterial":
            continue
        other = 1 - side
        if approxs[side] != "none" or approxs[other] != "sdf":
            return False
        um = umats[side] or {}
        if um.get("purpose") in (None, "scene-default") or um.get("static_friction") is None:
            return False
    return True


# Realised-pair readback sides: rigid objects pair with every partner and with each other; partners are static colliders
# (fixtures and Geometry) and robot links.
STATIC_PARTNER_KINDS: frozenset[str] = frozenset({"fixture", "geometry"})


def robot_link_static_pairs(partner_sides: list[dict[str, Any]]) -> list[tuple[dict[str, Any], dict[str, Any]]]:
    """Every robot-link side against every static side (#167: the pads on the table in each swap_T grasp; links on Table,
    Ground, Room, the dustbin and the camera stand). Static-static pairs never form in PhysX; rigid objects are paired
    elsewhere."""
    links = [p for p in partner_sides if p.get("kind") == "robot_link"]
    statics = [p for p in partner_sides if p.get("kind") in STATIC_PARTNER_KINDS]
    return [(lk, st) for lk in links for st in statics]


def robot_link_pairs(partner_sides: list[dict[str, Any]]) -> list[tuple[dict[str, Any], dict[str, Any]]]:
    """Robot-link sides against each other, self pairs included: the two arms' links meeting (finger on finger in a
    handover, camera on pad). The readback asserts every robot's per-link engine materials against one spec, so
    robot0's sides stand for robot1's."""
    links = [p for p in partner_sides if p.get("kind") == "robot_link"]
    return [(links[i], links[j]) for i in range(len(links)) for j in range(i, len(links))]


def _materials_on(resolver: Any) -> bool:
    """Table materials are active: ``values`` or its ``materials`` part (G4 split)."""
    return bool(getattr(resolver, "materials_values", "values" in resolver.fixes))


def _attr(prim: Any, name: str) -> Any:
    a = prim.GetAttribute(name)
    if not a or not a.IsValid():
        return None
    v = a.Get()
    if hasattr(v, "__float__") and not isinstance(v, bool):
        try:
            return float(v)
        except Exception:
            return str(v)
    return v if isinstance(v, (bool, int, float, str, type(None))) else str(v)


def solver_scene_check(scenes: dict[str, dict[str, Any]], authored_position_iterations: list[float | None], expected: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    """Pure check of the #62 ``solver`` fix: each physics scene's stabilization and minimum position iterations against
    ``expected`` (``PhysicsResolver.resolve_scene_physx()``, empty under shipped), and the effective count against the
    independent actors' authored counts. PhysX clamps each actor's count (default 4 when unauthored) to the scene's
    ``[minPositionIterationCount, maxPositionIterationCount]`` and runs the maximum clamped count across the actors in
    each contact island on CPU, or one maximum across all awake actors in the scene on GPU dynamics (#19, #28)."""
    from env.scene_manager.physics_spec import SOLVER_POSITION_ITERATION_FACTOR

    authored = [int(round(float(x))) if x is not None else 4 for x in authored_position_iterations]
    max_authored = max(authored) if authored else None
    rec: dict[str, Any] = {"scenes": scenes, "max_authored_position_iterations": max_authored, "expected": dict(expected)}
    failures: list[str] = []
    if expected and not scenes:
        failures.append("no physics scene discovered: the enabled solver fix cannot be read back")
    for path, sc in scenes.items():
        lo = int(round(float(sc.get("physxScene:minPositionIterationCount") or 1)))
        hi = int(round(float(sc.get("physxScene:maxPositionIterationCount") or 255)))
        eff = max(min(max(a, lo), hi) for a in authored) if authored else None
        sc["effective_position_iterations"] = eff
        sc["factor_over_authored"] = (eff / max_authored) if (eff is not None and max_authored) else None
        if not expected:
            continue
        if sc.get("physxScene:enableStabilization") is not expected["enable_stabilization"]:
            failures.append(f"{path}: enableStabilization {sc.get('physxScene:enableStabilization')!r} != {expected['enable_stabilization']!r}")
        if lo != int(expected["min_position_iteration_count"]):
            failures.append(f"{path}: minPositionIterationCount {lo} != {expected['min_position_iteration_count']}")
        if sc["factor_over_authored"] is None or abs(sc["factor_over_authored"] - SOLVER_POSITION_ITERATION_FACTOR) > 1e-9:
            failures.append(
                f"{path}: effective position iterations {eff} are {sc['factor_over_authored']} x the bodies' largest authored "
                f"count {max_authored}, not {SOLVER_POSITION_ITERATION_FACTOR} x"
            )
    return rec, failures


def solver_gate_failures(solver_scene: Any, expected: dict[str, Any]) -> list[str]:
    """Failures that invalidate a run whose ``solver`` fix is enabled (``expected`` non-empty), whatever the other
    assertions: the readback is missing (error string, absent), found no scene, or disagrees with the fix."""
    if not expected:
        return []
    if not isinstance(solver_scene, dict):
        return [f"solver readback missing or errored: {solver_scene!r}"]
    if not solver_scene.get("scenes"):
        return ["solver readback found no physics scene"]
    return list(solver_scene.get("failures") or [])


def solver_actor_position_iterations(stage: Any) -> dict[str, float | None]:
    """Authored position-iteration attribute per independent PhysX actor (``PxArticulationReducedCoordinate`` roots and
    standalone ``PxRigidDynamic`` bodies). Articulation links (descendants of an enabled ``ArticulationRootAPI`` prim or
    rigid bodies joined to one by enabled, non-excluded ``UsdPhysics.Joint``s) have no per-link count in PhysX, so the
    articulation root's ``physxArticulation:solverPositionIterationCount`` stands for the whole articulation.

    Also excludes two uninstantiated USD authoring patterns verified against PhysX ``scene_statistics.nb_articulations``:
    (1) ``ArticulationRootAPI`` or child ``RigidBodyAPI`` prims strictly beneath an enabled ancestor ``RigidBodyAPI``
        without ``!resetXformStack!`` on the path between them (rejected by ``omni.physicsschema.plugin``, e.g.
        ``Rigid/socket/00000`` ``E_body_3`` / ``E_button_6``), and
    (2) bodyless ``ArticulationRootAPI`` prims with zero dynamic rigid-body links in their subtree or joint component
        (e.g. ``Rigid/factory_nut/*`` ``/visual``).
    """
    from pxr import UsdGeom, UsdPhysics

    enabled_rb_paths: list[str] = []
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and _attr(p, "physics:rigidBodyEnabled") is not False:
            enabled_rb_paths.append(str(p.GetPath()))

    def _has_unreset_enabled_rb_ancestor(prim: Any, path: str) -> bool:
        cur = prim
        while True:
            parent = cur.GetParent()
            if not parent or not parent.IsValid() or str(parent.GetPath()) == "/":
                return False
            xf = UsdGeom.Xformable(cur)
            if xf and xf.GetResetXformStack():
                return False
            p_path = str(parent.GetPath())
            if p_path in enabled_rb_set:
                return True
            cur = parent

    enabled_rb_set = set(enabled_rb_paths)
    art_roots: dict[str, float | None] = {}
    root_rb_paths: set[str] = set()
    rigid_bodies: dict[str, float | None] = {}
    kinematic_bodies: set[str] = set()
    joints: list[tuple[str, list[str], list[str]]] = []
    for p in stage.Traverse():
        path = str(p.GetPath())
        has_rb = p.HasAPI(UsdPhysics.RigidBodyAPI)
        rb_enabled = has_rb and _attr(p, "physics:rigidBodyEnabled") is not False
        rb_kinematic = rb_enabled and _attr(p, "physics:kinematicEnabled") is True
        under_rb = _has_unreset_enabled_rb_ancestor(p, path)
        if (
            not under_rb
            and p.HasAPI(UsdPhysics.ArticulationRootAPI)
            and _attr(p, "physxArticulation:articulationEnabled") is not False
            and not (has_rb and (not rb_enabled or rb_kinematic))
        ):
            art_roots[path] = _attr(p, "physxArticulation:solverPositionIterationCount")
            if rb_enabled:
                root_rb_paths.add(path)
        elif rb_enabled and not under_rb:
            rigid_bodies[path] = _attr(p, "physxRigidBody:solverPositionIterationCount")
            if rb_kinematic:
                kinematic_bodies.add(path)
        if p.IsA(UsdPhysics.Joint) and _attr(p, "physics:jointEnabled") is not False and _attr(p, "physics:excludeFromArticulation") is not True:
            j = UsdPhysics.Joint(p)
            b0 = [str(t) for t in j.GetBody0Rel().GetTargets() if str(t)]
            b1 = [str(t) for t in j.GetBody1Rel().GetTargets() if str(t)]
            joints.append((path, b0, b1))
    root_prefixes = tuple(r + "/" for r in art_roots)
    dyn_rbs = set(rigid_bodies) - kinematic_bodies
    link_paths = {b for b in rigid_bodies if b.startswith(root_prefixes)}
    adj: dict[str, set[str]] = {}
    for jpath, b0, b1 in joints:
        if jpath in art_roots or jpath.startswith(root_prefixes):
            link_paths.update((set(b0) | set(b1)) & dyn_rbs)
        for u in b0:
            for v in b1:
                adj.setdefault(u, set()).add(v)
                adj.setdefault(v, set()).add(u)
    stack = list((link_paths & dyn_rbs) | root_rb_paths)
    seen = set(stack)
    while stack:
        u = stack.pop()
        for v in adj.get(u, ()):
            if v in dyn_rbs and v not in seen:
                seen.add(v)
                link_paths.add(v)
                stack.append(v)

    def _art_has_any_link(rpath: str) -> bool:
        if rpath in root_rb_paths:
            return True
        pref = rpath + "/"
        if any(lp.startswith(pref) for lp in link_paths):
            return True
        for jpath, b0, b1 in joints:
            if jpath == rpath or jpath.startswith(pref):
                if (set(b0) | set(b1)) & dyn_rbs:
                    return True
        return False

    out = {r: v for r, v in art_roots.items() if _art_has_any_link(r)}
    for b, v in rigid_bodies.items():
        if b not in link_paths:
            out[b] = v
    return out


def solver_scene_readback(stage: Any, resolver: Any) -> tuple[dict[str, Any], list[str]]:
    """USD readback of the physics scene and every independent actor's authored position iterations (articulation roots
    and non-link rigid bodies; articulation links are skipped), checked by ``solver_scene_check``."""
    from pxr import PhysxSchema

    keys = ("physxScene:enableStabilization", "physxScene:minPositionIterationCount", "physxScene:maxPositionIterationCount")
    scenes = {str(p.GetPath()): {k: _attr(p, k) for k in keys} for p in stage.Traverse() if p.HasAPI(PhysxSchema.PhysxSceneAPI)}
    authored = list(solver_actor_position_iterations(stage).values())
    expected = resolver.resolve_scene_physx() if resolver is not None else {}
    return solver_scene_check(scenes, authored, expected)


def _resolve_mat_path_values(stage: Any, mat_path: str) -> dict[str, Any]:
    """Resolve a PhysX contact-report material SDF path to its USD numeric MaterialAPI values."""
    from pxr import UsdPhysics

    if not mat_path or mat_path == "/physicsScene/defaultMaterial" or stage is None:
        return {
            "path": mat_path,
            "is_default_material": mat_path == "/physicsScene/defaultMaterial",
            "static_friction": None,
            "dynamic_friction": None,
            "restitution": None,
            "friction_combine_mode": None,
            "restitution_combine_mode": None,
        }
    mp = stage.GetPrimAtPath(mat_path)
    if mp and mp.IsValid() and mp.HasAPI(UsdPhysics.MaterialAPI):
        return {
            "path": mat_path,
            "is_default_material": False,
            "static_friction": _attr(mp, "physics:staticFriction"),
            "dynamic_friction": _attr(mp, "physics:dynamicFriction"),
            "restitution": _attr(mp, "physics:restitution"),
            "friction_combine_mode": _attr(mp, "physxMaterial:frictionCombineMode"),
            "restitution_combine_mode": _attr(mp, "physxMaterial:restitutionCombineMode"),
        }
    return {
        "path": mat_path,
        "is_default_material": False,
        "static_friction": None,
        "dynamic_friction": None,
        "restitution": None,
        "friction_combine_mode": None,
        "restitution_combine_mode": None,
    }


def _resolve_usd_physics_mat(prim: Any) -> dict[str, Any]:
    from pxr import UsdPhysics, UsdShade

    api = UsdShade.MaterialBindingAPI(prim)
    m, _ = api.ComputeBoundMaterial("physics")
    if m and m.GetPrim().IsValid() and m.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
        mp = m.GetPrim()
        return {
            "purpose": "physics",
            "path": str(mp.GetPath()),
            "static_friction": _attr(mp, "physics:staticFriction"),
            "dynamic_friction": _attr(mp, "physics:dynamicFriction"),
            "restitution": _attr(mp, "physics:restitution"),
            "friction_combine_mode": _attr(mp, "physxMaterial:frictionCombineMode"),
            "restitution_combine_mode": _attr(mp, "physxMaterial:restitutionCombineMode"),
        }
    m2, _ = api.ComputeBoundMaterial()
    if m2 and m2.GetPrim().IsValid() and m2.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
        mp = m2.GetPrim()
        return {
            "purpose": "all",
            "path": str(mp.GetPath()),
            "static_friction": _attr(mp, "physics:staticFriction"),
            "dynamic_friction": _attr(mp, "physics:dynamicFriction"),
            "restitution": _attr(mp, "physics:restitution"),
            "friction_combine_mode": _attr(mp, "physxMaterial:frictionCombineMode"),
            "restitution_combine_mode": _attr(mp, "physxMaterial:restitutionCombineMode"),
        }
    return {
        "purpose": "scene-default",
        "path": "/physicsScene/defaultMaterial",
        "all_purpose_bound": str(m2.GetPrim().GetPath()) if (m2 and m2.GetPrim().IsValid()) else None,
    }


def _iter_colliders(root_prim: Any) -> list[Any]:
    from pxr import Usd, UsdPhysics

    if not root_prim or not root_prim.IsValid():
        return []
    return [
        p
        for p in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
        if p.HasAPI(UsdPhysics.CollisionAPI)
    ]


def _iter_enabled_gprim_colliders(root_prim: Any) -> list[Any]:
    from pxr import UsdGeom

    return [
        c
        for c in _iter_colliders(root_prim)
        if bool(UsdGeom.Gprim(c)) and _attr(c, "physics:collisionEnabled") is not False
    ]


def compute_collider_aabb_extents(root_prim: Any, scale: tuple[float, float, float] = (1.0, 1.0, 1.0)) -> list[float] | None:
    """Compute exact local-frame collider AABB extents (scaled by ``scale``) across enabled ``UsdGeom.Gprim`` colliders under ``root_prim``.

    For ``UsdGeom.Mesh`` colliders, transforms actual mesh vertices into ``root_prim``'s frame so rotated child meshes
    (e.g. 45-deg ``camera_base`` and 43.7-deg ``rsd455`` under ``camera_stand``) are not inflated by ``Gf.BBox3d``
    8-corner OBB-to-AABB projection; falls back to ``BBoxCache.ComputeRelativeBound`` on analytic primitive Gprims.
    """
    from pxr import Usd, UsdGeom

    cols = _iter_enabled_gprim_colliders(root_prim)
    if not cols:
        return None
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    root_inv = xcache.GetLocalToWorldTransform(root_prim).GetInverse()
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide, UsdGeom.Tokens.render],
        useExtentsHint=False,
    )
    mins = np.array([np.inf, np.inf, np.inf], dtype=float)
    maxs = np.array([-np.inf, -np.inf, -np.inf], dtype=float)
    found = False
    for c in cols:
        mesh = UsdGeom.Mesh(c)
        pts_raw = mesh.GetPointsAttr().Get() if mesh else None
        if pts_raw is not None and len(pts_raw) > 0:
            pts = np.asarray(pts_raw, dtype=float)
            mat4 = np.asarray(xcache.GetLocalToWorldTransform(c) * root_inv, dtype=float)
            pts_root = (np.hstack([pts, np.ones((len(pts), 1), dtype=float)]) @ mat4)[:, :3]
            mins = np.minimum(mins, pts_root.min(axis=0))
            maxs = np.maximum(maxs, pts_root.max(axis=0))
            found = True
            continue
        rel_rng = cache.ComputeRelativeBound(c, root_prim).ComputeAlignedRange()
        if rel_rng.IsEmpty():
            continue
        lo, hi = rel_rng.GetMin(), rel_rng.GetMax()
        mins = np.minimum(mins, np.array([float(lo[0]), float(lo[1]), float(lo[2])], dtype=float))
        maxs = np.maximum(maxs, np.array([float(hi[0]), float(hi[1]), float(hi[2])], dtype=float))
        found = True
    if not found:
        return None
    ext = (maxs - mins) * np.asarray(scale, dtype=float)
    return [round(float(v), 6) for v in ext]


def compute_engine_collider_aabb_extents(
    root_prim: Any,
    *,
    world_pos: Any = None,
    world_ori_wxyz: Any = None,
) -> dict[str, Any] | None:
    """Measure PhysX's live cooked collider AABB extents in ``root_prim``'s oriented frame via 6-sided ``overlap_box`` bisection."""
    import carb
    from omni.physx import get_physx_scene_query_interface
    from pxr import Usd, UsdGeom

    if not root_prim or not root_prim.IsValid():
        return None
    root_path = str(root_prim.GetPath())
    if world_pos is not None and world_ori_wxyz is not None:
        p_root = np.asarray(world_pos.cpu() if hasattr(world_pos, "cpu") else world_pos, dtype=float).reshape(-1)[:3]
        q_arr = np.asarray(world_ori_wxyz.cpu() if hasattr(world_ori_wxyz, "cpu") else world_ori_wxyz, dtype=float).reshape(-1)[:4]
        q_norm = float(np.linalg.norm(q_arr))
        if q_norm < 1e-12:
            return None
        w, x, y, z = q_arr / q_norm
    else:
        xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
        m_world = xcache.GetLocalToWorldTransform(root_prim)
        p_root = np.array(m_world.ExtractTranslation(), dtype=float)
        q_gf = m_world.RemoveScaleShear().ExtractRotationQuat()
        im = q_gf.GetImaginary()
        w, x, y, z = float(q_gf.GetReal()), float(im[0]), float(im[1]), float(im[2])

    quat_xyzw = carb.Float4(float(x), float(y), float(z), float(w))
    rot_cols = (
        np.array([1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y)], dtype=float),
        np.array([2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + w * x)], dtype=float),
        np.array([2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)], dtype=float),
    )

    sq = get_physx_scene_query_interface()
    half_slab = 1.5
    trans = 1.5
    hit_colliders: set[str] = set()

    def _box_overlaps_target(he_xyz: tuple[float, float, float], center_xyz: np.ndarray) -> bool:
        matched = [False]

        def _cb(hit: Any) -> bool:
            cpath = str(hit.collision)
            if cpath == root_path or cpath.startswith(root_path + "/"):
                matched[0] = True
                hit_colliders.add(cpath)
                return False
            return True

        sq.overlap_box(
            carb.Float3(float(he_xyz[0]), float(he_xyz[1]), float(he_xyz[2])),
            carb.Float3(float(center_xyz[0]), float(center_xyz[1]), float(center_xyz[2])),
            quat_xyzw,
            _cb,
            False,
        )
        return matched[0]

    # Collect all colliders under root_path first and verify at least one PhysX shape exists.
    all_check: list[str] = []

    def _cb_all(hit: Any) -> bool:
        cpath = str(hit.collision)
        if cpath == root_path or cpath.startswith(root_path + "/"):
            all_check.append(cpath)
            hit_colliders.add(cpath)
        return True

    sq.overlap_box(
        carb.Float3(trans, trans, trans),
        carb.Float3(float(p_root[0]), float(p_root[1]), float(p_root[2])),
        quat_xyzw,
        _cb_all,
        False,
    )
    if not all_check:
        return None

    extents: list[float] = []
    for k in range(3):
        he = (
            half_slab if k == 0 else trans,
            half_slab if k == 1 else trans,
            half_slab if k == 2 else trans,
        )
        axis_w = rot_cols[k]

        # Supremum x_max: slab [s, s + 2*half_slab] centered at p_root + (s + half_slab)*axis_w
        # 28 bisection steps over [-1.5, 1.5] m -> 1.1e-8 m bracket (well below 1e-6 m rounding).
        lo_hi, hi_hi = -trans, trans
        for _ in range(28):
            mid = 0.5 * (lo_hi + hi_hi)
            if _box_overlaps_target(he, p_root + (mid + half_slab) * axis_w):
                lo_hi = mid
            else:
                hi_hi = mid
        x_max = 0.5 * (lo_hi + hi_hi)

        # Infimum x_min: slab [s - 2*half_slab, s] centered at p_root + (s - half_slab)*axis_w
        lo_lo, hi_lo = -trans, trans
        for _ in range(28):
            mid = 0.5 * (lo_lo + hi_lo)
            if _box_overlaps_target(he, p_root + (mid - half_slab) * axis_w):
                hi_lo = mid
            else:
                lo_lo = mid
        x_min = 0.5 * (lo_lo + hi_lo)
        extents.append(round(float(x_max - x_min), 6))

    return {
        "extents_m": extents,
        "hit_colliders": sorted(hit_colliders),
        "hit_materials": None,
    }


def _collect_contacts_for_prefixes(prefixes: list[str], stage: Any = None) -> list[dict[str, Any]]:
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools

    headers, data = get_physx_simulation_interface().get_contact_report()
    out: list[dict[str, Any]] = []
    for h in headers:
        a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
        a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        c0 = str(PhysicsSchemaTools.intToSdfPath(h.collider0))
        c1 = str(PhysicsSchemaTools.intToSdfPath(h.collider1))
        if not any(
            a0.startswith(p) or a1.startswith(p) or c0.startswith(p) or c1.startswith(p)
            for p in prefixes
        ):
            continue
        mats = sorted(
            {
                (
                    str(PhysicsSchemaTools.intToSdfPath(d.material0)),
                    str(PhysicsSchemaTools.intToSdfPath(d.material1)),
                )
                for d in data[h.contact_data_offset : h.contact_data_offset + h.num_contact_data]
            }
        )
        out.append(
            {
                "actor0": a0,
                "actor1": a1,
                "collider0": c0,
                "collider1": c1,
                "num_contacts": int(h.num_contact_data),
                "materials": [list(m) for m in mats],
                "material_values": [
                    [_resolve_mat_path_values(stage, m0), _resolve_mat_path_values(stage, m1)]
                    for (m0, m1) in mats
                ],
            }
        )
    return sorted(out, key=lambda r: (r["collider0"], r["collider1"]))


def _get_or_create_probe_sphere(stage: Any, prim_path: str, mat_spec: Any, radius: float = 0.006) -> Any:
    """Create or re-enable a small 6 mm dynamic sphere with contact reporting for clean canary touches."""
    from isaacsim.core.prims import SingleRigidPrim
    from pxr import PhysxSchema, UsdGeom, UsdPhysics

    from env.scene_manager.physics_spec import author_usd_mass, author_usd_material, bind_usd_physics_material

    existing = stage.GetPrimAtPath(prim_path)
    if not existing or not existing.IsValid():
        sph = UsdGeom.Sphere.Define(stage, prim_path)
        sph.CreateRadiusAttr().Set(float(radius))
        prim = sph.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr().Set(True)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
        mat = author_usd_material(stage, f"{prim_path}/physics_material", mat_spec)
        bind_usd_physics_material(prim, mat)
        author_usd_mass(prim, 0.02)
        srp = SingleRigidPrim(prim_path=prim_path, name=prim_path.split("/")[-1], translation=(100000.0, 100000.0, 100000.0))
        srp.initialize()
        return srp

    UsdPhysics.CollisionAPI(existing).CreateCollisionEnabledAttr().Set(True)
    mat = author_usd_material(stage, f"{prim_path}/physics_material", mat_spec)
    bind_usd_physics_material(existing, mat)
    srp = SingleRigidPrim(prim_path=prim_path, name=prim_path.split("/")[-1])
    srp.initialize()
    return srp


def _get_or_create_slide_box(stage: Any, prim_path: str, mat_spec: Any, size_xyz: tuple[float, float, float] = (0.04, 0.04, 0.02)) -> Any:
    """Create or re-enable a flat 4x4x2 cm cube rigid body for non-rolling Coulomb slide checks on Table and Ground."""
    from isaacsim.core.prims import SingleRigidPrim
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    from env.scene_manager.physics_spec import author_usd_mass, author_usd_material, bind_usd_physics_material

    existing = stage.GetPrimAtPath(prim_path)
    if not existing or not existing.IsValid():
        cube = UsdGeom.Cube.Define(stage, prim_path)
        cube.CreateSizeAttr().Set(1.0)
        prim = cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr().Set(True)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
        mat = author_usd_material(stage, f"{prim_path}/physics_material", mat_spec)
        bind_usd_physics_material(prim, mat)
        author_usd_mass(prim, 0.08)
        srp = SingleRigidPrim(
            prim_path=prim_path,
            name=prim_path.split("/")[-1],
            translation=(100000.0, 100000.0, 100200.0),
            scale=list(size_xyz),
        )
        srp.initialize()
        return srp

    UsdPhysics.CollisionAPI(existing).CreateCollisionEnabledAttr().Set(True)
    mat = author_usd_material(stage, f"{prim_path}/physics_material", mat_spec)
    bind_usd_physics_material(existing, mat)
    srp = SingleRigidPrim(prim_path=prim_path, name=prim_path.split("/")[-1])
    srp.initialize()
    return srp


def _write_procedural_test_assets(tmpdir: str) -> tuple[str, str]:
    """Write minimal 2-link Articulation USD and belt Dynamic USD in ``tmpdir`` (no downloads needed)."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    art_usd = os.path.join(tmpdir, "sentinel_hinge.usda")
    st_a = Usd.Stage.CreateNew(art_usd)
    UsdGeom.SetStageUpAxis(st_a, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st_a, 1.0)
    root_a = UsdGeom.Xform.Define(st_a, "/Root")
    st_a.SetDefaultPrim(root_a.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root_a.GetPrim())

    base = UsdGeom.Xform.Define(st_a, "/Root/base_link")
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
    base_col = UsdGeom.Cube.Define(st_a, "/Root/base_link/geom")
    base_col.CreateSizeAttr().Set(1.0)
    base_col.AddScaleOp().Set(Gf.Vec3f(0.12, 0.12, 0.04))
    UsdPhysics.CollisionAPI.Apply(base_col.GetPrim())

    lid = UsdGeom.Xform.Define(st_a, "/Root/lid_link")
    lid.AddTranslateOp().Set(Gf.Vec3d(0.14, 0.0, 0.0))
    UsdPhysics.RigidBodyAPI.Apply(lid.GetPrim())
    lid_col = UsdGeom.Cube.Define(st_a, "/Root/lid_link/geom")
    lid_col.CreateSizeAttr().Set(1.0)
    lid_col.AddScaleOp().Set(Gf.Vec3f(0.12, 0.12, 0.04))
    UsdPhysics.CollisionAPI.Apply(lid_col.GetPrim())

    jnt = UsdPhysics.RevoluteJoint.Define(st_a, "/Root/hinge_joint")
    jnt.CreateAxisAttr().Set("Y")
    jnt.CreateBody0Rel().SetTargets([Sdf.Path("/Root/base_link")])
    jnt.CreateBody1Rel().SetTargets([Sdf.Path("/Root/lid_link")])
    jnt.CreateLocalPos0Attr().Set(Gf.Vec3f(0.07, 0.0, 0.0))
    jnt.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.07, 0.0, 0.0))
    jnt.CreateLowerLimitAttr().Set(-90.0)
    jnt.CreateUpperLimitAttr().Set(90.0)
    UsdPhysics.DriveAPI.Apply(jnt.GetPrim(), "angular")
    st_a.GetRootLayer().Save()

    dyn_usd = os.path.join(tmpdir, "sentinel_belt.usda")
    st_d = Usd.Stage.CreateNew(dyn_usd)
    UsdGeom.SetStageUpAxis(st_d, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st_d, 1.0)
    root_d = UsdGeom.Xform.Define(st_d, "/Belt")
    st_d.SetDefaultPrim(root_d.GetPrim())
    belt_col = UsdGeom.Cube.Define(st_d, "/Belt/surface")
    belt_col.CreateSizeAttr().Set(1.0)
    belt_col.AddScaleOp().Set(Gf.Vec3f(0.30, 0.20, 0.04))
    UsdPhysics.CollisionAPI.Apply(belt_col.GetPrim())
    st_d.GetRootLayer().Save()

    return art_usd, dyn_usd


def run_canary_touches(
    env: Any,
    *,
    include_procedural_classes: bool = False,
    tmpdir: str | None = None,
) -> dict[str, Any]:
    """Touch a dedicated 6 mm probe sphere against static fixtures, geometry receptacles, and robot fingers
    across all envs (without moving any scene object), run 100-step Coulomb slides on Table and Ground in env_0,
    and optionally test procedural ``ArticulationObject`` and ``DynamicObject`` instances.
    """
    from isaacsim.core.simulation_manager import SimulationManager
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics

    from env.scene_manager.objects.articulation import ArticulationObject
    from env.scene_manager.objects.dynamic import DynamicObject
    from env.scene_manager.physics_spec import MaterialSpec, combine_values

    stage = omni.usd.get_context().get_stage()
    lm = env.scene_manager.layout_manager
    resolver = getattr(env, "physics_resolver", None)

    if resolver is not None and resolver.params_table is not None and "sentinel_rigid" in resolver.params_table.class_materials:
        probe_cls: str | None = "sentinel_rigid"
        probe_mat = resolver.params_table.class_materials["sentinel_rigid"]
    elif resolver is not None and resolver.params_table is not None and "pet_plastic" in resolver.params_table.class_materials:
        probe_cls = "pet_plastic"
        probe_mat = resolver.params_table.class_materials["pet_plastic"]
    elif resolver is not None and resolver.params_table is not None and resolver.params_table.class_materials:
        probe_cls, probe_mat = next(iter(resolver.params_table.class_materials.items()))
    else:
        probe_cls = None
        probe_mat = MaterialSpec(0.5, 0.5, 0.0)

    probe_radius = 0.006
    probe_path = "/World/canary_probe_sphere"
    probe = _get_or_create_probe_sphere(stage, probe_path, probe_mat, radius=probe_radius)
    slide_box_path = "/World/canary_slide_box"
    slide_box_prim = _get_or_create_slide_box(stage, slide_box_path, probe_mat, size_xyz=(0.04, 0.04, 0.02))
    env.sim_step(render=False)

    def _park_probe(p_wrapper: Any, z_off: float = 0.0) -> None:
        p_wrapper.set_world_pose(
            position=torch.tensor([100000.0, 100000.0, 100000.0 + z_off], dtype=torch.float32),
            orientation=torch.tensor([1.0, 0.0, 0.0, 0.0], dtype=torch.float32),
        )
        p_wrapper.set_linear_velocity(torch.zeros(3, dtype=torch.float32))
        p_wrapper.set_angular_velocity(torch.zeros(3, dtype=torch.float32))

    _park_probe(probe, 0.0)
    _park_probe(slide_box_prim, 200.0)
    env.sim_step(render=False)

    def _touch_and_read(
        world_xyz: tuple[float, float, float],
        vel_xyz: tuple[float, float, float] = (0.0, 0.0, -0.8),
        steps: int = 8,
    ) -> list[dict[str, Any]]:
        collected: dict[tuple[str, str], dict[str, Any]] = {}
        probe.set_world_pose(
            position=torch.tensor(world_xyz, dtype=torch.float32),
            orientation=torch.tensor([1.0, 0.0, 0.0, 0.0], dtype=torch.float32),
        )
        probe.set_linear_velocity(torch.tensor(vel_xyz, dtype=torch.float32))
        probe.set_angular_velocity(torch.zeros(3, dtype=torch.float32))
        for _ in range(steps):
            env.sim_step(render=False)
            for rec in _collect_contacts_for_prefixes([probe_path], stage=stage):
                k = (rec["collider0"], rec["collider1"])
                prev = collected.get(k)
                if prev is None or (rec["materials"] and not prev["materials"]) or rec["num_contacts"] > prev["num_contacts"]:
                    collected[k] = rec
        _park_probe(probe, 0.0)
        env.sim_step(render=False)
        return list(collected.values())

    def _run_surface_slide(start_xy: tuple[float, float], surface_top_z: float, v0: float = 0.5, steps: int = 100) -> dict[str, Any]:
        half_z = 0.01
        slide_box_prim.set_world_pose(
            position=torch.tensor([start_xy[0], start_xy[1], surface_top_z + half_z + 0.0005], dtype=torch.float32),
            orientation=torch.tensor([1.0, 0.0, 0.0, 0.0], dtype=torch.float32),
        )
        slide_box_prim.set_linear_velocity(torch.zeros(3, dtype=torch.float32))
        slide_box_prim.set_angular_velocity(torch.zeros(3, dtype=torch.float32))
        for _ in range(15):
            env.sim_step(render=False)
        p0 = np.asarray(slide_box_prim.get_world_pose()[0], dtype=float).reshape(-1)[:3]
        slide_box_prim.set_linear_velocity(torch.tensor([v0, 0.0, 0.0], dtype=torch.float32))
        slide_box_prim.set_angular_velocity(torch.zeros(3, dtype=torch.float32))
        slide_contacts: dict[tuple[str, str], dict[str, Any]] = {}
        for _ in range(steps):
            env.sim_step(render=False)
            for rec in _collect_contacts_for_prefixes([slide_box_path], stage=stage):
                slide_contacts[(rec["collider0"], rec["collider1"])] = rec
        p1 = np.asarray(slide_box_prim.get_world_pose()[0], dtype=float).reshape(-1)[:3]
        _park_probe(slide_box_prim, 200.0)
        env.sim_step(render=False)
        return {
            "v0_m_s": v0,
            "steps": steps,
            "slide_x_mm": round(float(p1[0] - p0[0]) * 1000.0, 2),
            "lateral_y_mm": round(float(p1[1] - p0[1]) * 1000.0, 2),
            "vertical_z_mm": round(float(p1[2] - p0[2]) * 1000.0, 2),
            "contacts": list(slide_contacts.values()),
        }

    bc = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide, UsdGeom.Tokens.render],
        useExtentsHint=False,
    )
    results: dict[str, Any] = {"per_env": {}, "geometry": {}, "fixture_slides_env0": {}, "procedural_classes": {}}
    origins = np.asarray(env.sim.scene.env_origins.cpu(), dtype=float)

    for env_i in range(env.num_envs):
        ox, oy, oz = origins[env_i]
        env_root = f"/World/envs/env_{env_i}"
        env_rec: dict[str, Any] = {}

        # 1. Table: measure world AABB of Table prim and drop 6 mm sphere 4 mm above top face on clear y=+0.35 strip
        tprim = stage.GetPrimAtPath(f"{env_root}/Table")
        tz_top = float(oz + 0.765)
        tx_mid = float(ox)
        ty_clear = float(oy + 0.35)
        tb_min_x, tb_max_x = float(ox - 0.70), float(ox + 0.70)
        tb_min_y, tb_max_y = float(oy - 0.60), float(oy + 0.50)
        if tprim and tprim.IsValid():
            tb = bc.ComputeWorldBound(tprim).ComputeAlignedRange()
            if not tb.IsEmpty():
                tz_top = float(tb.GetMax()[2])
                tb_min_x, tb_max_x = float(tb.GetMin()[0]), float(tb.GetMax()[0])
                tb_min_y, tb_max_y = float(tb.GetMin()[1]), float(tb.GetMax()[1])
                tx_mid = float(0.5 * (tb_min_x + tb_max_x))
                ty_clear = float(tb_max_y - 0.15)
        table_prefix = f"{env_root}/Table"

        def _non_table_xy_boxes() -> list[tuple[float, float, float, float]]:
            env_p = stage.GetPrimAtPath(env_root)
            boxes_xy: list[tuple[float, float, float, float]] = []
            if not env_p or not env_p.IsValid():
                return boxes_xy
            bc_live = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide, UsdGeom.Tokens.render],
                useExtentsHint=False,
            )
            for cp in _iter_enabled_gprim_colliders(env_p):
                cp_s = str(cp.GetPath())
                if cp_s.startswith(table_prefix) or cp_s.startswith(f"{env_root}/Ground") or cp_s.startswith(f"{env_root}/Rooms"):
                    continue
                if cp_s.startswith(probe_path) or cp_s.startswith(slide_box_path):
                    continue
                wr = bc_live.ComputeWorldBound(cp).ComputeAlignedRange()
                if wr.IsEmpty():
                    continue
                wmin, wmax = wr.GetMin(), wr.GetMax()
                if float(wmax[2]) > tz_top - 0.002 and float(wmin[2]) < tz_top + 0.08:
                    boxes_xy.append((float(wmin[0]), float(wmax[0]), float(wmin[1]), float(wmax[1])))
            return boxes_xy

        env_rec["Table"] = _touch_and_read((tx_mid, ty_clear, tz_top + probe_radius + 0.004), (0.0, 0.0, -0.8), steps=8)
        if not any(
            (c["collider1"] if c["collider0"] == probe_path else c["collider0"]).startswith(table_prefix)
            for c in env_rec["Table"]
        ):
            # Fallback when fixtures or clutter occupy (tx_mid, ty_clear): test corners first, then BBoxCache-clear grid points.
            occ_xy = _non_table_xy_boxes()
            touch_cands: list[tuple[float, float]] = [
                (float(tb_max_x - 0.14), float(tb_max_y - 0.08)),
                (float(tb_min_x + 0.14), float(tb_max_y - 0.08)),
            ]
            for gy_f in np.linspace(tb_max_y - 0.05, tb_min_y + 0.08, 12):
                for gx_f in np.linspace(tb_min_x + 0.06, tb_max_x - 0.06, 15):
                    if not any(
                        x0 - 0.015 <= gx_f <= x1 + 0.015 and y0 - 0.015 <= gy_f <= y1 + 0.015
                        for x0, x1, y0, y1 in occ_xy
                    ):
                        touch_cands.append((float(gx_f), float(gy_f)))
            for fx, fy in touch_cands:
                cand_t = _touch_and_read((fx, fy, tz_top + probe_radius + 0.004), (0.0, 0.0, -0.8), steps=8)
                if any(
                    (c["collider1"] if c["collider0"] == probe_path else c["collider0"]).startswith(table_prefix)
                    for c in cand_t
                ):
                    env_rec["Table"] = cand_t
                    break

        # 2. Ground: outside Simple_Room_nolight (|x|<=2.31, y in [-1.75, 2.48] at scale 0.5), inside env_spacing [-5, 5]
        gprim_fix = stage.GetPrimAtPath(f"{env_root}/Ground")
        gz_top = float(oz)
        gx_clear, gy_clear = float(ox + 3.5), float(oy + 3.5)
        if gprim_fix and gprim_fix.IsValid():
            gb_fix = bc.ComputeWorldBound(gprim_fix).ComputeAlignedRange()
            if not gb_fix.IsEmpty():
                gz_top = float(gb_fix.GetMax()[2])
                gx_clear = float(min(gb_fix.GetMax()[0] - 0.5, ox + 3.5))
                gy_clear = float(min(gb_fix.GetMax()[1] - 0.5, oy + 3.5))
        env_rec["Ground"] = _touch_and_read((gx_clear, gy_clear, gz_top + probe_radius + 0.004), (0.0, 0.0, -0.8), steps=8)

        # 3. Room floor (rooms live at {env_root}/Rooms/{cat}/{inst}; floor top is at oz + 0.0152 at scale 0.5, inside Room away from Table)
        r_obj_canary = env.scene_manager._rooms[env_i]
        r_prim_path = getattr(r_obj_canary, "prim_path", None) if r_obj_canary is not None else None
        room_prim = stage.GetPrimAtPath(r_prim_path) if r_prim_path else None
        if room_prim and room_prim.IsValid():
            rcols = _iter_enabled_gprim_colliders(room_prim)
            if rcols:
                target_rx, target_ry = float(ox + 1.2), float(oy + 1.2)
                rz_top: float | None = None
                for rc in rcols:
                    rbox = bc.ComputeWorldBound(rc).ComputeAlignedRange()
                    if rbox.IsEmpty():
                        continue
                    rmin, rmax = rbox.GetMin(), rbox.GetMax()
                    if rmin[0] <= target_rx <= rmax[0] and rmin[1] <= target_ry <= rmax[1]:
                        if rz_top is None or float(rmax[2]) < rz_top:
                            rz_top = float(rmax[2])
                if rz_top is None:
                    rbox0 = bc.ComputeWorldBound(rcols[0]).ComputeAlignedRange()
                    if not rbox0.IsEmpty():
                        rz_top = float(rbox0.GetMax()[2])
                if rz_top is not None:
                    env_rec["Room"] = _touch_and_read((target_rx, target_ry, rz_top + probe_radius + 0.004), (0.0, 0.0, -0.8), steps=8)
            else:
                env_rec["Room"] = []

        # 4. Robot fingers: query live world bounds of link7 and link8 colliders in env_i and approach each finger's
        # outer face near its lower fingertip along the inter-finger axis u_out (strictly opposite the other finger and below link6).
        # Note: in ARX_physics.usd, CollisionAPI sits on .../link{7,8}/collisions/link{7,8} (an Xform wrapping the STL Mesh).
        finger_contacts: dict[str, Any] = {}
        for ri in range(len(env.robot_manager.robot_key)):
            l7_lp = stage.GetPrimAtPath(f"{env_root}/robot{ri}/link7")
            l8_lp = stage.GetPrimAtPath(f"{env_root}/robot{ri}/link8")
            l7_cols = [c for c in _iter_colliders(l7_lp) if _attr(c, "physics:collisionEnabled") is not False]
            l8_cols = [c for c in _iter_colliders(l8_lp) if _attr(c, "physics:collisionEnabled") is not False]
            if not l7_cols or not l8_cols:
                finger_contacts[f"robot{ri}/link7"] = []
                finger_contacts[f"robot{ri}/link8"] = []
                continue
            b7 = bc.ComputeWorldBound(l7_cols[0]).ComputeAlignedRange()
            b8 = bc.ComputeWorldBound(l8_cols[0]).ComputeAlignedRange()
            if b7.IsEmpty() or b8.IsEmpty():
                finger_contacts[f"robot{ri}/link7"] = []
                finger_contacts[f"robot{ri}/link8"] = []
                continue
            b7_min, b7_max = np.array(b7.GetMin(), dtype=float), np.array(b7.GetMax(), dtype=float)
            b8_min, b8_max = np.array(b8.GetMin(), dtype=float), np.array(b8.GetMax(), dtype=float)
            c7 = 0.5 * (b7_min + b7_max)
            c8 = 0.5 * (b8_min + b8_max)
            for link, b_min, b_max, c_self, c_other in (
                ("link7", b7_min, b7_max, c7, c8),
                ("link8", b8_min, b8_max, c8, c7),
            ):
                diff_xy = np.array([c_self[0] - c_other[0], c_self[1] - c_other[1], 0.0], dtype=float)
                norm_xy = float(np.linalg.norm(diff_xy))
                u_out = diff_xy / norm_xy if norm_xy > 1e-6 else np.array([0.0, 1.0 if link == "link7" else -1.0, 0.0], dtype=float)
                half_ext = 0.5 * (b_max - b_min)
                span_out = float(np.sum(np.abs(u_out) * half_ext))
                # Target mid-height of finger collider from 2 mm outside the outer face
                target_pt = np.array([c_self[0], c_self[1], b_min[2] + 0.50 * (b_max[2] - b_min[2])], dtype=float)
                start_p = target_pt + (span_out + probe_radius + 0.002) * u_out
                v_in = -1.2 * u_out
                finger_contacts[f"robot{ri}/{link}"] = _touch_and_read(
                    (float(start_p[0]), float(start_p[1]), float(start_p[2])),
                    (float(v_in[0]), float(v_in[1]), float(v_in[2])),
                    steps=16,
                )
        env_rec["fingers"] = finger_contacts
        results["per_env"][f"env_{env_i}"] = env_rec

        if env_i == 0:
            def _slide_only_touches_table(s_rec: dict[str, Any]) -> bool:
                return (
                    bool(s_rec["contacts"])
                    and abs(float(s_rec.get("vertical_z_mm", 0.0))) <= 1.0
                    and all(
                        (c["collider1"] if c["collider0"] == slide_box_path else c["collider0"]).startswith(table_prefix)
                        for c in s_rec["contacts"]
                    )
                )

            t_slide = _run_surface_slide((tx_mid - 0.04, ty_clear), tz_top, v0=0.5, steps=100)
            if not _slide_only_touches_table(t_slide):
                # Fallback to unobstructed tabletop strips when fixtures or clutter block y=ty_clear.
                occ_slide_xy = _non_table_xy_boxes()
                slide_cands: list[tuple[float, float]] = [
                    (float(tb_max_x - 0.18), float(tb_max_y - 0.08)),
                    (float(tb_min_x + 0.12), float(tb_max_y - 0.08)),
                ]
                for gy_s in np.linspace(tb_max_y - 0.05, tb_min_y + 0.08, 12):
                    for gx_s in np.linspace(tb_min_x + 0.05, tb_max_x - 0.14, 14):
                        if not any(
                            x0 - 0.025 <= gx_s + 0.095 and x1 + 0.025 >= gx_s - 0.025
                            and y0 - 0.025 <= gy_s <= y1 + 0.025
                            for x0, x1, y0, y1 in occ_slide_xy
                        ):
                            slide_cands.append((float(gx_s), float(gy_s)))
                for sfx, sfy in slide_cands:
                    cand_s = _run_surface_slide((sfx, sfy), tz_top, v0=0.5, steps=100)
                    if _slide_only_touches_table(cand_s):
                        t_slide = cand_s
                        break
            g_slide = _run_surface_slide((gx_clear - 0.04, gy_clear), gz_top, v0=0.5, steps=100)
            t_obj = env.scene_manager._tables[0]
            g_obj = env.scene_manager._grounds[0][0] if env.scene_manager._grounds[0] else None
            for sname, srec, fobj in (("Table", t_slide, t_obj), ("Ground", g_slide, g_obj)):
                fspec = getattr(fobj, "physics_spec", None) if fobj is not None else None
                if fspec is not None and fspec.material.friction_combine_mode is not None:
                    eff_k, eff_mode = combine_values(
                        probe_mat.dynamic_friction,
                        fspec.material.dynamic_friction,
                        probe_mat.friction_combine_mode,
                        fspec.material.friction_combine_mode,
                    )
                    srec["expected_eff_mu_kinetic"] = round(float(eff_k), 5)
                    srec["expected_combine_mode"] = eff_mode
                    srec["coulomb_predicted_mm"] = round(0.5 * 0.5 / (2.0 * max(eff_k, 1e-6) * 9.81) * 1000.0, 2)
                    if (
                        probe_cls is not None
                        and resolver is not None
                        and resolver.params_table is not None
                        and fspec.asset_id in resolver.params_table.assets
                    ):
                        surf_cls = resolver.params_table.assets[fspec.asset_id].get("material_class")
                        if surf_cls:
                            pk_slide = f"{min(probe_cls, str(surf_cls))}__{max(probe_cls, str(surf_cls))}"
                            rvn_cp = (resolver.params_table.realised_vs_nominal or {}).get("class_pairs") or {}
                            if pk_slide in rvn_cp:
                                srec["pair_key"] = pk_slide
                                srec["nominal_mu_kinetic"] = rvn_cp[pk_slide]["mu_kinetic"]["nominal"]
                                srec["realised_mu_kinetic"] = rvn_cp[pk_slide]["mu_kinetic"]["realised"]
            results["fixture_slides_env0"] = {"Table": t_slide, "Ground": g_slide}

    # 5. Geometry objects in env_0 (e.g. dustbin, camera_stand)
    for label in lm.get_labels_by_prefix(0, ""):
        inst = lm.get_instance_name(0, label)
        if lm.instance_type_by_env[0].get(inst) != "geometry":
            continue
        obj = lm.get_scene_object(0, inst)
        if obj is None:
            continue
        gprim = stage.GetPrimAtPath(obj.prim_path)
        gcols = _iter_enabled_gprim_colliders(gprim)
        for gc in gcols:
            gb = bc.ComputeWorldBound(gc).ComputeAlignedRange()
            if gb.IsEmpty():
                continue
            gmin, gmax = np.array(gb.GetMin(), dtype=float), np.array(gb.GetMax(), dtype=float)
            gx = float(0.5 * (gmin[0] + gmax[0]))
            gy = float(0.5 * (gmin[1] + gmax[1]))
            if "dustbin" in label:
                # Approach outer -x wall of open-top dustbin at mid-height so inner-cavity geometry never matters.
                gz_mid = float(0.5 * (gmin[2] + gmax[2]))
                probe_rays = [((float(gmin[0] - probe_radius - 0.004), gy, gz_mid), (0.8, 0.0, 0.0), 14)]
            else:
                gz_mid = float(0.5 * (gmin[2] + gmax[2]))
                z_above = float(gmax[2] + probe_radius + 0.004)
                dx_in = min(0.006, 0.25 * float(gmax[0] - gmin[0]))
                dy_in = min(0.006, 0.25 * float(gmax[1] - gmin[1]))
                # Primary ray stays top-centre (unchanged on camera_stand and uncovered fixtures); corner and outer-wall
                # fallback rays run only when top-centre hits an open cavity (toolbox, drawer, key_slot) or a spawned
                # Rigid object resting at (gx, gy) (coin0 on vertical_coin_stand, equation tokens on mat0..mat4).
                probe_rays = [
                    ((gx, gy, z_above), (0.0, 0.0, -0.8), 14),
                    ((float(gmin[0] + dx_in), float(gmin[1] + dy_in), z_above), (0.0, 0.0, -0.8), 14),
                    ((float(gmax[0] - dx_in), float(gmax[1] - dy_in), z_above), (0.0, 0.0, -0.8), 14),
                    ((float(gmin[0] - probe_radius - 0.004), gy, gz_mid), (0.8, 0.0, 0.0), 14),
                    ((float(gmax[0] + probe_radius + 0.004), gy, gz_mid), (-0.8, 0.0, 0.0), 14),
                    ((gx, float(gmin[1] - probe_radius - 0.004), gz_mid), (0.0, 0.8, 0.0), 14),
                    ((gx, float(gmax[1] + probe_radius + 0.004), gz_mid), (0.0, -0.8, 0.0), 14),
                ]
            matched_target = []
            cand = []
            for p_start, p_vel, p_steps in probe_rays:
                cand = _touch_and_read(p_start, p_vel, steps=p_steps)
                matched_target = [
                    c for c in cand if c["collider0"].startswith(obj.prim_path) or c["collider1"].startswith(obj.prim_path)
                ]
                if matched_target:
                    break
            if not matched_target and gc.IsA(UsdGeom.Mesh):
                # Fallback for recessed/tilted Geometry meshes (e.g. Geometry/monitor/00001 whose screen sits 49.9 mm
                # inside its stand AABB): drop straight onto the collider mesh's highest world-space vertex.
                raw_mpts = UsdGeom.Mesh(gc).GetPointsAttr().Get()
                if raw_mpts is not None and len(raw_mpts) > 0:
                    xf_gc = np.asarray(
                        UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(gc), dtype=float
                    )
                    mpts_l = np.asarray(raw_mpts, dtype=float)
                    mpts_w = (np.hstack([mpts_l, np.ones((len(mpts_l), 1), dtype=float)]) @ xf_gc)[:, :3]
                    v_top = mpts_w[int(np.argmax(mpts_w[:, 2]))]
                    cand = _touch_and_read(
                        (float(v_top[0]), float(v_top[1]), float(v_top[2]) + probe_radius + 0.004),
                        (0.0, 0.0, -0.8),
                        steps=14,
                    )
                    matched_target = [
                        c for c in cand if c["collider0"].startswith(obj.prim_path) or c["collider1"].startswith(obj.prim_path)
                    ]
            if matched_target:
                results["geometry"][label] = matched_target
                break
            results["geometry"][label] = cand

    # 6. Optional procedural ArticulationObject & DynamicObject positive control (cpu mode)
    art_obj = None
    dyn_obj = None
    art_prim_path = "/World/envs/env_0/articulation/sentinel_hinge/sentinel_hinge_0_991"
    dyn_prim_path = "/World/envs/env_0/dynamic/sentinel_belt/sentinel_belt_0_992"
    if include_procedural_classes and resolver is not None and "values" in resolver.fixes:
        work_tmp = tmpdir or tempfile.mkdtemp(prefix="d2pr1_sentinel_")
        os.makedirs(work_tmp, exist_ok=True)
        art_usd, dyn_usd = _write_procedural_test_assets(work_tmp)
        ox, oy, oz = origins[0]

        art_spec = resolver.resolve_articulation(
            prim_path=art_prim_path,
            asset_id="Articulation/sentinel_hinge/00000",
            inst_config={"physics": {"type": "articulation", "fixed_base": True}},
            scale=(1.0, 1.0, 1.0),
            env_id=0,
        )
        art_local = (3.2, -3.2, 0.40)
        art_pos = (float(ox + art_local[0]), float(oy + art_local[1]), float(oz + art_local[2]))
        art_obj = ArticulationObject(
            prim_path=art_prim_path,
            usd_path=art_usd,
            inst_config={"physics": {"type": "articulation", "fixed_base": True}, "visual": {}},
            env_origin=env.sim.scene.env_origins[0],
            default_pos=art_local,
            default_ori=(1.0, 0.0, 0.0, 0.0),
            scale=(1.0, 1.0, 1.0),
            physics_spec=art_spec,
        )
        art_obj.initialize()

        dyn_spec = resolver.resolve_dynamic(
            prim_path=dyn_prim_path,
            asset_id="Dynamic/sentinel_belt/00000",
            inst_config={"physics": {"type": "dynamic"}},
            scale=(1.0, 1.0, 1.0),
            metadata_bbox_extents=(0.30, 0.20, 0.04),
            env_id=0,
        )
        dyn_local = (-3.2, -3.2, 0.40)
        dyn_pos = (float(ox + dyn_local[0]), float(oy + dyn_local[1]), float(oz + dyn_local[2]))
        dyn_obj = DynamicObject(
            prim_path=dyn_prim_path,
            usd_path=dyn_usd,
            inst_config={"physics": {"type": "dynamic"}, "visual": {}},
            env_origin=env.sim.scene.env_origins[0],
            default_pos=dyn_local,
            default_ori=(1.0, 0.0, 0.0, 0.0),
            scale=(1.0, 1.0, 1.0),
            physics_spec=dyn_spec,
        )
        dyn_obj.initialize()
        env.sim_step(render=False)

        art_phys_view = SimulationManager.get_physics_sim_view().create_articulation_view(art_prim_path)
        art_masses = [round(float(x), 5) for x in np.asarray(art_phys_view.get_masses()).reshape(-1)]
        art_mats = [[round(float(v), 4) for v in row] for row in np.asarray(art_phys_view.get_material_properties()).reshape(-1, 3)]
        art_stiff = [round(float(x), 4) for x in np.asarray(art_phys_view.get_dof_stiffnesses()).reshape(-1)]
        art_damp = [round(float(x), 4) for x in np.asarray(art_phys_view.get_dof_dampings()).reshape(-1)]
        art_fmax = [round(float(x), 4) for x in np.asarray(art_phys_view.get_dof_max_forces()).reshape(-1)]

        art_base_contacts = _touch_and_read((art_pos[0], art_pos[1], art_pos[2] + 0.02 + probe_radius + 0.004), (0.0, 0.0, -0.8), steps=8)
        art_lid_contacts = _touch_and_read((art_pos[0] + 0.14, art_pos[1], art_pos[2] + 0.02 + probe_radius + 0.004), (0.0, 0.0, -0.8), steps=8)
        dyn_contacts = _touch_and_read((dyn_pos[0], dyn_pos[1], dyn_pos[2] + 0.02 + probe_radius + 0.004), (0.0, 0.0, -0.8), steps=8)

        art_usd_cols = [_resolve_usd_physics_mat(c) for c in _iter_enabled_gprim_colliders(stage.GetPrimAtPath(art_prim_path))]
        dyn_usd_cols = [_resolve_usd_physics_mat(c) for c in _iter_enabled_gprim_colliders(stage.GetPrimAtPath(dyn_prim_path))]

        results["procedural_classes"] = {
            "ArticulationObject": {
                "prim_path": art_prim_path,
                "engine_link_masses_kg": art_masses,
                "engine_shape_materials": art_mats,
                "engine_dof_stiffness": art_stiff,
                "engine_dof_damping": art_damp,
                "engine_dof_max_force": art_fmax,
                "usd_colliders": art_usd_cols,
                "contacts_base_link": art_base_contacts,
                "contacts_lid_link": art_lid_contacts,
            },
            "DynamicObject": {
                "prim_path": dyn_prim_path,
                "usd_colliders": dyn_usd_cols,
                "contacts_belt": dyn_contacts,
            },
        }

        for tpath in (art_prim_path, dyn_prim_path):
            tp = stage.GetPrimAtPath(tpath)
            if tp and tp.IsValid():
                for cp in _iter_colliders(tp):
                    UsdPhysics.CollisionAPI(cp).CreateCollisionEnabledAttr().Set(False)
        art_obj.relocate_offscreen()
        dyn_obj.relocate_offscreen()

    # Disable collisions on probes and park far away so subsequent episode steps are completely unaffected.
    for ppath in (probe_path, slide_box_path):
        pprim = stage.GetPrimAtPath(ppath)
        if pprim and pprim.IsValid():
            UsdPhysics.CollisionAPI(pprim).CreateCollisionEnabledAttr().Set(False)
    _park_probe(probe, 0.0)
    _park_probe(slide_box_prim, 200.0)
    env.sim_step(render=False)
    return results


def _assert_contact_touched_and_matches_spec(
    clist: list[dict[str, Any]],
    *,
    label: str,
    expected_collider_prefix: str,
    forbidden_collider_prefix: str | None = None,
    expected_mat_spec: Any = None,
    failures: list[str],
) -> None:
    """Assert >=1 narrowphase contact touched ``expected_collider_prefix`` (and not ``forbidden_collider_prefix``),
    used no ``defaultMaterial``, and its contact material path resolves in USD to ``expected_mat_spec``'s values.
    """
    matched = [
        c
        for c in clist
        if (c["collider0"].startswith(expected_collider_prefix) or c["collider1"].startswith(expected_collider_prefix))
        and c.get("materials")
    ]
    if not matched:
        got_cols = [(c["collider0"], c["collider1"]) for c in clist]
        failures.append(
            f"Canary {label} produced 0 narrowphase contact material pairs on target prefix {expected_collider_prefix!r} (got colliders={got_cols})"
        )
        return
    if forbidden_collider_prefix is not None:
        for c in clist:
            if c["collider0"].startswith(forbidden_collider_prefix) or c["collider1"].startswith(forbidden_collider_prefix):
                failures.append(f"Canary {label} touched forbidden collider {forbidden_collider_prefix!r}: {c}")
    for c in matched:
        for mp in c["materials"]:
            if "/physicsScene/defaultMaterial" in mp:
                failures.append(f"Canary {label} used defaultMaterial: {c}")
        if expected_mat_spec is not None:
            side_idx = 0 if c["collider0"].startswith(expected_collider_prefix) else 1
            for mv_pair in c.get("material_values") or []:
                mv = mv_pair[side_idx]
                sf, df, rest = mv.get("static_friction"), mv.get("dynamic_friction"), mv.get("restitution")
                if (
                    sf is None
                    or df is None
                    or rest is None
                    or abs(float(sf) - expected_mat_spec.static_friction) > 1e-4
                    or abs(float(df) - expected_mat_spec.dynamic_friction) > 1e-4
                    or abs(float(rest) - expected_mat_spec.restitution) > 1e-4
                ):
                    failures.append(
                        f"Canary {label} target material {mv} != expected "
                        f"({expected_mat_spec.static_friction}, {expected_mat_spec.dynamic_friction}, {expected_mat_spec.restitution})"
                    )


def collect_and_verify_readback(
    env: Any,
    *,
    assert_spec: bool = False,
    run_canaries: bool = False,
    sentinel_all_classes: bool = False,
    tmpdir: str | None = None,
) -> dict[str, Any]:
    """Collect full engine + USD readback across all ``num_envs`` and optionally assert against resolved specs."""
    from isaacsim.core.simulation_manager import SimulationManager
    import omni.usd
    from pxr import PhysxSchema, UsdGeom

    stage = omni.usd.get_context().get_stage()
    sim_view = SimulationManager.get_physics_sim_view()
    resolver = getattr(env, "physics_resolver", None)
    lm = env.scene_manager.layout_manager
    failures: list[str] = []

    import carb

    carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    cpu_contacts_supported = str(env.device).startswith("cpu")

    def _iter_env_label_inst(env_idx: int) -> list[tuple[str, str]]:
        """Yield ``(label_or_cluttered_id, inst_name)`` for all scene objects in ``env_idx``, including unlabeled ``type='cluttered'`` items."""
        out_items: list[tuple[str, str]] = []
        seen_insts: set[str] = set()
        for t_sec in ("Rigid", "Geometry", "Articulation", "Garment", "Fluid"):
            rec_obj = lm.object_records_by_type.get(t_sec)
            if rec_obj is None or env_idx >= len(rec_obj.layout_records_by_env):
                continue
            for obj_cfg in rec_obj.layout_records_by_env[env_idx]:
                inst_n = obj_cfg.get("inst_name")
                if not inst_n or inst_n in seen_insts:
                    continue
                seen_insts.add(inst_n)
                lbl_n = obj_cfg.get("label") or f"cluttered_{inst_n}"
                out_items.append((str(lbl_n), str(inst_n)))
        return out_items

    all_rigid_paths: list[str] = []
    for env_i in range(env.num_envs):
        for _label, inst in _iter_env_label_inst(env_i):
            if lm.instance_type_by_env[env_i].get(inst) == "rigid":
                obj = lm.get_scene_object(env_i, inst)
                if obj is not None:
                    p = stage.GetPrimAtPath(obj.prim_path)
                    if p and p.IsValid() and not p.HasAPI(PhysxSchema.PhysxContactReportAPI):
                        PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr().Set(0.0)
                    all_rigid_paths.append(obj.prim_path)
    env.sim_step(render=False)

    if cpu_contacts_supported:
        resting_contacts: list[dict[str, Any]] | None = (
            _collect_contacts_for_prefixes(all_rigid_paths, stage=stage) if all_rigid_paths else []
        )
        contact_evidence_status = "verified_cpu_contact_report"
        if all_rigid_paths and not resting_contacts:
            failures.append("Resting contact report on cpu is empty despite rigid objects in scene")
    else:
        resting_contacts = None
        contact_evidence_status = "absent_direct_gpu_contact_report_unsupported"

    per_env_objects: dict[str, dict[str, Any]] = {}
    per_env_fixtures: dict[str, dict[str, Any]] = {}
    per_env_robots: dict[str, dict[str, Any]] = {}
    compensations_used: list[dict[str, Any]] = []

    for env_i in range(env.num_envs):
        env_key = f"env_{env_i}"
        objs_i: dict[str, Any] = {}
        for label, inst in _iter_env_label_inst(env_i):
            otype = lm.instance_type_by_env[env_i].get(inst)
            obj = lm.get_scene_object(env_i, inst)
            if obj is None:
                continue
            prim = stage.GetPrimAtPath(obj.prim_path)
            all_cols = _iter_colliders(prim)
            enabled_gprim_cols = _iter_enabled_gprim_colliders(prim)
            non_gprim_enabled = [
                str(c.GetPath())
                for c in all_cols
                if not bool(UsdGeom.Gprim(c)) and _attr(c, "physics:collisionEnabled") is not False
            ]
            from env.scene_manager.physics_spec import classify_duplicate_colliders

            gprim_col_entries = [
                (str(c.GetPath()), _attr(c, "physics:collisionEnabled") is not False)
                for c in all_cols
                if bool(UsdGeom.Gprim(c))
            ]
            dup_info = classify_duplicate_colliders(str(prim.GetPath()), gprim_col_entries, strip_pre_disabled=True)
            en_map = dict(gprim_col_entries)
            visual_dups_enabled = [p_s for p_s in dup_info["visual_duplicates"] if en_map.get(p_s, False)]
            scale_t = tuple(float(x) for x in getattr(obj, "scale", (1.0, 1.0, 1.0)))
            aabb_ext = compute_collider_aabb_extents(prim, scale=scale_t)
            if cpu_contacts_supported:
                if otype == "rigid":
                    live_pos, live_ori = lm.get_instance_pose(env_idx=env_i, inst_name=inst, relative=False)
                    eng_bounds = compute_engine_collider_aabb_extents(prim, world_pos=live_pos, world_ori_wxyz=live_ori)
                else:
                    # Static Geometry prims sit at their authored USD world transform (including env_origin);
                    # LayoutManager.get_instance_pose ignores relative=False on geometry (layout_manager.py:455).
                    eng_bounds = compute_engine_collider_aabb_extents(prim)
            else:
                eng_bounds = None
            eng_aabb_ext = eng_bounds["extents_m"] if eng_bounds is not None else None
            spec = getattr(obj, "physics_spec", None)
            exp_ext = list(spec.expected_extents_m) if (spec is not None and getattr(spec, "expected_extents_m", None) is not None) else None
            aabb_rel_err = (
                round(max(abs(aabb_ext[k] - exp_ext[k]) / max(exp_ext[k], 1e-9) for k in range(3)), 5)
                if (aabb_ext is not None and exp_ext is not None)
                else None
            )
            eng_aabb_rel_err = (
                round(max(abs(eng_aabb_ext[k] - exp_ext[k]) / max(exp_ext[k], 1e-9) for k in range(3)), 5)
                if (eng_aabb_ext is not None and exp_ext is not None)
                else None
            )
            eng_aabb_abs_err = (
                round(max(abs(eng_aabb_ext[k] - exp_ext[k]) for k in range(3)), 6)
                if (eng_aabb_ext is not None and exp_ext is not None)
                else None
            )
            eng_vs_usd_narrowing_m = (
                [round(aabb_ext[k] - eng_aabb_ext[k], 6) for k in range(3)]
                if (eng_aabb_ext is not None and aabb_ext is not None)
                else None
            )

            rec: dict[str, Any] = {
                "inst": inst,
                "prim": obj.prim_path,
                "type": otype,
                "asset_id": getattr(spec, "asset_id", None) if spec is not None else None,
                "colliders": len(all_cols),
                "colliders_enabled": len(enabled_gprim_cols),
                "non_gprim_colliders_enabled": non_gprim_enabled,
                "visual_duplicate_colliders_enabled": visual_dups_enabled,
                "approximations": sorted({str(_attr(c, "physics:approximation")) for c in enabled_gprim_cols}),
                "collider_aabb_extents_m": aabb_ext,
                "engine_collider_aabb_extents_m": eng_aabb_ext,
                "engine_vs_usd_narrowing_m": eng_vs_usd_narrowing_m,
                "engine_hit_colliders": eng_bounds["hit_colliders"] if eng_bounds is not None else None,
                "expected_extents_m": exp_ext,
                "aabb_max_rel_err": aabb_rel_err,
                "engine_aabb_max_rel_err": eng_aabb_rel_err,
                "engine_aabb_max_abs_err_m": eng_aabb_abs_err,
                "usd_materials": [_resolve_usd_physics_mat(c) for c in enabled_gprim_cols],
            }
            obj_col_disabled = (
                spec is not None
                and getattr(spec, "collision", None) is not None
                and spec.collision.enabled is False
            )
            if obj_col_disabled:
                if len(enabled_gprim_cols) > 0:
                    failures.append(
                        f"{env_key}.{label}: enabled Gprim colliders={len(enabled_gprim_cols)} > 0 despite collision.enabled=False"
                    )
            elif len(enabled_gprim_cols) < 1:
                failures.append(f"{env_key}.{label}: enabled Gprim colliders={len(enabled_gprim_cols)} < 1")
            if resolver is not None and "collision" in resolver.fixes and otype == "geometry" and non_gprim_enabled:
                failures.append(f"{env_key}.{label}: Xform/non-Gprim collider(s) enabled under collision fix: {non_gprim_enabled}")
            if resolver is not None and "collision" in resolver.fixes and otype in ("rigid", "geometry"):
                if visual_dups_enabled:
                    failures.append(
                        f"{env_key}.{label}: duplicate visual-branch Gprim collider(s) enabled under collision fix (#24): {visual_dups_enabled}"
                    )
                if dup_info["disabled_duplicates"]:
                    failures.append(
                        f"{env_key}.{label}: pre-disabled duplicate Gprim collider(s) still carry CollisionAPI under collision fix (#24): {dup_info['disabled_duplicates']}"
                    )

            # Enforce <= 2% AABB check across ALL object types (rigid, geometry, dynamic) in BOTH USD and PhysX engine
            # when 'size' fix is active (skipping zero-collider visual markers with collision.enabled=False).
            if resolver is not None and "size" in resolver.fixes and exp_ext is not None and not obj_col_disabled:
                if aabb_rel_err is None or aabb_rel_err > 0.02:
                    failures.append(
                        f"{env_key}.{label} ({otype}): collider AABB {aabb_ext} differs by {aabb_rel_err} (>2%) from {exp_ext}"
                    )
                if cpu_contacts_supported:
                    spec_aid = getattr(spec, "asset_id", None)
                    allowance_ok, comp_kind, narrowing_m = evaluate_collider_narrowing_allowance(
                        asset_id=spec_aid,
                        approximations=rec["approximations"],
                        usd_aabb_extents_m=aabb_ext,
                        engine_aabb_extents_m=eng_aabb_ext,
                        expected_extents_m=exp_ext,
                        allowance_m=COLLIDER_NARROWING_ALLOWANCE_M,
                    )
                    if eng_aabb_rel_err is not None and eng_aabb_rel_err > 0.02 and allowance_ok and comp_kind is not None:
                        masks_issue = "#25" if comp_kind == "vhacd_min_thickness_allowance" else "#95"
                        eff_allow_m, one_sided_str = collider_compensation_logged_limits(
                            spec_aid, comp_kind, COLLIDER_NARROWING_ALLOWANCE_M
                        )
                        comp_entry: dict[str, Any] = {
                            "kind": comp_kind,
                            "masks": masks_issue,
                            "env": env_key,
                            "label": label,
                            "asset_id": spec_aid,
                            "approximations": list(rec["approximations"]),
                            "usd_collider_aabb_extents_m": aabb_ext,
                            "engine_collider_aabb_extents_m": eng_aabb_ext,
                            "engine_vs_usd_narrowing_m": narrowing_m,
                            "engine_max_narrowing_vs_usd_m": round(max(narrowing_m), 6) if narrowing_m is not None else None,
                            "engine_aabb_max_rel_err": eng_aabb_rel_err,
                            "engine_aabb_max_abs_err_m": eng_aabb_abs_err,
                            "allowance_m": eff_allow_m,
                            "one_sided": one_sided_str,
                        }
                        if comp_kind == "convex_hull_vertex_limit_allowance":
                            comp_entry["physx_hull_vertex_limit"] = 64
                        rec["collider_width_compensation"] = comp_entry
                        compensations_used.append(comp_entry)
                    if eng_aabb_rel_err is None or (eng_aabb_rel_err > 0.02 and not allowance_ok):
                        failures.append(
                            f"{env_key}.{label} ({otype}): PhysX engine collider AABB {eng_aabb_ext} vs USD {aabb_ext} "
                            f"(narrowing={narrowing_m} m) differs by {eng_aabb_rel_err} (>2%, abs={eng_aabb_abs_err} m) from {exp_ext}"
                        )

            if otype == "rigid":
                view = sim_view.create_rigid_body_view(obj.prim_path)
                masses = np.asarray(view.get_masses()).reshape(-1)
                coms = np.asarray(view.get_coms()).reshape(-1, 7)[:, :3]
                inertias = np.asarray(view.get_inertias()).reshape(-1, 9)
                diag_inertia = inertias[:, [0, 4, 8]]
                mats = np.asarray(view.get_material_properties()).reshape(-1, 3)

                rec["engine_mass_kg"] = [round(float(x), 6) for x in masses]
                rec["engine_com_m"] = [[round(float(c), 6) for c in row] for row in coms]
                rec["engine_diag_inertia_kg_m2"] = [[round(float(d), 8) for d in row] for row in diag_inertia]
                rec["engine_inertia_over_mass_m2"] = [
                    [round(float(d / max(float(masses[idx_m]), 1e-12)), 8) for d in row]
                    for idx_m, row in enumerate(diag_inertia)
                ]
                rec["engine_shape_count"] = int(mats.shape[0])
                rec["engine_shape_materials"] = [[round(float(v), 5) for v in row] for row in mats]

                if not np.all(np.isfinite(coms)):
                    failures.append(f"{env_key}.{label}: non-finite COM {coms.tolist()}")
                if not np.all(diag_inertia > 0.0):
                    failures.append(f"{env_key}.{label}: non-positive diagonal inertia {diag_inertia.tolist()}")

                if spec is not None:
                    if abs(float(masses[0]) - spec.mass_kg) > max(1e-5, 1e-4 * spec.mass_kg):
                        failures.append(
                            f"{env_key}.{label}: engine mass {float(masses[0]):.6f} != spec {spec.mass_kg:.6f}"
                        )
                    if spec.author_usd_mass:
                        if spec.com_offset_m is not None:
                            target_com = np.asarray(spec.com_offset_m, dtype=float)
                            if not np.allclose(coms[0], target_com, atol=1e-4, rtol=1e-3):
                                failures.append(
                                    f"{env_key}.{label}: engine COM {coms[0].tolist()} != authored spec {target_com.tolist()}"
                                )
                        if spec.inertia_kg_m2 is not None:
                            target_iner = np.asarray(spec.inertia_kg_m2, dtype=float)
                            if not np.allclose(diag_inertia[0], target_iner, rtol=1e-3, atol=1e-8):
                                failures.append(
                                    f"{env_key}.{label}: engine diagonal inertia {diag_inertia[0].tolist()} != authored shell/composite spec {target_iner.tolist()}"
                                )
                        elif getattr(spec, "solid_collider_inertia_over_mass_m2", None) is not None:
                            i_over_m = diag_inertia[0] / max(float(masses[0]), 1e-12)
                            ref_iom = np.asarray(spec.solid_collider_inertia_over_mass_m2, dtype=float)
                            if not np.allclose(i_over_m, ref_iom, rtol=0.08, atol=1e-6):
                                failures.append(
                                    f"{env_key}.{label}: engine solid I/m {i_over_m.tolist()} outside 8% of hull shape factor {ref_iom.tolist()}"
                                )
                        if aabb_ext is not None:
                            # Upper bound sanity check: I_axis <= m * (L_j^2 + L_k^2) / 4 (catches stale post-parse set_mass inertia #33).
                            i_over_m = diag_inertia[0] / max(float(masses[0]), 1e-12)
                            lx, ly, lz = aabb_ext
                            bbox_i_bound = np.array([ly * ly + lz * lz, lx * lx + lz * lz, lx * lx + ly * ly], dtype=float) * 0.25
                            if np.any(i_over_m > bbox_i_bound * 1.05):
                                failures.append(
                                    f"{env_key}.{label}: I/m {i_over_m.tolist()} exceeds bbox bound {bbox_i_bound.tolist()} (stale post-parse inertia #33)"
                                )
                    if spec.author_pre_parse:
                        target_m = np.array(
                            [spec.material.static_friction, spec.material.dynamic_friction, spec.material.restitution],
                            dtype=float,
                        )
                        max_diff = float(np.max(np.abs(mats - target_m[None, :])))
                        if max_diff > 1e-4:
                            failures.append(
                                f"{env_key}.{label}: shape material max_diff={max_diff:.6f} vs {target_m.tolist()} across {mats.shape[0]} shapes"
                            )
                    elif resolver is not None and resolver.is_shipped and str(env.device).startswith("cpu"):
                        rec["shipped_cpu_05_confirmed"] = bool(
                            np.max(np.abs(mats - np.array([[0.5, 0.5, 0.0]]))) < 1e-4
                        )
            elif otype == "geometry" and spec is not None and spec.bind_material:
                for idx_c, um in enumerate(rec["usd_materials"]):
                    if um["purpose"] == "scene-default":
                        failures.append(f"{env_key}.{label} collider[{idx_c}] resolved to scene-default")
            elif otype == "articulation":
                aview = sim_view.create_articulation_view(obj.prim_path)
                amasses = np.asarray(aview.get_masses()).reshape(-1)
                amats = np.asarray(aview.get_material_properties()).reshape(-1, 3)
                astiff = np.asarray(aview.get_dof_stiffnesses()).reshape(-1)
                adamp = np.asarray(aview.get_dof_dampings()).reshape(-1)
                rec["engine_link_masses_kg"] = [round(float(x), 6) for x in amasses]
                rec["engine_shape_materials"] = [[round(float(v), 5) for v in row] for row in amats]
                rec["engine_dof_stiffnesses"] = [round(float(x), 4) for x in astiff]
                rec["engine_dof_dampings"] = [round(float(x), 4) for x in adamp]

            objs_i[label] = rec
        per_env_objects[env_key] = objs_i

        # Fixtures (Table, Ground, Room; Room lives at env.scene_manager._rooms[env_i].prim_path = {env_root}/Rooms/{cat}/{inst})
        fix_i: dict[str, Any] = {}
        env_root = f"/World/envs/env_{env_i}"
        room_obj_i = env.scene_manager._rooms[env_i]
        for fname, fobj, fpath in (
            ("Table", env.scene_manager._tables[env_i], f"{env_root}/Table"),
            (
                "Ground",
                env.scene_manager._grounds[env_i][0] if env.scene_manager._grounds[env_i] else None,
                f"{env_root}/Ground",
            ),
            (
                "Room",
                room_obj_i,
                getattr(room_obj_i, "prim_path", None) if room_obj_i is not None else None,
            ),
        ):
            if not fpath:
                continue
            fprim = stage.GetPrimAtPath(fpath)
            if not fprim or not fprim.IsValid():
                if fobj is not None:
                    failures.append(f"{env_key}.{fname}: fixture object exists at {fpath!r} but USD prim is missing/invalid")
                continue
            all_fcols = _iter_colliders(fprim)
            fcols = _iter_enabled_gprim_colliders(fprim)
            non_gprim_fcols = [
                str(c.GetPath())
                for c in all_fcols
                if not bool(UsdGeom.Gprim(c)) and _attr(c, "physics:collisionEnabled") is not False
            ]
            fspec = getattr(fobj, "physics_spec", None) if fobj is not None else None
            fcol_disabled = (
                fspec is not None
                and getattr(fspec, "collision", None) is not None
                and fspec.collision.enabled is False
            )
            faabb = compute_collider_aabb_extents(fprim, scale=(1.0, 1.0, 1.0))
            from pxr import Usd

            bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide])
            wb = bc.ComputeWorldBound(fprim).ComputeAlignedRange()
            w_ext = [round(float(wb.GetMax()[k] - wb.GetMin()[k]), 5) for k in range(3)] if not wb.IsEmpty() else faabb
            exp_f = list(fspec.expected_extents_m) if (fspec is not None and getattr(fspec, "expected_extents_m", None) is not None) else None
            f_err = (
                round(max(abs(w_ext[k] - exp_f[k]) / max(exp_f[k], 1e-9) for k in range(3)), 5)
                if (w_ext is not None and exp_f is not None)
                else None
            )
            umats = [_resolve_usd_physics_mat(c) for c in fcols]
            fix_i[fname] = {
                "prim": fpath,
                "colliders": len(all_fcols),
                "colliders_enabled": len(fcols),
                "non_gprim_colliders_enabled": non_gprim_fcols,
                "approximations": sorted({str(_attr(c, "physics:approximation")) for c in fcols}),
                "world_aabb_extents_m": w_ext,
                "expected_extents_m": exp_f,
                "aabb_max_rel_err": f_err,
                "usd_materials": umats,
            }
            if resolver is not None and "collision" in resolver.fixes and not fcol_disabled:
                if len(fcols) < 1:
                    failures.append(f"{env_key}.{fname}: enabled Gprim colliders={len(fcols)} < 1")
                if non_gprim_fcols:
                    failures.append(f"{env_key}.{fname}: Xform/non-Gprim collider(s) enabled under collision fix: {non_gprim_fcols}")
            if resolver is not None and "size" in resolver.fixes and f_err is not None and f_err > 0.02:
                failures.append(f"{env_key}.{fname}: fixture AABB {w_ext} differs by {f_err:.4f} (>2%) from {exp_f}")
            if resolver is not None and _materials_on(resolver) and "order" in resolver.fixes:
                if any(u["purpose"] == "scene-default" for u in umats):
                    failures.append(f"{env_key}.{fname}: collider resolved to scene-default")
                if fspec is not None and fspec.material is not None:
                    for u in umats:
                        if (
                            u.get("static_friction") is None
                            or abs(float(u["static_friction"]) - fspec.material.static_friction) > 1e-4
                            or abs(float(u["dynamic_friction"]) - fspec.material.dynamic_friction) > 1e-4
                            or abs(float(u["restitution"]) - fspec.material.restitution) > 1e-4
                        ):
                            failures.append(
                                f"{env_key}.{fname}: USD bound material {u} != spec "
                                f"({fspec.material.static_friction}, {fspec.material.dynamic_friction}, {fspec.material.restitution})"
                            )
        per_env_fixtures[env_key] = fix_i

        # Robots (masses, COMs, inertias, per-link shape materials for link7 and link8, drive gains/limits, finger de-instancing)
        robs_i: dict[str, Any] = {}
        for ri, rk in enumerate(env.robot_manager.robot_key):
            rv = rk.root_physx_view
            rmasses = np.asarray(rv.get_masses())[env_i]
            rcoms = np.asarray(rv.get_coms())[env_i, :, :3]
            rinert = np.asarray(rv.get_inertias())[env_i].reshape(-1, 9)[:, [0, 4, 8]]
            rmats = np.asarray(rv.get_material_properties())[env_i]
            rstiff = np.asarray(rv.get_dof_stiffnesses())[env_i]
            rdamp = np.asarray(rv.get_dof_dampings())[env_i]
            rfmax = np.asarray(rv.get_dof_max_forces())[env_i]
            rvmax = np.asarray(rv.get_dof_max_velocities())[env_i]

            # Exact per-link shape index ranges into rv.get_material_properties() (IsaacLab events.py:210-216).
            link_paths_env = list(rv.link_paths[env_i])
            shapes_per_link = [int(sim_view.create_rigid_body_view(lp).max_shapes) for lp in link_paths_env]
            link_shape_slices: dict[str, tuple[int, int]] = {}
            cursor = 0
            for lp, n_sh in zip(link_paths_env, shapes_per_link):
                lname = lp.split("/")[-1]
                link_shape_slices[lname] = (cursor, cursor + n_sh)
                cursor += n_sh

            finger_usd: dict[str, Any] = {}
            per_link_engine_mats: dict[str, list[list[float]]] = {}
            per_finger_engine_mats: dict[str, list[list[float]]] = {}
            all_robot_links = list(rk.body_names)
            for link in all_robot_links:
                if link in link_shape_slices:
                    s0, s1 = link_shape_slices[link]
                    per_link_engine_mats[link] = [[round(float(v), 5) for v in row] for row in rmats[s0:s1]]
            for link in ("link7", "link8"):
                cg = stage.GetPrimAtPath(f"{env_root}/robot{ri}/{link}/collisions")
                lp = stage.GetPrimAtPath(f"{env_root}/robot{ri}/{link}")
                lcols = [c for c in _iter_colliders(lp) if _attr(c, "physics:collisionEnabled") is not False]
                if link in per_link_engine_mats:
                    per_finger_engine_mats[link] = per_link_engine_mats[link]
                finger_usd[link] = {
                    "collisions_instanceable": bool(cg.IsInstanceable()) if (cg and cg.IsValid()) else None,
                    "engine_shape_slice": list(link_shape_slices[link]) if link in link_shape_slices else None,
                    "engine_shape_materials": per_finger_engine_mats.get(link),
                    "colliders": [
                        {
                            "path": str(c.GetPath()),
                            "is_instance_proxy": bool(c.IsInstanceProxy()),
                            "torsional_patch_radius": _attr(c, "physxCollision:torsionalPatchRadius"),
                            "min_torsional_patch_radius": _attr(c, "physxCollision:minTorsionalPatchRadius"),
                            "usd_material": _resolve_usd_physics_mat(c),
                        }
                        for c in lcols
                    ],
                }

            robs_i[f"robot{ri}"] = {
                "links": list(rk.body_names),
                "joint_names": list(rk.joint_names),
                "masses_kg": [round(float(x), 6) for x in rmasses],
                "coms_finite": bool(np.all(np.isfinite(rcoms))),
                "inertias_positive": bool(np.all(rinert > 0.0)),
                "camera_links_mass_com": {
                    lname: {
                        "mass_kg": round(float(rmasses[list(rk.body_names).index(lname)]), 6),
                        "com_m": [round(float(c), 6) for c in rcoms[list(rk.body_names).index(lname)]],
                    }
                    for lname in ("camera_base", "camera")
                    if lname in rk.body_names
                },
                "dof_stiffnesses": [round(float(x), 4) for x in rstiff],
                "dof_dampings": [round(float(x), 4) for x in rdamp],
                "dof_max_forces": [round(float(x), 4) for x in rfmax],
                "dof_max_velocities": [round(float(x), 4) for x in rvmax],
                "shape_count": int(rmats.shape[0]),
                "shapes_per_link_sum": int(cursor),
                "unique_shape_materials": sorted({tuple(round(float(v), 5) for v in row) for row in rmats}),
                "per_finger_engine_materials": per_finger_engine_mats,
                "per_link_engine_materials": per_link_engine_mats,
                "all_shape_materials": [[round(float(v), 5) for v in row] for row in rmats],
                "fingers_usd": finger_usd,
            }

            if cursor != int(rmats.shape[0]):
                failures.append(f"{env_key}.robot{ri}: sum(shapes_per_link)={cursor} != max_shapes={int(rmats.shape[0])}")
            if not np.all(np.isfinite(rcoms)):
                failures.append(f"{env_key}.robot{ri}: non-finite link COMs")
            if not np.all(rinert > 0.0):
                failures.append(f"{env_key}.robot{ri}: non-positive link inertias")

            if list(rk.joint_names) == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"]:
                exp_k = np.array([4400.0] * 6 + [2300.0] * 2)
                exp_d = np.array([40.0] * 6 + [100.0] * 2)
                exp_f = np.array([100.0] * 8)
                exp_v = np.array([5.0] * 6 + [1000.0] * 2)
                gspec = resolver.resolve_gripper(env.robot_manager.robot_list[ri].robot_name) if resolver is not None else None
                if gspec is not None:
                    # D3 (#90): joint7 carries both fingers (2x per-finger values); joint8's drive is off; vmax unchanged.
                    exp_k[6:] = [gspec.drive_stiffness, 0.0]
                    exp_d[6:] = [gspec.drive_damping, 0.0]
                    exp_f[6] = gspec.drive_effort_limit
                    jm = stage.GetPrimAtPath(f"{env_root}/robot{ri}/joints/{gspec.mimic_joint}")
                    pfx = f"physxMimicJoint:{gspec.mimic_axis}"
                    ref_t = jm.GetRelationship(f"{pfx}:referenceJoint").GetTargets() if (jm and jm.IsValid()) else []
                    mimic_rb = {
                        "applied_schemas": [str(x) for x in jm.GetAppliedSchemas()] if (jm and jm.IsValid()) else None,
                        "reference_joint": [str(t) for t in ref_t],
                        "gearing": _attr(jm, f"{pfx}:gearing") if (jm and jm.IsValid()) else None,
                        "offset": _attr(jm, f"{pfx}:offset") if (jm and jm.IsValid()) else None,
                        "reference_joint_axis": _attr(jm, f"{pfx}:referenceJointAxis") if (jm and jm.IsValid()) else None,
                    }
                    robs_i[f"robot{ri}"]["gripper_mimic_usd"] = mimic_rb
                    if f"PhysxMimicJointAPI:{gspec.mimic_axis}" not in (mimic_rb["applied_schemas"] or []):
                        failures.append(f"{env_key}.robot{ri}: {gspec.mimic_joint} lacks PhysxMimicJointAPI:{gspec.mimic_axis}")
                    if [t.split("/")[-1] for t in mimic_rb["reference_joint"]] != [gspec.driven_joint]:
                        failures.append(f"{env_key}.robot{ri}: mimic referenceJoint {mimic_rb['reference_joint']} != {gspec.driven_joint}")
                    if mimic_rb["gearing"] is None or abs(float(mimic_rb["gearing"]) - gspec.mimic_gearing) > 1e-6:
                        failures.append(f"{env_key}.robot{ri}: mimic gearing {mimic_rb['gearing']} != {gspec.mimic_gearing}")
                    if mimic_rb["offset"] is None or abs(float(mimic_rb["offset"]) - gspec.mimic_offset) > 1e-9:
                        failures.append(f"{env_key}.robot{ri}: mimic offset {mimic_rb['offset']} != {gspec.mimic_offset}")
                    if str(mimic_rb["reference_joint_axis"]) != gspec.mimic_axis:
                        failures.append(f"{env_key}.robot{ri}: mimic referenceJointAxis {mimic_rb['reference_joint_axis']} != {gspec.mimic_axis}")
                    # D3 item 3: q8 follows q7 in the engine (joint8's drive is off, so only the mimic holds it there).
                    rpos = np.asarray(rv.get_dof_positions())[env_i]
                    mimic_rb["engine_q7_q8_m"] = [float(rpos[6]), float(rpos[7])]
                    if abs(float(rpos[7]) + gspec.mimic_gearing * float(rpos[6]) + gspec.mimic_offset) > 1e-4:
                        failures.append(f"{env_key}.robot{ri}: engine q8 {float(rpos[7])} does not follow q7 {float(rpos[6])}")
                    if not np.allclose(rvmax[6:], exp_v[6:], rtol=1e-4):
                        failures.append(f"{env_key}.robot{ri}: gripper max_vel {rvmax[6:].tolist()} != {exp_v[6:].tolist()}")
                    # #90 round 2: the motor's reflected inertia sits on joint7 (2x per finger); joint8 keeps none.
                    rarm = np.asarray(rv.get_dof_armatures())[env_i]
                    robs_i[f"robot{ri}"]["dof_armatures"] = [round(float(x), 5) for x in rarm]
                    if abs(float(rarm[6]) - gspec.drive_armature) > 1e-4 * max(1.0, gspec.drive_armature) or abs(float(rarm[7])) > 1e-9:
                        failures.append(f"{env_key}.robot{ri}: gripper armature {rarm[6:].tolist()} != [{gspec.drive_armature}, 0.0]")
                # #11: engine joint ranges; under robot_limits also the arm's velocity and effort limits.
                rlim = np.asarray(rv.get_dof_limits())[env_i]
                robs_i[f"robot{ri}"]["dof_limits_rad"] = [[round(float(a), 6), round(float(b), 6)] for a, b in rlim]
                aspec = resolver.resolve_arm_limits(env.robot_manager.robot_list[ri].robot_name) if resolver is not None else None
                if aspec is not None:
                    robs_i[f"robot{ri}"]["arm_limits_usd_deg"] = {
                        j: [_attr(stage.GetPrimAtPath(f"{env_root}/robot{ri}/joints/{j}"), "physics:lowerLimit"),
                            _attr(stage.GetPrimAtPath(f"{env_root}/robot{ri}/joints/{j}"), "physics:upperLimit")]
                        for j in aspec.joints
                    }
                    exp_v[:6] = aspec.velocity_rad_s
                    if aspec.effort_nm is not None:
                        exp_f[:6] = aspec.effort_nm
                    exp_lim = np.array([aspec.lower_rad, aspec.upper_rad]).T
                    if not np.allclose(rlim[:6], exp_lim, atol=1e-4):
                        failures.append(f"{env_key}.robot{ri}: arm dof limits {rlim[:6].tolist()} != {exp_lim.tolist()}")
                if not np.allclose(rstiff, exp_k, rtol=1e-4):
                    failures.append(f"{env_key}.robot{ri}: stiffness {rstiff.tolist()} != {exp_k.tolist()}")
                if not np.allclose(rdamp, exp_d, rtol=1e-4):
                    failures.append(f"{env_key}.robot{ri}: damping {rdamp.tolist()} != {exp_d.tolist()}")
                if not np.allclose(rfmax, exp_f, rtol=1e-4):
                    failures.append(f"{env_key}.robot{ri}: max_force {rfmax.tolist()} != {exp_f.tolist()}")
                if not np.allclose(rvmax[:6], exp_v[:6], rtol=1e-4):
                    failures.append(f"{env_key}.robot{ri}: arm max_vel {rvmax[:6].tolist()} != {exp_v[:6].tolist()}")

            if resolver is not None and _materials_on(resolver) and "order" in resolver.fixes:
                rspec = resolver.resolve_robot(env.robot_manager.robot_list[0].robot_name)
                pad_set = set(rspec.pad_links or (("link7", "link8") if rspec.robot_name == "x5" else ("panda_leftfinger", "panda_rightfinger")))
                for link, mspec in rspec.link_materials.items():
                    target = np.array([mspec.static_friction, mspec.dynamic_friction, mspec.restitution], dtype=float)
                    if link not in link_shape_slices:
                        failures.append(f"{env_key}.robot{ri}.{link}: link missing from root_physx_view.link_paths")
                        continue
                    s0, s1 = link_shape_slices[link]
                    link_mats_arr = rmats[s0:s1]
                    if link_mats_arr.shape[0] < 1 or float(np.max(np.abs(link_mats_arr - target[None, :]))) > 1e-4:
                        failures.append(
                            f"{env_key}.robot{ri}.{link}: engine link shapes[{s0}:{s1}]={link_mats_arr.tolist()} != {target.tolist()}"
                        )
                    cg_link = stage.GetPrimAtPath(f"{env_root}/robot{ri}/{link}/collisions")
                    if cg_link and cg_link.IsValid() and bool(cg_link.IsInstanceable()):
                        failures.append(f"{env_key}.robot{ri}.{link}: collisions still instanceable=True")
                    lp_check = stage.GetPrimAtPath(f"{env_root}/robot{ri}/{link}")
                    lcols_check = [c for c in _iter_colliders(lp_check) if _attr(c, "physics:collisionEnabled") is not False]
                    if len(lcols_check) < 1:
                        failures.append(f"{env_key}.robot{ri}.{link}: 0 enabled USD colliders found")
                    for lcol_prim in lcols_check:
                        um_l = _resolve_usd_physics_mat(lcol_prim)
                        if um_l.get("purpose") == "scene-default":
                            failures.append(f"{env_key}.robot{ri}.{link}: collider {lcol_prim.GetPath()} resolved to scene-default")
                    if link in pad_set and "collision" in resolver.fixes and rspec.pad_torsional_radius_m is not None:
                        fcols = finger_usd.get(link, {}).get("colliders") or []
                        for col_rec in fcols:
                            tr = col_rec.get("torsional_patch_radius")
                            if tr is None or abs(float(tr) - rspec.pad_torsional_radius_m) > 1e-6:
                                failures.append(
                                    f"{env_key}.robot{ri}.{link}: torsional_patch_radius={tr} != {rspec.pad_torsional_radius_m}"
                                )
                            if rspec.pad_min_torsional_radius_m is not None:
                                mtr = col_rec.get("min_torsional_patch_radius")
                                if mtr is None or abs(float(mtr) - rspec.pad_min_torsional_radius_m) > 1e-6:
                                    failures.append(
                                        f"{env_key}.robot{ri}.{link}: min_torsional_patch_radius={mtr} != {rspec.pad_min_torsional_radius_m}"
                                    )
                if set(rspec.link_materials.keys()) >= set(rk.body_names):
                    default_rows = [
                        idx_r
                        for idx_r, row in enumerate(rmats)
                        if float(np.max(np.abs(row - np.array([0.5, 0.5, 0.0])))) < 1e-5
                    ]
                    if default_rows:
                        failures.append(
                            f"{env_key}.robot{ri}: {len(default_rows)} robot shapes remained at default [0.5, 0.5, 0.0]: indices={default_rows}"
                        )
        per_env_robots[env_key] = robs_i

    # Check resting contacts on cpu when order+values are active (resting_contacts is None on cuda:0)
    sdf_trimesh_report_null_contacts: list[dict[str, Any]] = []
    if resolver is not None and "order" in resolver.fixes and _materials_on(resolver) and resting_contacts is not None:
        for rc in resting_contacts:
            p0_rc = stage.GetPrimAtPath(str(rc["collider0"]))
            p1_rc = stage.GetPrimAtPath(str(rc["collider1"]))
            ap0_rc = str(_attr(p0_rc, "physics:approximation")) if (p0_rc and p0_rc.IsValid()) else None
            ap1_rc = str(_attr(p1_rc, "physics:approximation")) if (p1_rc and p1_rc.IsValid()) else None
            um0_rc = _resolve_usd_physics_mat(p0_rc) if (p0_rc and p0_rc.IsValid()) else {}
            um1_rc = _resolve_usd_physics_mat(p1_rc) if (p1_rc and p1_rc.IsValid()) else {}
            for mpair in rc["materials"]:
                if "/physicsScene/defaultMaterial" in mpair:
                    if is_sdf_trimesh_face_index_report_null(
                        c0_approx=ap0_rc,
                        c1_approx=ap1_rc,
                        mpair=mpair,
                        c0_usd_mat=um0_rc,
                        c1_usd_mat=um1_rc,
                    ):
                        sdf_trimesh_report_null_contacts.append(
                            {
                                "collider0": rc["collider0"],
                                "collider1": rc["collider1"],
                                "approx0": ap0_rc,
                                "approx1": ap1_rc,
                                "reported_materials": mpair,
                                "verified_usd_material0": um0_rc,
                                "verified_usd_material1": um1_rc,
                                "reason": "NpShape.cpp:566 PxShape::getMaterialFromInternalFaceIndex(0xFFFFffff)->NULL on sdf x trimesh contact report; solver uses bound single-material shape.",
                            }
                        )
                    else:
                        failures.append(f"Resting contact used defaultMaterial: {rc}")

    canary_report = None
    if (run_canaries or sentinel_all_classes) and cpu_contacts_supported:
        canary_report = run_canary_touches(
            env,
            include_procedural_classes=sentinel_all_classes,
            tmpdir=tmpdir,
        )
        if resolver is not None and "order" in resolver.fixes and _materials_on(resolver):
            for env_i in range(env.num_envs):
                ek = f"env_{env_i}"
                erec = canary_report["per_env"].get(ek) or {}
                env_root = f"/World/envs/env_{env_i}"
                t_spec = getattr(env.scene_manager._tables[env_i], "physics_spec", None)
                g_spec = (
                    getattr(env.scene_manager._grounds[env_i][0], "physics_spec", None)
                    if env.scene_manager._grounds[env_i]
                    else None
                )
                _assert_contact_touched_and_matches_spec(
                    erec.get("Table") or [],
                    label=f"{ek}.Table",
                    expected_collider_prefix=f"{env_root}/Table",
                    expected_mat_spec=t_spec.material if t_spec is not None else None,
                    failures=failures,
                )
                _assert_contact_touched_and_matches_spec(
                    erec.get("Ground") or [],
                    label=f"{ek}.Ground",
                    expected_collider_prefix=f"{env_root}/Ground",
                    expected_mat_spec=g_spec.material if g_spec is not None else None,
                    failures=failures,
                )
                r_obj_assert = env.scene_manager._rooms[env_i]
                if r_obj_assert is not None:
                    r_spec = getattr(r_obj_assert, "physics_spec", None)
                    r_col_disabled = (
                        r_spec is not None
                        and getattr(r_spec, "collision", None) is not None
                        and r_spec.collision.enabled is False
                    )
                    if not r_col_disabled:
                        _assert_contact_touched_and_matches_spec(
                            erec.get("Room") or [],
                            label=f"{ek}.Room",
                            expected_collider_prefix=f"{r_obj_assert.prim_path}/",
                            expected_mat_spec=(r_spec.material if (r_spec is not None and r_spec.bind_material) else None),
                            failures=failures,
                        )

                rspec = resolver.resolve_robot(env.robot_manager.robot_list[0].robot_name)
                fingers_map = erec.get("fingers") or {}
                for ri in range(len(env.robot_manager.robot_key)):
                    for l_part in ("link7", "link8"):
                        fk = f"robot{ri}/{l_part}"
                        flist = fingers_map.get(fk) or []
                        other_link = "link8" if l_part == "link7" else "link7"
                        _assert_contact_touched_and_matches_spec(
                            flist,
                            label=f"{ek}.fingers.{fk}",
                            expected_collider_prefix=f"{env_root}/robot{ri}/{l_part}/",
                            forbidden_collider_prefix=f"{env_root}/robot{ri}/{other_link}/",
                            expected_mat_spec=rspec.link_materials.get(l_part),
                            failures=failures,
                        )

            # Value-level Coulomb slide verification on both Table and Ground in env_0 (fail loud if missing)
            fslides = canary_report.get("fixture_slides_env0") or {}
            for surf_name in ("Table", "Ground"):
                srec = fslides.get(surf_name)
                if not isinstance(srec, dict):
                    failures.append(f"Fixture slide env_0.{surf_name}: missing slide record")
                    continue
                pred_mm = srec.get("coulomb_predicted_mm")
                meas_mm = srec.get("slide_x_mm")
                if pred_mm is None or meas_mm is None:
                    failures.append(
                        f"Fixture slide env_0.{surf_name}: missing prediction ({pred_mm}) or measurement ({meas_mm})"
                    )
                elif abs(float(meas_mm) - float(pred_mm)) > 2.5:
                    failures.append(
                        f"Fixture slide env_0.{surf_name}: measured {meas_mm} mm differs by >2.5 mm from Coulomb {pred_mm} mm"
                    )

            for gk, glist in (canary_report.get("geometry") or {}).items():
                inst = lm.get_instance_name(0, gk)
                gobj = lm.get_scene_object(0, inst)
                gspec = getattr(gobj, "physics_spec", None) if gobj is not None else None
                _assert_contact_touched_and_matches_spec(
                    glist,
                    label=f"geometry.{gk}",
                    expected_collider_prefix=gobj.prim_path if gobj is not None else f"/World/envs/env_0/geometry/{gk}",
                    expected_mat_spec=gspec.material if gspec is not None else None,
                    failures=failures,
                )

            if sentinel_all_classes:
                pc = canary_report.get("procedural_classes") or {}
                art = pc.get("ArticulationObject") or {}
                dyn = pc.get("DynamicObject") or {}
                art_path = art.get("prim_path", "/World/envs/env_0/articulation/sentinel_hinge/sentinel_hinge_0_991")
                dyn_path = dyn.get("prim_path", "/World/envs/env_0/dynamic/sentinel_belt/sentinel_belt_0_992")
                art_mat_spec = resolver.params_table.class_materials.get("sentinel_articulation") if resolver.params_table else None
                dyn_mat_spec = resolver.params_table.class_materials.get("sentinel_dynamic") if resolver.params_table else None
                _assert_contact_touched_and_matches_spec(
                    art.get("contacts_base_link") or [],
                    label="ArticulationObject.base_link",
                    expected_collider_prefix=f"{art_path}/base_link/",
                    expected_mat_spec=art_mat_spec,
                    failures=failures,
                )
                _assert_contact_touched_and_matches_spec(
                    art.get("contacts_lid_link") or [],
                    label="ArticulationObject.lid_link",
                    expected_collider_prefix=f"{art_path}/lid_link/",
                    expected_mat_spec=art_mat_spec,
                    failures=failures,
                )
                art_mats = np.asarray(art.get("engine_shape_materials") or [])
                if art_mats.size == 0 or float(np.max(np.abs(art_mats - np.array([[0.888, 0.777, 0.06]])))) > 1e-4:
                    failures.append(f"ArticulationObject shape materials mismatch: {art.get('engine_shape_materials')}")
                if not np.allclose(art.get("engine_link_masses_kg") or [], [0.321, 0.123], atol=1e-4):
                    failures.append(f"ArticulationObject link masses mismatch: {art.get('engine_link_masses_kg')}")
                # Enforce procedural articulation drive gains & max_force in engine (B3)!
                if not np.allclose(art.get("engine_dof_stiffness") or [], [50.0], rtol=1e-4):
                    failures.append(f"ArticulationObject engine dof stiffness {art.get('engine_dof_stiffness')} != [50.0]")
                if not np.allclose(art.get("engine_dof_damping") or [], [5.0], rtol=1e-4):
                    failures.append(f"ArticulationObject engine dof damping {art.get('engine_dof_damping')} != [5.0]")
                if not np.allclose(art.get("engine_dof_max_force") or [], [25.0], rtol=1e-4):
                    failures.append(f"ArticulationObject engine dof max_force {art.get('engine_dof_max_force')} != [25.0]")

                _assert_contact_touched_and_matches_spec(
                    dyn.get("contacts_belt") or [],
                    label="DynamicObject.belt",
                    expected_collider_prefix=f"{dyn_path}/",
                    expected_mat_spec=dyn_mat_spec,
                    failures=failures,
                )

    realised_pair_readback: dict[str, Any] | None = None
    if (
        resolver is not None
        and "order" in resolver.fixes
        and _materials_on(resolver)
        and resolver.params_table is not None
        and resolver.params_table.realised_vs_nominal
    ):
        from env.scene_manager.physics_spec import combine_values

        ptab = resolver.params_table
        rvn = ptab.realised_vs_nominal
        rvn_cp = rvn.get("class_pairs") or {}
        rvn_op = rvn.get("asset_override_pairs") or {}

        def _lookup_rvn_target(
            aid_a: str | None, cls_a: str | None, ov_a: bool,
            aid_b: str | None, cls_b: str | None, ov_b: bool,
        ) -> tuple[str | None, dict[str, Any] | None, str]:
            if ov_a and ov_b:
                return None, None, "both_override"
            if ov_a and aid_a and cls_b:
                opk = f"{aid_a}__{cls_b}"
                return (opk, rvn_op[opk], "asset_override") if opk in rvn_op else (opk, None, "unbanded")
            if ov_b and aid_b and cls_a:
                opk = f"{aid_b}__{cls_a}"
                return (opk, rvn_op[opk], "asset_override") if opk in rvn_op else (opk, None, "unbanded")
            if cls_a and cls_b:
                cpk = f"{min(cls_a, cls_b)}__{max(cls_a, cls_b)}"
                return (cpk, rvn_cp[cpk], "class_pair") if cpk in rvn_cp else (cpk, None, "unbanded")
            return None, None, "unclassified"

        checked_scene_pairs: list[dict[str, Any]] = []
        unbanded_scene_pairs: list[str] = []
        checked_resting_contacts: list[dict[str, Any]] = []
        checked_link_static_pairs: list[dict[str, Any]] = []
        unbanded_link_static_pairs: list[str] = []
        checked_link_link_pairs: list[dict[str, Any]] = []
        unbanded_link_link_pairs: list[str] = []

        for env_i in range(env.num_envs):
            ek = f"env_{env_i}"
            objs_i = per_env_objects.get(ek) or {}
            fix_i = per_env_fixtures.get(ek) or {}
            robs_i = per_env_robots.get(ek) or {}

            rigid_sides: list[dict[str, Any]] = []
            partner_sides: list[dict[str, Any]] = []
            prefix_map: list[tuple[str, dict[str, Any]]] = []

            for lbl, orec in sorted(objs_i.items()):
                inst = orec["inst"]
                sobj = lm.get_scene_object(env_i, inst)
                sspec = getattr(sobj, "physics_spec", None) if sobj is not None else None
                if sspec is None or sspec.material is None:
                    continue
                aid = sspec.asset_id
                ainfo = ptab.assets.get(aid) or {}
                mcls = ainfo.get("material_class")
                has_ov = bool(ainfo.get("has_material_override"))
                if orec["type"] == "rigid" and orec.get("engine_shape_materials"):
                    em = orec["engine_shape_materials"][0]
                    entry_r = {
                        "name": lbl,
                        "kind": "rigid",
                        "asset_id": aid,
                        "material_class": mcls,
                        "has_override": has_ov,
                        "static_friction": float(em[0]),
                        "dynamic_friction": float(em[1]),
                        "restitution": float(em[2]),
                        "fmode": sspec.material.friction_combine_mode,
                        "rmode": sspec.material.restitution_combine_mode,
                    }
                    rigid_sides.append(entry_r)
                    prefix_map.append((str(orec["prim"]), entry_r))
                elif orec["type"] == "geometry" and orec.get("usd_materials"):
                    um0 = orec["usd_materials"][0]
                    if um0.get("static_friction") is not None:
                        entry_g = {
                            "name": lbl,
                            "kind": "geometry",
                            "asset_id": aid,
                            "material_class": mcls,
                            "has_override": has_ov,
                            "static_friction": float(um0["static_friction"]),
                            "dynamic_friction": float(um0["dynamic_friction"]),
                            "restitution": float(um0["restitution"]),
                            "fmode": sspec.material.friction_combine_mode,
                            "rmode": sspec.material.restitution_combine_mode,
                        }
                        partner_sides.append(entry_g)
                        prefix_map.append((str(orec["prim"]), entry_g))

            for fname, fobj in (
                ("Table", env.scene_manager._tables[env_i]),
                ("Ground", env.scene_manager._grounds[env_i][0] if env.scene_manager._grounds[env_i] else None),
                ("Room", env.scene_manager._rooms[env_i]),
            ):
                frec = fix_i.get(fname)
                fspec = getattr(fobj, "physics_spec", None) if fobj is not None else None
                if frec and frec.get("usd_materials") and fspec is not None and fspec.material is not None:
                    um0 = frec["usd_materials"][0]
                    if um0.get("static_friction") is not None:
                        ainfo = ptab.assets.get(fspec.asset_id) or {}
                        entry_f = {
                            "name": fname,
                            "kind": "fixture",
                            "asset_id": fspec.asset_id,
                            "material_class": ainfo.get("material_class") or ("ground" if fname == "Room" else None),
                            "has_override": bool(ainfo.get("has_material_override")),
                            "static_friction": float(um0["static_friction"]),
                            "dynamic_friction": float(um0["dynamic_friction"]),
                            "restitution": float(um0["restitution"]),
                            "fmode": fspec.material.friction_combine_mode,
                            "rmode": fspec.material.restitution_combine_mode,
                        }
                        partner_sides.append(entry_f)
                        prefix_map.append((str(frec["prim"]), entry_f))

            rob0 = robs_i.get("robot0")
            if rob0 and env.robot_manager.robot_list:
                rspec = resolver.resolve_robot(env.robot_manager.robot_list[0].robot_name)
                plem = rob0.get("per_link_engine_materials") or {}
                for lname in ("link7", "link4", "camera"):
                    if lname in plem and plem[lname] and lname in rspec.link_materials:
                        lms = rspec.link_materials[lname]
                        em_l = plem[lname][0]
                        matched_cls = next(
                            (
                                c
                                for c, cm in sorted(ptab.class_materials.items())
                                if abs(cm.static_friction - lms.static_friction) < 1e-6
                                and abs(cm.dynamic_friction - lms.dynamic_friction) < 1e-6
                                and abs(cm.restitution - lms.restitution) < 1e-6
                            ),
                            None,
                        )
                        entry_l = {
                            "name": f"robot0.{lname}",
                            "kind": "robot_link",
                            "asset_id": None,
                            "material_class": matched_cls,
                            "has_override": False,
                            "static_friction": float(em_l[0]),
                            "dynamic_friction": float(em_l[1]),
                            "restitution": float(em_l[2]),
                            "fmode": lms.friction_combine_mode,
                            "rmode": lms.restitution_combine_mode,
                        }
                        partner_sides.append(entry_l)
                        prefix_map.append((f"/World/envs/{ek}/robot0/{lname}/", entry_l))

            def _check_pair_entry(
                sa: dict[str, Any], sb: dict[str, Any], *, ctx_label: str, unbanded: list[str] | None = None
            ) -> dict[str, Any] | None:
                tkey, trow, tkind = _lookup_rvn_target(
                    sa["asset_id"], sa["material_class"], sa["has_override"],
                    sb["asset_id"], sb["material_class"], sb["has_override"],
                )
                if trow is None:
                    unb = unbanded_scene_pairs if unbanded is None else unbanded
                    if tkey and tkey not in unb:
                        unb.append(tkey)
                    return None
                eff_s, mode_s = combine_values(sa["static_friction"], sb["static_friction"], sa["fmode"], sb["fmode"])
                eff_k, _ = combine_values(sa["dynamic_friction"], sb["dynamic_friction"], sa["fmode"], sb["fmode"])
                eff_r, _ = combine_values(sa["restitution"], sb["restitution"], sa["rmode"], sb["rmode"])

                nom_key = "class_pair_nominal" if tkind == "asset_override" else "nominal"
                del_key = "delta_vs_class_nominal" if tkind == "asset_override" else "delta"
                ts_real = float(trow["mu_static"]["realised"])
                tk_real = float(trow["mu_kinetic"]["realised"])
                s_band = trow["mu_static"]["band"]
                k_band = trow["mu_kinetic"]["band"]

                ok_s = abs(eff_s - ts_real) <= 1e-4 and (s_band[0] - 1e-6 <= eff_s <= s_band[1] + 1e-6)
                ok_k = abs(eff_k - tk_real) <= 1e-4 and (k_band[0] - 1e-6 <= eff_k <= k_band[1] + 1e-6)
                ok_r = True
                tr_real = None
                if "restitution" in trow:
                    tr_real = float(trow["restitution"]["realised"])
                    ok_r = abs(eff_r - tr_real) <= 1e-4
                if not (ok_s and ok_k and ok_r):
                    failures.append(
                        f"Realised pair readback mismatch at {ctx_label} ({tkey}): "
                        f"engine=({eff_s:.6f}, {eff_k:.6f}, {eff_r:.6f}) vs table_realised=({ts_real:.6f}, {tk_real:.6f}, {tr_real})"
                    )
                return {
                    "context": ctx_label,
                    "table_key": tkey,
                    "kind": tkind,
                    "combine_mode": mode_s,
                    "engine_realised": {"mu_static": round(eff_s, 6), "mu_kinetic": round(eff_k, 6), "restitution": round(eff_r, 6)},
                    "table_realised": {"mu_static": ts_real, "mu_kinetic": tk_real, "restitution": tr_real},
                    "nominal": {
                        "mu_static": trow["mu_static"][nom_key],
                        "mu_kinetic": trow["mu_kinetic"][nom_key],
                    },
                    "delta_realised_vs_nominal": {
                        "mu_static": trow["mu_static"][del_key],
                        "mu_kinetic": trow["mu_kinetic"][del_key],
                    },
                    "band": {"mu_static": s_band, "mu_kinetic": k_band},
                    "matched_realised": bool(ok_s and ok_k and ok_r),
                }

            for idx_a, ra in enumerate(rigid_sides):
                for pb_side in partner_sides:
                    crec = _check_pair_entry(ra, pb_side, ctx_label=f"{ek}:{ra['name']}x{pb_side['name']}")
                    if crec is not None and env_i == 0:
                        checked_scene_pairs.append(crec)
                for rb in rigid_sides[idx_a + 1 :]:
                    crec = _check_pair_entry(ra, rb, ctx_label=f"{ek}:{ra['name']}x{rb['name']}")
                    if crec is not None and env_i == 0:
                        checked_scene_pairs.append(crec)
            # Robot links on static colliders and on the other arm's links (kept apart so the rigid-object pair count
            # stays comparable with v1).
            for la, sb in robot_link_static_pairs(partner_sides):
                crec = _check_pair_entry(la, sb, ctx_label=f"{ek}:{la['name']}x{sb['name']}", unbanded=unbanded_link_static_pairs)
                if crec is not None and env_i == 0:
                    checked_link_static_pairs.append(crec)
            for la, lb in robot_link_pairs(partner_sides):
                lb_name = lb["name"].replace("robot0.", "robot1.", 1)
                crec = _check_pair_entry(la, lb, ctx_label=f"{ek}:{la['name']}x{lb_name}", unbanded=unbanded_link_link_pairs)
                if crec is not None and env_i == 0:
                    checked_link_link_pairs.append(crec)

            if resting_contacts is not None:
                prefix_sorted = sorted(prefix_map, key=lambda kv: len(kv[0]), reverse=True)
                for rc in resting_contacts:
                    c0, c1 = str(rc["collider0"]), str(rc["collider1"])
                    if not (c0.startswith(f"/World/envs/{ek}/") or c1.startswith(f"/World/envs/{ek}/")):
                        continue
                    s0 = next((ent for pfx, ent in prefix_sorted if c0 == pfx or c0.startswith(pfx + "/") or c0.startswith(pfx)), None)
                    s1 = next((ent for pfx, ent in prefix_sorted if c1 == pfx or c1.startswith(pfx + "/") or c1.startswith(pfx)), None)
                    if s0 is not None and s1 is not None:
                        rcrec = _check_pair_entry(s0, s1, ctx_label=f"resting_contact:{c0}x{c1}")
                        if rcrec is not None:
                            checked_resting_contacts.append(rcrec)

        realised_pair_readback = {
            "checked_scene_pair_count_env0": len(checked_scene_pairs),
            "checked_resting_contact_count": len(checked_resting_contacts),
            "unbanded_scene_pairs": sorted(unbanded_scene_pairs),
            "scene_pairs_env0": checked_scene_pairs,
            "resting_contacts": checked_resting_contacts,
            "checked_robot_link_static_pair_count_env0": len(checked_link_static_pairs),
            "unbanded_robot_link_static_pairs": sorted(unbanded_link_static_pairs),
            "robot_link_static_pairs_env0": checked_link_static_pairs,
            "checked_robot_link_pair_count_env0": len(checked_link_link_pairs),
            "unbanded_robot_link_pairs": sorted(unbanded_link_link_pairs),
            "robot_link_pairs_env0": checked_link_link_pairs,
        }

    solver_rec, solver_failures = solver_scene_readback(stage, resolver)
    failures.extend(solver_failures)

    out = {
        "physics_resolver": resolver.manifest_summary() if resolver is not None else None,
        "solver_scene": solver_rec,
        "num_envs": int(env.num_envs),
        "device": str(env.device),
        "cpu_contact_report_supported": cpu_contacts_supported,
        "contact_evidence_status": contact_evidence_status,
        "compensations_used": compensations_used,
        "sdf_trimesh_face_index_report_null_contacts": sdf_trimesh_report_null_contacts,
        "realised_pair_readback": realised_pair_readback,
        "resting_contacts": resting_contacts,
        "per_env_objects": per_env_objects,
        "per_env_fixtures": per_env_fixtures,
        "per_env_robots": per_env_robots,
        "canary_contacts": canary_report,
        "assertion_passed": len(failures) == 0,
        "failures": failures,
    }
    if assert_spec and failures:
        raise AssertionError("Physics readback assertion failed:\n  - " + "\n  - ".join(failures))
    return out
