"""G4 variant of the frozen cover_blocks script: the cup's cover lowering point shifted by a fixed lateral offset
(`--policy script_cover_blocks_offset --cb_offset_mm DX,DY`; a declared policy change).

The frozen script (`audit/harness/script_cover_blocks.py`, tag `script-cover-blocks-v1`) is imported and run
unchanged. It has no parameter for the lowering point, so this wrapper gives it a subclass of its controller
(`script_x5.DualX5`) whose `line` adds (DX, DY) to the end point of each cover move's carry and to both points of its
lower and retreat moves (phases `cover_*:carry`, `cover_*:lower`, `cover_*:retreat`). Nothing else changes: the
grasps, the heights, the uncover moves (the script reads each cup just before it grasps it, so it picks the shifted
cup where it stands) and every privileged read. The script's own log still measures cube displacement from the cube
positions it read at the first action.
"""
from __future__ import annotations

import numpy as np

import script_cover_blocks as _scb
from script_x5 import DualX5


def make_offset_controller(offset_m):
    off = np.array([float(offset_m[0]), float(offset_m[1]), 0.0])

    class OffsetDualX5(DualX5):
        def line(self, i, p0, p1, R0, R1=None, grip=None, n=10, phase=None):
            ph = phase or self.phase
            if ph.startswith("cover_"):
                if ph.endswith(":carry"):
                    p1 = np.asarray(p1, dtype=float) + off
                elif ph.endswith(":lower") or ph.endswith(":retreat"):
                    p0 = np.asarray(p0, dtype=float) + off
                    p1 = np.asarray(p1, dtype=float) + off
            return super().line(i, p0, p1, R0, R1=R1, grip=grip, n=n, phase=phase)

    return OffsetDualX5


def describe_deviation(offset_mm) -> str:
    return (
        f"\n- Policy `script_cover_blocks_offset` (G4 variant, a policy change): the frozen `script_cover_blocks` "
        f"(unchanged file) runs with its controller's straight-line moves shifted by ({offset_mm[0]:g}, {offset_mm[1]:g}) "
        "mm in world x and y at the end of each cover carry and through each cover lower and retreat, so every cup is "
        "lowered off-centre on its cube by that offset; grasps, heights, uncover moves and privileged reads are the "
        "script's own." + _scb.describe_deviation()
    )


def run_offset(env, repo_dir: str, policy_log: dict, offset_mm, max_actions: int = 0) -> None:
    orig = _scb.DualX5
    _scb.DualX5 = make_offset_controller([v / 1000.0 for v in offset_mm])
    try:
        _scb.run_script_cover_blocks(env, repo_dir, policy_log, max_actions=max_actions)
    finally:
        _scb.DualX5 = orig
    policy_log["cover_offset_mm"] = [float(v) for v in offset_mm]
