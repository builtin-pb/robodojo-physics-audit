"""Read-only trace of who holds each bottle and how high it is, per action (#109; harness instrumentation only).

Wraps ``env.reward_manager.step`` so that after each action it records, for every bottle, the rise of the oriented
box's lowest point above its first-seen height and which arms satisfy ``gripper_holds_bottle_v2``. It calls the same
functions the v2 tracker uses, with the ledger's jaw-gap floor (v2.2, #177), and writes nothing the evaluator reads.
``summarize`` turns the rows into, per bottle, the arms that held it while it was above the lift threshold, in order,
and the nearest pad-to-bottle distance of any arm over the episode (to show that a bottle was not touched by a gripper).
"""
import numpy as np

from utils.transformer import calc_polygon
from src.eval_client.evaluator_v2 import (
    BOTTLES_V2_LIFT_CLEARANCE_M,
    gripper_gap_m_v2,
    gripper_holds_bottle_v2,
    pad_center_world_v2,
)


def install(env, labels=("bottle0", "bottle1", "bottle2", "bottle3")):
    rm = env.reward_manager
    lm = env.scene_manager.layout_manager
    rows = []
    first_zmin = {}
    orig = rm.step

    def step(env_idx_list=None):
        out = orig(env_idx_list)
        arms = []
        for robot in env.robot_manager.robot_list:
            if robot.type != "target":
                continue
            link = env.robot_manager.get_link_pose(robot, "link6", env_idx_list=[0], is_relative=True)[0]
            q = np.asarray(env.robot_manager.get_end_effector_real_val(robot=robot, env_idx_list=[0])[0], dtype=float).reshape(-1)
            arms.append((robot.arm_name, pad_center_world_v2(link[:3], link[3:7]), gripper_gap_m_v2(q[0], q[1])))
        row = {"a": int(env.take_action_cnt[0]), "b": {}}
        gap_min_m = getattr(getattr(env, "bottle_pickup_ledger_v2", None), "gap_min_m", None)
        for label in labels:
            inst = lm.get_instance_name(label=label, env_idx=0)
            if inst is None:
                continue
            pos, rot = lm.get_instance_pose(inst_name=inst, env_idx=0)
            bbox = np.asarray(lm.get_instance_bbox_vertices(inst_name=inst, env_idx=0), dtype=float).reshape(-1, 3)
            pos = np.asarray(pos, dtype=float).reshape(-1)[:3]
            rot = np.asarray(rot, dtype=float).reshape(-1)[:4]
            _, z_min, _ = calc_polygon(origin_pose=np.concatenate([pos, rot]), origin_bbox_points=bbox)
            first_zmin.setdefault(label, float(z_min))
            holders = [
                name
                for name, pad, gap in arms
                if gripper_holds_bottle_v2(
                    pad_world=pad, gap_m=gap, bottle_pos=pos, bottle_quat_wxyz=rot, local_bbox_vertices=bbox,
                    gap_min_m=gap_min_m,
                )
            ]
            rel = pos  # nearest pad distance to the bottle centre
            near = min(float(np.linalg.norm(np.asarray(pad) - rel)) for _, pad, _ in arms)
            row["b"][label] = {"rise": round(float(z_min) - first_zmin[label], 4), "holders": holders, "pad_dist": round(near, 4)}
        rows.append(row)
        return out

    rm.step = step
    return rows


def summarize(rows, clearance=BOTTLES_V2_LIFT_CLEARANCE_M):
    out = {}
    labels = sorted({k for r in rows for k in r["b"]})
    for label in labels:
        seq = []  # (arm, first_action, last_action) while held above the lift threshold
        max_rise = 0.0
        min_pad = 9.0
        for r in rows:
            d = r["b"].get(label)
            if d is None:
                continue
            max_rise = max(max_rise, d["rise"])
            min_pad = min(min_pad, d["pad_dist"])
            if d["rise"] > clearance:
                for arm in d["holders"]:
                    if seq and seq[-1][0] == arm and seq[-1][2] >= r["a"] - 2:
                        seq[-1][2] = r["a"]
                    else:
                        seq.append([arm, r["a"], r["a"]])
        out[label] = {"max_rise_m": round(max_rise, 4), "nearest_pad_centre_m": round(min_pad, 4), "held_above_threshold": seq}
    return out
