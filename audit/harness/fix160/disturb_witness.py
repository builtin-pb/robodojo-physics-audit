"""#160 disturbance witness: a bottle held still in the corrected (D3) gripper, then small declared disturbances.

Policy `disturb` of `audit/harness/run_harness.py`, through RoboDojo's own `TASK_ENV.take_action` path, robot0 (the
left arm, the receiver in #160's handovers). Per trial (one bottle under one gripper variant):

1. **Place.** The left arm moves (30 actions, gripper open) to the hand pose the handover diagnosis logged at the end of
   the giver's retreat (`audit/out/fix_160/witness_inputs.json`, from `audit/tools/fix160_poses.py`). The bottle is
   placed in the open hand at the pose it had in link6's frame there, and re-placed there with zero velocity before
   every physics step (pinned) while the gripper closes from fully open (u = 0), until the fingers are within
   ``release_gap_mm`` (1 mm) of the logged hold opening; then it is free (re-placing it into a closing pad injects
   energy: smoke run `f160_b0_smoke`). So the pads close on the same spots of the same hull as in the episode. With
   `support_close` (default on) the bottle's gravity is off (`physxRigidBody:disableGravity`, read back from its
   vertical velocity) from its placement until 10 actions after both pads touch, as the giver holds it in the episode;
   then gravity is on for the rest of the trial. (Batch 1 closed on a free-falling bottle, and the close's chatter lost
   it in several variants before any disturbance.)
2. **Quiet hold**, 10 + 25 actions with nothing changing. The steady grip F_ss is the mean summed normal force of both
   pads on the bottle over the last 50 steps.
3. **Taps.** 1 N on the bottle's centre of mass (`IPhysxSimulation.apply_force_at_pos`, mode Force, before each of 10
   physics steps = one action), along the closing axis (toward link7), along the bottle's axis, and straight down;
   each followed by 15 quiet actions.
4. **Jerks.** One action whose arm target puts link6 1 cm (then 2 cm) along the closing axis, or 1 cm up, followed by
   one action back to the hold pose (RoboDojo re-aims at every action boundary, #160's G8), then 13 quiet actions.
5. **Swing.** The logged carry (`left_carry_high` and `left_to_dustbin`): one arm target per action, the IK solution
   of link6's logged pose at that action's last step (seeded by the previous solution), then 10 quiet actions.

Per physics step: joint7/joint8 position, joint7 velocity and engine target, the D3 law's state (frozen, force), both
pads' normal force on the bottle (PhysX contact report), the bottle's rotation and centre-of-mass slip in link6's frame
since the quiet hold, link6's position. Per phase: steps whose summed pad force is below 20% of F_ss (and below 32 N,
#160's absolute threshold), the largest step force, the rotation and slip, link6's peak acceleration (second
differences), and the loss (summed pad force under 1 N for 20 ms); after a loss the trial's remaining phases are skipped.

Gripper variants (runtime, declared in each trial's record and read back from the engine):
- `armature_kg`: [joint7, joint8] armature (D3: [23.524, 0]);
- `joint7_effort_limit_n`: joint7's engine effort limit only, the D3 law still freezing at the spec's limit (arx5-sdk's
  force law: the motor pushes past the freeze threshold up to this limit);
- `retarget_frac`: diagnostic, not a candidate law: once the grip has settled (after `settle`, before `quiet`), the
  target is moved to ``q - frac * limit / K`` and held while the command keeps closing, so the same grasp is held
  inside the drive's linear range at ``frac`` of the cap. (A first version latched this at the D3 law's first freeze;
  that falls mid-bounce in the close and tested the close transient instead: `f160_b1_runtime`.)
- `latch_at_freeze_frac`: a candidate law, from the start of the trial: when the D3 law first freezes while closing,
  the target is set to ``q - frac * threshold / K`` (the threshold being the spec's limit, 2 x 80 N on joint7) and held
  while the command keeps closing; with `joint7_effort_limit_n` at the motor's peak, the steady grip is the threshold and
  the drive keeps its stiffness and damping, with headroom to the peak, around it.
After each trial the bottle is put back where it settled at the run's start (a dropped bottle fouled the next trial in
`f160_b2_runtime`).
The witness lifts RoboDojo's episode step limit (`TASK_ENV.step_lim`, 700 actions) for its own episode: one process runs
many trials.
Scene and pad-material contrasts are pre-parse harness flags (`--scene_physx_extra`, `--pad_compliance`).
Unknown configuration or variant keys are rejected before the run (`witness_cfg.validate_cfg`), the drive read back at
the start must be the spec's (joint7's effort limit and armature), and every variant's overrides are compared with the
engine's readback after they are applied; a mismatch stops the run (#163 review, problem 5).
"""
from __future__ import annotations

import dataclasses
import gzip
import json
import math
import os
import sys

import numpy as np
import torch

from witness_cfg import readback_mismatches, validate_cfg

PH_NAMES = ("close", "settle", "quiet", "tap_close", "tap_axis", "tap_down", "jerk_close_1cm", "jerk_close_2cm",
            "jerk_up_1cm", "swing", "swing_rest")


def _np(x):
    return np.asarray(x.cpu() if hasattr(x, "cpu") else x, dtype=float)


def _r(x, n=4):
    return round(float(x), n)


def quat_to_mat(q):
    w, x, y, z = np.asarray(q, dtype=float).reshape(-1)[:4] / np.linalg.norm(np.asarray(q, dtype=float).reshape(-1)[:4])
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.asarray(q) / np.linalg.norm(q)
    return q if q[0] >= 0 else -q


class _RetargetSpec:
    """Diagnostic wrapper of a D3 ``GripperSpec``: the D3 law until ``latch(target)``; then that target while the
    command keeps closing (see the module docstring)."""

    def __init__(self, spec, frac, at_freeze=False):
        self._spec, self.frac, self._latched, self.at_freeze = spec, float(frac), None, bool(at_freeze)

    def __getattr__(self, name):
        return getattr(self._spec, name)

    def latch(self, target):
        self._latched = float(target)

    @property
    def latched(self):
        return self._latched

    def command_step(self, prev_target, command, q, qdot, dt, upper):
        target, frozen, force = self._spec.command_step(prev_target, command, q, qdot, dt, upper)
        if self._latched is not None:
            if command < self._latched:  # still asked to close: hold the moved target
                return self._latched, True, force
            self._latched = None
            return target, frozen, force
        if self.at_freeze and frozen and force < 0.0:  # first closing freeze (q grows opening)
            self._latched = q - self.frac * getattr(self._spec, "freeze_threshold", self._spec.drive_effort_limit) / self._spec.drive_stiffness
            return self._latched, True, force
        return target, frozen, force


def run_disturb(TASK_ENV, REPO, POLICY_LOG, SETTINGS, cfg, workdir):
    sys.path.insert(0, os.path.join(REPO, "audit/probes"))
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    import carb
    import omni.usd
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdPhysics, UsdShade
    from x5_ik import X5
    import p32_gripper as _p32
    from p32_gripper import finger_contact_forces, read_drive_and_pos

    SETTINGS.set_bool("/physics/disableContactProcessing", False)
    dt = float(getattr(TASK_ENV, "dt", 0.004))
    _p32.DT = dt
    spa = int(TASK_ENV.obs_manager.collect_interval)
    with open(os.path.join(REPO, cfg.get("inputs", "audit/out/fix_160/witness_inputs.json"))) as f:
        inputs = json.load(f)["inputs"]
    validate_cfg(cfg, input_keys=inputs)
    log = POLICY_LOG.setdefault("disturb", {"cfg": cfg, "dt": dt, "steps_per_action": spa, "trials": []})
    raw_trials = []

    rm = TASK_ENV.robot_manager
    lm = TASK_ENV.scene_manager.layout_manager
    arm_i = 0
    robot, art = rm.robot_list[arm_i], rm.robot_key[arm_i]
    spec0 = robot.gripper_spec
    if spec0 is None:
        raise RuntimeError("policy disturb needs the physics fix 'gripper'")
    names = list(robot.gripper_joints_name)
    j7 = robot.gripper_joint_indices[names.index(spec0.driven_joint)]
    j8 = robot.gripper_joint_indices[names.index(spec0.mimic_joint)]
    base_rb = read_drive_and_pos(rm, arm_i)
    base_arm = list(base_rb["armature_eng"])
    base_fmax = float(base_rb["Fmax_eng"][0])
    log["drive_readback_at_start"] = base_rb
    bad = readback_mismatches({}, base_rb, [spec0.drive_armature, 0.0], spec0.drive_effort_limit)
    if bad:  # the table's drive must be what the engine runs before any variant is compared against it
        raise RuntimeError(f"disturb: start readback differs from the gripper spec: {bad}")
    bnames = list(art.body_names)
    i6 = bnames.index("link6")
    root = np.asarray(robot.default_root_pos, dtype=float)
    Rb = quat_to_mat(robot.default_root_rot)
    kin = X5(os.path.join(REPO, "Assets/Robots/x5/X5A.urdf"))
    stage = omni.usd.get_context().get_stage()
    stage_id = omni.usd.get_context().get_stage_id()
    sim_if = get_physx_simulation_interface()

    # pad materials as authored (compliance contrast readback)
    pads = {}
    for link in ("link7", "link8"):
        lp = stage.GetPrimAtPath(f"/World/envs/env_0/robot{arm_i}/{link}")
        for p in Usd.PrimRange(lp, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)) if lp and lp.IsValid() else []:
            if p.HasAPI(UsdPhysics.CollisionAPI):
                m, _ = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial("physics")
                if m and m.GetPrim().IsValid():
                    mp = m.GetPrim()
                    pads[str(p.GetPath())] = {"material": str(mp.GetPath()), **{
                        a: (mp.GetAttribute(a).Get() if mp.GetAttribute(a) and mp.GetAttribute(a).IsValid() else None)
                        for a in ("physics:staticFriction", "physics:dynamicFriction", "physics:restitution",
                                  "physxMaterial:compliantContactStiffness", "physxMaterial:compliantContactDamping")}}
    log["pad_materials_usd"] = pads
    scn = [p for p in stage.Traverse() if p.GetTypeName() == "PhysicsScene"]
    log["scene_usd"] = {str(p.GetPath()): {a: (p.GetAttribute(a).Get() if p.GetAttribute(a) and p.GetAttribute(a).IsValid() else None)
                                           for a in ("physxScene:solveArticulationContactLast", "physxScene:minVelocityIterationCount",
                                                     "physxScene:minPositionIterationCount", "physxScene:enableStabilization")}
                        for p in scn}

    obs = TASK_ENV.get_obs()
    base_act = {
        "left_arm_joint_state": np.asarray(obs["state"]["left_arm_joint_state"], dtype=float),
        "left_ee_joint_state": [1.0],
        "right_arm_joint_state": np.asarray(obs["state"]["right_arm_joint_state"], dtype=float),
        "right_ee_joint_state": [1.0],
    }
    q_now = [np.asarray(obs["state"]["left_arm_joint_state"], dtype=float)]

    def l6_pose():
        p = _np(art.data.body_link_pose_w[0, i6])
        return p[:3], quat_to_mat(p[3:7])

    def to_base(p_w, R_w):
        return Rb.T @ (np.asarray(p_w) - root), Rb.T @ R_w

    # --- per-step state shared with the wrapped step ---
    st = {"bottle": None, "pin": None, "force": None, "phase": "idle", "rec": None, "ref": None, "low1": 0, "lost": None,
          "release_q7_m": 0.0}
    orig_step = TASK_ENV.step

    def bottle_pose(obj_label):
        p, q = lm.get_instance_pose(env_idx=0, label=obj_label)
        return _np(p).reshape(-1)[:3], _np(q).reshape(-1)[:4]

    def set_bottle(obj, p, q):
        obj.set_world_pose(position=torch.tensor([float(x) for x in p], dtype=torch.float32),
                           orientation=torch.tensor([float(x) for x in q], dtype=torch.float32))
        obj.set_linear_velocity(torch.zeros(3))
        obj.set_angular_velocity(torch.zeros(3))

    def logged_step(env_idx_list, decimation=1):
        b = st["bottle"]
        if b is not None and st["pin"] is not None:
            # free the bottle while the pads are still ``release_gap_mm`` away, so no placement pushes it into a pad
            if float(art.data.joint_pos[0, j7]) < st["release_q7_m"]:
                st["pin"] = None
            else:
                set_bottle(b["obj"], *st["pin"])
        if b is not None and st["force"] is not None:
            bp, bq = bottle_pose(b["label"])
            com = bp + quat_to_mat(bq) @ b["com_local"]
            sim_if.apply_force_at_pos(stage_id, b["body_id"], carb.Float3(*[float(x) for x in st["force"]]),
                                      carb.Float3(*[float(x) for x in com]), "Force")
        orig_step(env_idx_list=env_idx_list, decimation=decimation)
        if b is None or st["rec"] is None:
            return
        rb = read_drive_and_pos(rm, arm_i)
        cf = finger_contact_forces(b["prim"], arm_i)
        gs = (getattr(robot, "gripper_step_state", None) or [None])[0] or {}
        hp, hR = l6_pose()
        bp, bq = bottle_pose(b["label"])
        bR = quat_to_mat(bq)
        rel_R, rel_c = hR.T @ bR, hR.T @ (bp + bR @ b["com_local"] - hp)
        rot = slip = None
        if st["ref"] is not None:
            Rd = st["ref"][0].T @ rel_R
            rot = _r(math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Rd) - 1.0) / 2.0)))), 4)
            slip = _r(1000.0 * float(np.linalg.norm(rel_c - st["ref"][1])), 4)
        n7, n8 = cf["normal_N"]["link7"], cf["normal_N"]["link8"]
        st["rec"].append([PH_NAMES.index(st["phase"]), _r(rb["pos_eng_m"][0] * 1e3, 4), _r(rb["pos_eng_m"][1] * 1e3, 4),
                          _r(rb["vel_eng_mps"][0], 5), _r(rb["tgt_eng_m"][0] * 1e3, 4), _r(gs.get("drive_force_N", 0.0), 2),
                          int(bool(gs.get("frozen", False))), _r(n7, 3), _r(n8, 3), rot, slip,
                          _r(hp[0], 6), _r(hp[1], 6), _r(hp[2], 6), _r(cf["max_penetration_mm"], 4),
                          1 if st["force"] is not None else 0, _r(_np(b["obj"].get_linear_velocity()).reshape(-1)[2], 5)])
        st["low1"] = st["low1"] + 1 if (n7 + n8) < 1.0 else 0
        if st["lost"] is None and st["phase"] != "close" and st["low1"] >= max(1, int(round(0.02 / dt))):
            st["lost"] = {"phase": st["phase"], "step": len(st["rec"]) - st["low1"]}  # index of the first step under 1 N

    def send(q_arm, u):
        a = dict(base_act)
        a["left_arm_joint_state"] = np.asarray(q_arm, dtype=float)
        a["left_ee_joint_state"] = [float(u)]
        TASK_ENV.take_action(a)
        q_now[0] = np.asarray(q_arm, dtype=float)

    def move(q1, n, u, phase="idle"):
        st["phase"] = phase
        q0 = q_now[0].copy()
        for k in range(n):
            s = (k + 1) / n
            s = 3 * s * s - 2 * s * s * s
            send((1 - s) * q0 + s * np.asarray(q1, dtype=float), u)

    def run_phase(name, n_actions, q_arm, force=None, n_force_steps=0):
        """``n_actions`` actions at ``q_arm`` (gripper closed); a force on the first ``n_force_steps`` physics steps."""
        if st["lost"] is not None:
            return False
        st["phase"] = name
        for k in range(n_actions):
            if force is not None:
                st["force"] = force if k * spa < n_force_steps else None
            send(q_arm, 0.0)
            if force is not None and (k + 1) * spa >= n_force_steps:
                st["force"] = None
            if st["lost"] is not None:
                break
        st["force"] = None
        return st["lost"] is None

    def apply_variant(v):
        a7, a8 = v.get("armature_kg", base_arm)
        art.write_joint_armature_to_sim(float(a7), joint_ids=[j7])
        art.write_joint_armature_to_sim(float(a8), joint_ids=[j8])
        art.write_joint_effort_limit_to_sim(float(v.get("joint7_effort_limit_n", base_fmax)), joint_ids=[j7])
        if v.get("latch_at_freeze_frac"):
            robot.gripper_spec = _RetargetSpec(spec0, v["latch_at_freeze_frac"], at_freeze=True)
        elif v.get("retarget_frac"):
            robot.gripper_spec = _RetargetSpec(spec0, v["retarget_frac"])
        else:
            robot.gripper_spec = spec0
        rb = read_drive_and_pos(rm, arm_i)
        bad = readback_mismatches(v, rb, base_arm, base_fmax)
        if bad:
            raise RuntimeError(f"disturb: variant {v.get('name')!r} did not reach the engine: {bad}")
        return rb

    labels = lm.get_labels_by_prefix(0, "bottle")
    home = {l: bottle_pose(l) for l in labels}
    inst_of = {lm.get_instance_name(0, l): l for l in labels}
    proto = {"quiet_actions": 25, "tap_N": 1.0, "tap_steps": spa, "after_actions": 15, "jerk_after_actions": 13,
             "release_gap_mm": 1.0, "support_close": True}
    proto.update(cfg.get("protocol") or {})
    log["protocol"] = proto

    log["step_lim_orig"] = int(TASK_ENV.step_lim)
    TASK_ENV.step_lim = 10 ** 7  # many trials in one episode (declared in the module docstring)
    TASK_ENV.step = logged_step
    try:
        for v in cfg["variants"]:
            for key in v.get("bottles", cfg["bottles"]):
                inp = inputs[key]
                # instance names end in a load counter (bottle_22_19 in a four-layout run, bottle_22_1 alone): match the asset
                found = [i for i in inst_of if i.startswith(f"bottle_{int(inp['asset'])}_")]
                if len(found) != 1:
                    raise RuntimeError(f"bottle asset {inp['asset']} not unique in this layout ({sorted(inst_of)})")
                label = inst_of[found[0]]
                obj = lm.get_scene_object(0, found[0])
                prim = str(obj.prim_path)
                b = {"label": label, "obj": obj, "prim": prim, "com_local": np.asarray(inp["bottle_com_local"], dtype=float),
                     "body_id": PhysicsSchemaTools.sdfPathToInt(prim)}
                trial = {"variant": v, "bottle": key, "asset": inp["asset"], "inst": found[0], "label": label}
                st.update(bottle=None, pin=None, force=None, rec=None, ref=None, low1=0, lost=None)
                # 1. arm to the logged hold pose, gripper open; variant applied with the gripper open and empty
                L = np.asarray(inp["l6_world"], dtype=float)
                p_b, R_b = to_base(L[:3], quat_to_mat(L[3:]))
                q_hold, ep, er = kin.ik(p_b, R_b, q_now[0]) if cfg.get("ik_seed_current", True) else kin.ik_best(p_b, R_b)
                if ep > 1e-3 or er > 0.5:
                    q_hold, ep, er = kin.ik_best(p_b, R_b)
                trial["hold_ik_err_mm_deg"] = [_r(ep * 1e3, 3), _r(er, 3)]
                move(q_hold, 30, 1.0)
                for _ in range(10):
                    send(q_hold, 1.0)
                trial["drive_readback"] = apply_variant(v)
                hp, hR = l6_pose()
                trial["l6_hold_err_mm"] = _r(1e3 * np.linalg.norm(hp - L[:3]), 3)
                rel = np.asarray(inp["bottle_in_l6"], dtype=float)
                pin_p = hp + hR @ rel[:3]
                pin_q = mat_to_quat(hR @ quat_to_mat(rel[3:]))
                gattr = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(prim)).CreateDisableGravityAttr()
                trial["gravity_attr_before"] = gattr.Get()
                if proto["support_close"]:
                    gattr.Set(True)
                st["bottle"], st["pin"], st["rec"] = b, (pin_p, pin_q), []
                st["release_q7_m"] = (inp["q7_q8_mm"][0] + float(proto["release_gap_mm"])) / 1000.0
                # 2. close on the pinned bottle, release once both pads press it
                st["phase"] = "close"
                contact_step = None
                for k in range(60):
                    send(q_hold, 0.0)
                    if not st["rec"]:
                        raise RuntimeError("take_action ran no physics step (episode limit or end_flag)")
                    r = st["rec"][-1]
                    if r[7] > 1.0 and r[8] > 1.0:
                        contact_step = len(st["rec"])
                        break
                st["pin"] = None
                trial["close_steps_to_bilateral_contact"] = contact_step
                trial["pin_released_at_q7_mm"] = _r(st["release_q7_m"] * 1e3, 4)
                ok = run_phase("settle", 10, q_hold)
                if proto["support_close"]:
                    vz_off = [r[16] for r in st["rec"] if r[0] == 0 and r[7] < 0.5 and r[8] < 0.5][-3:]
                    gattr.Set(False)
                    st["phase"] = "settle"
                    for _ in range(5):
                        send(q_hold, 0.0)
                    trial["support_readback_vz_m_s"] = {"gravity_off_free_steps": vz_off}
                if isinstance(robot.gripper_spec, _RetargetSpec) and not robot.gripper_spec.at_freeze:
                    q7 = float(art.data.joint_pos[0, j7])
                    tgt = q7 - robot.gripper_spec.frac * getattr(spec0, "freeze_threshold", spec0.drive_effort_limit) / spec0.drive_stiffness
                    robot.gripper_spec.latch(tgt)
                    trial["retarget_mm"] = [_r(q7 * 1e3, 4), _r(tgt * 1e3, 4)]
                hp, hR = l6_pose()
                bp, bq = bottle_pose(label)
                bR = quat_to_mat(bq)
                st["ref"] = (hR.T @ bR, hR.T @ (bp + bR @ b["com_local"] - hp))
                ok = ok and run_phase("quiet", proto["quiet_actions"], q_hold)
                # 3. taps
                ax_close = hR[:, 1] / np.linalg.norm(hR[:, 1])  # link6 +y: toward link7 (joint7 origin at +y)
                ax_bottle = bR[:, 2]
                for name, d in (("tap_close", ax_close), ("tap_axis", ax_bottle), ("tap_down", np.array([0.0, 0.0, -1.0]))):
                    ok = ok and run_phase(name, 1 + proto["after_actions"], q_hold, force=proto["tap_N"] * d, n_force_steps=proto["tap_steps"])
                # 4. jerks: one action displaced, one back, then quiet
                for name, dv in (("jerk_close_1cm", 0.01 * ax_close), ("jerk_close_2cm", 0.02 * ax_close), ("jerk_up_1cm", np.array([0.0, 0.0, 0.01]))):
                    if not ok:
                        break
                    pj, Rj = to_base(L[:3] + dv, quat_to_mat(L[3:]))
                    qj, _, _ = kin.ik(pj, Rj, q_hold)
                    st["phase"] = name
                    send(qj, 0.0)
                    ok = st["lost"] is None and run_phase(name, 1 + proto["jerk_after_actions"], q_hold)
                # 5. the logged carry swing
                if ok:
                    qs, qprev, ik_err = [], q_hold, []
                    for s in inp["swing"]:
                        P = np.asarray(s["l6_world"], dtype=float)
                        ps, Rs = to_base(P[:3], quat_to_mat(P[3:]))
                        qk, e1, e2 = kin.ik(ps, Rs, qprev)
                        qs.append(qk)
                        ik_err.append(max(e1 * 1e3, 0.0))
                        qprev = qk
                    trial["swing_ik_err_mm_max"] = _r(max(ik_err), 3)
                    st["phase"] = "swing"
                    for qk in qs:
                        send(qk, 0.0)
                        if st["lost"] is not None:
                            break
                    ok = st["lost"] is None and run_phase("swing_rest", 10, qs[-1])
                trial["lost"] = st["lost"]
                trial["summary"] = summarize(st["rec"], dt, spa)
                raw_trials.append({"variant": v.get("name"), "bottle": key, "columns": REC_COLUMNS, "rec": st["rec"]})
                log["trials"].append(trial)
                # reset: open, drop the bottle out of the hand, restore D3
                st.update(bottle=None, pin=None, force=None, rec=None)
                for _ in range(12):
                    send(q_now[0], 1.0)
                gattr.Set(False)
                set_bottle(obj, *home[label])
                for _ in range(3):
                    send(q_now[0], 1.0)
                apply_variant({})
                print("HARNESS disturb trial", v.get("name"), key, "lost", trial["lost"], "low20",
                      {k: x.get("low20") for k, x in trial["summary"]["phases"].items()}, flush=True)
    finally:
        TASK_ENV.step = orig_step
        robot.gripper_spec = spec0
        path = os.path.join(workdir, "disturb_trials.json.gz")
        with gzip.open(path, "wt") as f:
            json.dump(raw_trials, f, separators=(",", ":"))
        log["raw_file"] = path


REC_COLUMNS = ["phase", "q7_mm", "q8_mm", "v7_m_s", "tgt7_mm", "d3_force_N", "frozen", "n7_N", "n8_N", "rot_deg",
               "slip_mm", "l6_x", "l6_y", "l6_z", "pen_mm", "force_on", "bottle_vz_m_s"]


def summarize(rec, dt, spa):
    """Per phase: low-force steps against 20% of the quiet hold's steady grip (and 32 N), extremes, motion, hand accel."""
    if not rec:
        return {}
    A = np.array([[np.nan if x is None else x for x in r] for r in rec], dtype=float)
    ph = A[:, 0].astype(int)
    fsum = A[:, 7] + A[:, 8]
    quiet = np.where(ph == PH_NAMES.index("quiet"))[0]
    f_ss = float(fsum[quiet[-50:]].mean()) if len(quiet) >= 50 else float("nan")
    pos = A[:, 11:14]
    acc = np.full(len(A), np.nan)
    if len(A) >= 3:
        acc[1:-1] = np.linalg.norm(pos[2:] - 2 * pos[1:-1] + pos[:-2], axis=1) / dt ** 2
    out = {"F_ss_N": _r(f_ss, 3), "quiet_cv": _r(float(fsum[quiet[-50:]].std() / max(f_ss, 1e-9)), 5) if len(quiet) >= 50 else None,
           "quiet_lead_mm": _r(float((A[quiet[-50:], 1] - A[quiet[-50:], 4]).mean()), 4) if len(quiet) >= 50 else None,
           "quiet_drive_force_est_N": _r(float(A[quiet[-50:], 5].mean()), 2) if len(quiet) >= 50 else None,
           "quiet_pen_mm": _r(float(A[quiet[-50:], 14].mean()), 4) if len(quiet) >= 50 else None,
           "phases": {}}
    for i, name in enumerate(PH_NAMES):
        ix = np.where(ph == i)[0]
        if not len(ix):
            continue
        f = fsum[ix]
        d = {"steps": int(len(ix)), "low20": int((f < 0.2 * f_ss).sum()) if f_ss == f_ss else None,
             "low32N": int((f < 32.0).sum()), "f_min_N": _r(f.min(), 2), "f_max_N": _r(f.max(), 2),
             "f_mean_N": _r(f.mean(), 2), "rot_max_deg": _r(np.nanmax(A[ix, 9]), 3) if np.isfinite(A[ix, 9]).any() else None,
             "slip_max_mm": _r(np.nanmax(A[ix, 10]), 3) if np.isfinite(A[ix, 10]).any() else None,
             "l6_acc_peak_m_s2": _r(np.nanmax(acc[ix]), 2) if np.isfinite(acc[ix]).any() else None,
             "frozen_steps": int(A[ix, 6].sum()), "q7_range_mm": [_r(A[ix, 1].min(), 4), _r(A[ix, 1].max(), 4)]}
        on = ix[A[ix, 15] > 0]
        if len(on):  # tap readback: link7 minus link8 normal force while the tap is on, against the 10 steps before it
            pre = np.arange(max(0, on[0] - 10), on[0])
            d["tap_n7_minus_n8_shift_N"] = _r(float((A[on, 7] - A[on, 8]).mean() - (A[pre, 7] - A[pre, 8]).mean()), 3)
        out["phases"][name] = d
    return out
