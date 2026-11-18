"""Clips for the blog (#106 item 5, D4 #100): run audit/harness/run_harness.py unchanged and capture frames from cameras
that this wrapper adds to the stage. On Isaac Sim 6.2.0.1 it applies the four #40 compatibility shims after
SimulationApp starts; on 5.1 (no RTX on Della's driver) use --no_render to keep only the frame-aligned state log.

usage (inside the container): python.sh clip_run.py --clip_spec SPEC.json --frames_dir DIR [--active_gpu N]
           [--render_mode M] [--subframes K] [--no_render] [--replay STATE.json] -- <run_harness.py arguments>

What it adds to the harness run, all declared in the result's `deviations` and in the frames' `index.json`:
- harness-side cameras (UsdGeom.Camera prims under /World/ClipCams, Replicator render products, `rgb` annotator);
- captures: `env.render()` (the harness's own evaluation render: forward, then app.update with playSimulations off), then
  `rep.orchestrator.step(rt_subframes=subframes, delta_time=0.0, pause_timeline=False)` with playSimulations off, then the
  annotators are read on the GPU; every capture checks that no physics step ran during it;
- read-only state at each capture: rigid poses, every articulation's joint positions, reward progress, press counters;
- --replay (fallback route): instead of the policy, set the logged 5.1 state (poses, joints, zero velocities) frame by
  frame, write PhysX's transforms to USD (`update_transformations`, no simulation) and render, without stepping physics.
Frames are triggered by windows in the clip spec, counted from the episode start or from a stress case's start:
  {"case": null|NAME, "unit": "action"|"step", "from": a, "to": b, "stride": s}.
A case's step count starts at the witness function's entry: for `launch` cases the logged series index k is case step
k + pre_steps + 1 (restore, pre_steps, then the launch); for `robot_press` without restore, trace index k is case step k + 1.
The case "drop" is the sweep drop test (`--policy sweep --sweep_drop_m`, audit/harness/popsweep/sweep_settle.py
`run_drop`), which runs before the episode: its step 0 (`case_start`) is the settled state before the lift.
A step frame shows the state after that step; it is rendered at the entry of the next step, after the caller's reads.
`physics_steps_in_episode` in index.json is this wrapper's count, to be checked against the harness's `episode_steps`.
"""
import argparse
import builtins
import faulthandler
import json
import os
import runpy
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--clip_spec", required=True)
ap.add_argument("--frames_dir", required=True)
ap.add_argument("--active_gpu", type=int, default=0, help="renderer GPU (Vulkan index; with all GPUs visible, the physical index)")
ap.add_argument("--physics_gpu", type=int, default=0, help="PhysX CUDA device index within CUDA_VISIBLE_DEVICES")
ap.add_argument("--render_mode", default="RaytracedLighting")
ap.add_argument("--subframes", type=int, default=3)
ap.add_argument("--capture", choices=["app", "rep"], default="app",
                help="app: `subframes` evaluation renders (forward + app.update); rep: one evaluation render, then "
                     "rep.orchestrator.step(rt_subframes=subframes, delta_time=0) (about 2.8 s per frame after physics steps)")
ap.add_argument("--warmup", type=int, default=30, help="renders before the first capture (shader and texture loading)")
ap.add_argument("--no_render", action="store_true")
ap.add_argument("--viewport", default="320x180", help="SimulationApp width x height (the unused viewport texture)")
ap.add_argument("--stack_dump_s", type=int, default=240, help="print every thread's Python stack this often (diagnoses stalls)")
ap.add_argument("--replay", default=None, help="state log (frames index.json of a --no_render 5.1 run) to replay kinematically")
A, rest = ap.parse_known_args()
if rest and rest[0] == "--":
    rest = rest[1:]
HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.join(os.path.dirname(HERE), "run_harness.py")
SPEC = json.load(open(A.clip_spec))
os.makedirs(A.frames_dir, exist_ok=True)
RENDER = not A.no_render
faulthandler.dump_traceback_later(A.stack_dump_s, repeat=True, file=sys.stderr)
T_START = time.perf_counter()


def log(msg):
    print(f"[clip_run {time.perf_counter() - T_START:7.1f}s] {msg}", flush=True)
PHYS = [0]
CAMS = []
FRAMES = []
INFO = {"spec": SPEC, "render_mode": A.render_mode, "subframes": A.subframes, "capture": A.capture, "render": RENDER, "replay": A.replay,
        "harness_args": rest, "frames": FRAMES, "cameras": [], "errors": []}


def _settings_for_render():
    import carb

    s = carb.settings.get_settings()
    s.set("/rtx/rendermode", A.render_mode)
    for k, v in SPEC.get("carb", {}).items():
        s.set(k, v)
    INFO["carb"] = {k: s.get(k) for k in ["/rtx/rendermode", "/rtx/post/aa/op", "/rtx/pathtracing/spp",
                                           "/rtx/pathtracing/totalSpp", "/rtx/pathtracing/optixDenoiser/enabled"]
                    + list(SPEC.get("carb", {}).keys())}


def _shims_62():
    """The four #40 shims (audit/harness/route2_robodojo_render.py), needed only on Isaac Sim 6.x."""
    import omni.physx as _ox

    if not hasattr(_ox, "acquire_physx_interface"):
        _ox.acquire_physx_interface = _ox.get_physx_interface
    import types

    import omni.physics.tensors.api as _api

    if "omni.physics.tensors.impl" not in sys.modules:
        for old, new in (("SoftBodyView", "DeformableBodyView"), ("SoftBodyMaterialView", "DeformableMaterialView")):
            if not hasattr(_api, old):
                setattr(_api, old, getattr(_api, new))
        for n in ("ParticleSystemView", "ParticleClothView", "ParticleMaterialView"):
            if not hasattr(_api, n):
                setattr(_api, n, type(n, (), {}))
        impl = types.ModuleType("omni.physics.tensors.impl")
        impl.api = _api
        sys.modules["omni.physics.tensors.impl"] = impl
        sys.modules["omni.physics.tensors.impl.api"] = _api
    from pxr import PhysxSchema, Tf, UsdPhysics

    try:
        from pxr import OmniUsdPhysicsDeformableSchema as _oupd
    except Exception:
        _oupd = None
    for old, cand in (("PhysxDeformableBodyAPI", "DeformableBodyAPI"), ("PhysxDeformableAPI", "BodyAPI"),
                      ("PhysxDeformableBodyMaterialAPI", "DeformableMaterialAPI")):
        if not hasattr(PhysxSchema, old):
            setattr(PhysxSchema, old, getattr(_oupd, cand, None) or getattr(UsdPhysics, cand, None)
                    or Tf.Type.FindByName("OmniPhysics" + cand))


import isaacsim  # noqa: E402

_RealSimApp = isaacsim.SimulationApp


def _SimApp(launch_config=None, experience=""):
    cfg = dict(launch_config or {})
    if RENDER:
        w, h = (int(x) for x in A.viewport.split("x"))
        cfg.update(active_gpu=A.active_gpu, physics_gpu=A.physics_gpu, multi_gpu=False, width=w, height=h)
    app = _RealSimApp(cfg, experience=experience)
    from isaacsim.core.version import get_version

    INFO["isaac_sim"] = list(get_version())[:3]
    import carb

    INFO["gpus"] = {"active_gpu": A.active_gpu, "physics_gpu": A.physics_gpu, "CUDA_VISIBLE_DEVICES": os.environ.get("CUDA_VISIBLE_DEVICES"),
                    "/physics/cudaDevice": carb.settings.get_settings().get("/physics/cudaDevice"),
                    "/renderer/activeGpu": carb.settings.get_settings().get("/renderer/activeGpu")}
    if str(INFO["isaac_sim"][0]).startswith("6."):
        _shims_62()
        INFO["shims"] = "acquire_physx_interface, omni.physics.tensors.impl.api, PhysxDeformable*API (#40)"
    if RENDER:
        _settings_for_render()
    from omni.physx import get_physx_interface

    INFO["_sub"] = get_physx_interface().subscribe_physics_step_events(lambda dt: PHYS.__setitem__(0, PHYS[0] + 1))
    return app


isaacsim.SimulationApp = _SimApp


# --- cameras and capture ------------------------------------------------------------------------------------------------


def _look_at_matrix(eye, target, up=(0.0, 0.0, 1.0)):
    import numpy as np
    from pxr import Gf

    eye, target, up = (np.asarray(x, dtype=float) for x in (eye, target, up))
    f = target - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    rows = [list(r) + [0.0], list(u) + [0.0], list(-f) + [0.0], list(eye) + [1.0]]
    return Gf.Matrix4d(*[float(x) for row in rows for x in row])


def make_cameras(env):
    import omni.replicator.core as rep
    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    origin = [0.0, 0.0, 0.0]
    try:
        import numpy as np

        o = env.scene_manager.env_origins[0]
        origin = [float(x) for x in np.asarray(o.cpu() if hasattr(o, "cpu") else o).reshape(-1)[:3]]
    except Exception:
        pass
    try:
        rep.orchestrator.set_capture_on_play(False)  # no Replicator captures on timeline ticks; only explicit steps
    except Exception as e:
        INFO["errors"].append(f"set_capture_on_play: {e!r}")
    for c in SPEC["cameras"]:
        path = f"/World/ClipCams/{c['name']}"
        cam = UsdGeom.Camera.Define(stage, path)
        cam.CreateFocalLengthAttr(float(c.get("focal_mm", 24.0)))
        cam.CreateHorizontalApertureAttr(float(c.get("h_aperture_mm", 20.955)))
        w, h = c.get("res", [1280, 720])
        cam.CreateVerticalApertureAttr(float(c.get("h_aperture_mm", 20.955)) * h / w)
        cam.CreateClippingRangeAttr((0.01, 100.0))
        eye = [a + b for a, b in zip(c["eye"], origin)]
        tgt = [a + b for a, b in zip(c["target"], origin)]
        UsdGeom.Xformable(cam.GetPrim()).AddTransformOp().Set(_look_at_matrix(eye, tgt))
        rp = rep.create.render_product(path, (int(w), int(h)))
        annot = rep.AnnotatorRegistry.get_annotator("rgb", device="cuda", do_array_copy=False)  # as RoboDojo's CameraView
        annot.attach(rp.path)
        os.makedirs(os.path.join(A.frames_dir, c["name"]), exist_ok=True)
        CAMS.append({"name": c["name"], "rp": rp, "annot": annot})
        INFO["cameras"].append({"name": c["name"], "prim": path, "eye_world": eye, "target_world": tgt,
                                "focal_mm": float(c.get("focal_mm", 24.0)), "res": [int(w), int(h)]})


def read_state(env):
    import numpy as np

    lm = env.scene_manager.layout_manager
    st = {"obj": {}, "art": {}, "robots": []}
    for label in lm.get_labels_by_prefix(0, ""):
        typ = lm.instance_type_by_env[0].get(lm.get_instance_name(0, label))
        try:
            if typ == "rigid":
                p, q = lm.get_instance_pose(env_idx=0, label=label)
                p = np.asarray(p.cpu() if hasattr(p, "cpu") else p).reshape(-1)[:3]
                q = np.asarray(q.cpu() if hasattr(q, "cpu") else q).reshape(-1)[:4]
                st["obj"][label] = [round(float(v), 6) for v in list(p) + list(q)]
            elif typ == "articulation":
                jp = lm.get_scene_object(0, lm.get_instance_name(0, label)).get_joint_positions()
                st["art"][label] = [round(float(v), 6) for v in np.asarray(jp.cpu() if hasattr(jp, "cpu") else jp).reshape(-1)]
        except Exception as e:
            st.setdefault("err", {})[label] = repr(e)[:200]
    for k in env.robot_manager.robot_key:
        st["robots"].append([round(float(v), 6) for v in np.asarray(k.data.joint_pos[0].cpu()).reshape(-1)])
    rm = getattr(env, "reward_manager", None)
    if rm is not None:
        try:
            st["stages_left"] = len(rm.check_list[0])
            st["score_tiers_done"] = int(rm.score_completed_count[0])
        except Exception:
            pass
        try:
            st["queries"] = [[int(q[1]), int(q[2])] for q in rm.query_list[0]]
        except Exception:
            pass
    st["success_flag"] = bool(env.success[0])
    st["end_flag"] = bool(env.end_flag[0])
    st["action"] = int(env.take_action_cnt[0])
    return st


def render_frame(env, subframes=None):
    """The harness's evaluation render (forward, then app.update with playSimulations off), then a Replicator step that
    renders `subframes` subframes into the annotators without advancing the timeline (delta_time 0) or simulating."""
    import carb
    import omni.replicator.core as rep

    n = int(subframes or A.subframes)
    if A.capture == "app":
        for _ in range(n):
            env.render()
        return
    env.render()
    st = carb.settings.get_settings()
    st.set_bool("/app/player/playSimulations", False)
    try:
        rep.orchestrator.step(rt_subframes=n, pause_timeline=False, delta_time=0.0)
    finally:
        st.set_bool("/app/player/playSimulations", True)


def capture(env, why):
    from PIL import Image

    rec = {"i": len(FRAMES), "why": why, "phys": PHYS[0] - EP["phys0"], "case": EP["case"],
           "case_phys": PHYS[0] - EP["case_phys0"] if EP["case"] else None,
           "case_action": int(env.take_action_cnt[0]) - EP["case_act0"] if EP["case"] else None}
    rec.update(read_state(env))
    if RENDER and CAMS:
        s0 = PHYS[0]
        t0 = time.perf_counter()
        render_frame(env)
        if PHYS[0] != s0:
            INFO["errors"].append(f"physics advanced during capture {rec['i']} ({s0} -> {PHYS[0]})")
            _write_index()
            raise RuntimeError(f"physics advanced during a capture ({s0} -> {PHYS[0]})")
        for c in CAMS:
            img = c["annot"].get_data(device="cuda")
            img = img.numpy() if hasattr(img, "numpy") else img
            if img is None or getattr(img, "size", 0) == 0:
                INFO["errors"].append(f"empty frame {rec['i']} {c['name']}")
                continue
            Image.fromarray(img[..., :3]).save(os.path.join(A.frames_dir, c["name"], f"f_{rec['i']:05d}.jpg"), quality=93)
            rec.setdefault("img_mean", {})[c["name"]] = round(float(img[..., :3].mean()), 3)
        rec["render_s"] = round(time.perf_counter() - t0, 3)
    FRAMES.append(rec)
    if rec["i"] < 3 or rec["i"] % 50 == 0:
        log(f"frame {rec['i']} {why} render_s={rec.get('render_s')}")


EP = {"phys0": 0, "case": None, "case_phys0": 0, "case_act0": 0, "armed": False, "pending": False}


def flush_pending(env):
    if EP["pending"]:
        EP["pending"] = False
        capture(env, "step")


def _in_windows(unit, n_ep, n_case):
    for w in SPEC.get("windows", []):
        if w.get("unit", "action") != unit:
            continue
        if w.get("case") is None:
            n = n_ep
        elif w["case"] == EP["case"]:
            n = n_case
        else:
            continue
        if n is not None and w["from"] <= n <= w["to"] and (n - w["from"]) % int(w.get("stride", 1)) == 0:
            return True
    return False


def install_episode_hooks(env):
    orig_sim_step = env.sim_step
    orig_tab = env.take_action_batch

    def sim_step(render=True):
        # A step frame is captured at the entry of the next physics step (or at the end of the case or episode), so the
        # caller's own reads after each step (contact reports, poses) happen before any render.
        flush_pending(env)
        orig_sim_step(render=render)
        if EP["armed"]:
            n_ep = PHYS[0] - EP["phys0"]
            n_case = PHYS[0] - EP["case_phys0"] if EP["case"] else None
            if _in_windows("step", n_ep, n_case):
                EP["pending"] = True

    def take_action_batch(actions_list, env_idx_list=None):
        orig_tab(actions_list, env_idx_list=env_idx_list)
        flush_pending(env)
        if int(env.take_action_cnt[0]) % 100 == 0:
            log(f"action {int(env.take_action_cnt[0])}, physics steps {PHYS[0] - EP['phys0']}")
        if EP["armed"]:
            n_ep = int(env.take_action_cnt[0])
            n_case = n_ep - EP["case_act0"] if EP["case"] else None
            if _in_windows("action", n_ep, n_case):
                capture(env, "action")

    env.sim_step = sim_step
    env.take_action_batch = take_action_batch
    if any(w.get("case") not in (None, "drop") for w in SPEC.get("windows", [])):
        sys.path.insert(0, os.path.join(os.path.dirname(HERE), "stress_hv"))
        import witness_stress as ws

        for fn in ("case_launch", "case_robot_press", "case_joint_probe", "case_rays"):
            orig = getattr(ws, fn)

            def wrapped(ctx, case, _o=orig):
                EP.update(case=case.get("name"), case_phys0=PHYS[0], case_act0=int(env.take_action_cnt[0]))
                if _in_windows("step", None, 0) or _in_windows("action", None, 0):
                    capture(env, "case_start")
                try:
                    return _o(ctx, case)
                finally:
                    flush_pending(env)
                    EP["case"] = None

            setattr(ws, fn, wrapped)
    return orig_sim_step, orig_tab


def install_drop_hook():
    """Log state through the sweep drop test (case "drop"), which the harness runs after the settle and before the
    episode: the settled state at its entry (`case_start`, before the lift), then step frames per the "drop" windows.
    Steps are counted as `env.sim_step` calls (`case_phys`), not physics step events, since a scene whose GPU state is
    corrupted may not emit them; the event count is kept as `case_phys_events`."""
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), "popsweep"))
    import sweep_settle

    orig = sweep_settle.run_drop

    def run_drop(env, *a, **kw):
        o_ss = env.sim_step
        n = [0]

        def flush():
            if EP["pending"]:
                EP["pending"] = False
                capture(env, "step")
                FRAMES[-1]["case_phys_events"] = FRAMES[-1]["case_phys"]
                FRAMES[-1]["case_phys"] = n[0]

        def sim_step(render=True):
            flush()
            o_ss(render=render)
            n[0] += 1
            if _in_windows("step", None, n[0]):
                EP["pending"] = True

        EP.update(case="drop", case_phys0=PHYS[0], case_act0=int(env.take_action_cnt[0]), armed=True, pending=False)
        if _in_windows("step", None, 0):
            capture(env, "case_start")
        env.sim_step = sim_step
        try:
            return orig(env, *a, **kw)
        finally:
            try:
                flush()
            finally:
                env.sim_step = o_ss
                INFO["drop_sim_steps"] = n[0]
                EP.update(case=None, armed=False)

    sweep_settle.run_drop = run_drop


def _author_usd_pose(prim, v):
    """Write a logged world pose [x y z qw qx qy qz] onto the prim's own translate and orient ops (the layout's parents
    carry identity transforms at env origin 0, so the logged pose is also the local pose; checked by read-back)."""
    from pxr import Gf, UsdGeom

    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        t = op.GetOpType()
        if t == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*[float(x) for x in v[:3]]) if op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble
                   else Gf.Vec3f(*[float(x) for x in v[:3]]))
        elif t == UsdGeom.XformOp.TypeOrient:
            w, x, y, z = (float(c) for c in v[3:7])
            op.Set(Gf.Quatd(w, x, y, z) if op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble else Gf.Quatf(w, x, y, z))


def replay(env):
    """Fallback route: set each logged state and render; physics is never stepped."""
    import numpy as np
    import torch
    from omni.physx import get_physx_interface

    log = json.load(open(A.replay))
    lm = env.scene_manager.layout_manager
    s0 = PHYS[0]

    def wanted(fr):  # the clip spec's windows, applied to the source frame's own counters
        for w in SPEC.get("windows", []):
            unit = w.get("unit", "action")
            why = "step" if fr.get("why") == "case_start" else fr.get("why")  # a case's start is its step 0
            if why != unit or (w.get("case") or None) != (fr.get("case") or None):
                continue
            n = fr.get("action") if unit == "action" else (fr.get("case_phys") if w.get("case") else fr.get("phys"))
            if n is not None and w["from"] <= n <= w["to"] and (n - w["from"]) % int(w.get("stride", 1)) == 0:
                return True
        return False

    frames = [fr for fr in log["frames"] if wanted(fr)]
    INFO["replay_source_frames"] = len(log["frames"])
    INFO["replay_selected_frames"] = len(frames)
    log_msg = f"replaying {len(frames)} of {len(log['frames'])} logged states"
    globals()["log"](log_msg)
    for k, fr in enumerate(frames):
        for label, v in fr["obj"].items():
            o = lm.get_scene_object(0, lm.get_instance_name(0, label))
            o.set_local_pose(translation=np.asarray(v[:3], dtype=float), orientation=np.asarray(v[3:7], dtype=float))
            o.set_linear_velocity(torch.zeros(3))
            o.set_angular_velocity(torch.zeros(3))
            _author_usd_pose(o.prim, v)  # the renderer does not see tensor-API poses without a simulation step
        for label, q in fr.get("art", {}).items():
            lm.get_scene_object(0, lm.get_instance_name(0, label)).set_joint_positions(torch.tensor(q, dtype=torch.float32))
        for key, q in zip(env.robot_manager.robot_key, fr["robots"]):
            jp = torch.tensor([q], dtype=torch.float32, device=key.data.joint_pos.device)
            key.write_joint_state_to_sim(jp, torch.zeros_like(jp))
        # Without a simulation step PhysX does not write the new poses out: compute link poses from the joints
        # (forward), then write every body's PhysX transform to USD, which the renderer reads. No simulation runs.
        env.sim.sim.forward()
        get_physx_interface().update_transformations(False, True, False, False)
        EP["case"] = fr.get("case")
        capture(env, f"replay:{fr['i']}")
        rec = FRAMES[-1]
        # keep this run's engine read-back of the set state; carry the source frame's metadata for overlays
        rec["readback"] = {k: rec.pop(k) for k in ("obj", "art", "robots") if k in rec}
        for k, v in fr.items():
            if k not in ("i", "render_s", "img_mean"):
                rec[k] = v
        rec["replayed_from"] = fr["i"]
    INFO["replay_physics_steps"] = PHYS[0] - s0


_orig_import = builtins.__import__


class _DeployProxy:
    def __init__(self, mod):
        self._mod = mod

    def __getattr__(self, name):
        return getattr(self._mod, name)

    def eval_one_episode(self, TASK_ENV, model_client):
        env = TASK_ENV
        log("episode start")
        if RENDER and not CAMS:
            make_cameras(env)
            log(f"cameras made: {[c['name'] for c in CAMS]}")
        if RENDER:
            s0 = PHYS[0]
            for k in range(A.warmup):
                t0 = time.perf_counter()
                render_frame(env, 1)
                if k < 3 or k == A.warmup - 1:
                    log(f"warm-up render {k} {time.perf_counter() - t0:.2f}s")
            INFO["warmup_physics_steps"] = PHYS[0] - s0
        g = getattr(self._mod.eval_one_episode, "__globals__", {})
        if "RESULT" in g and "clip_run" not in g["RESULT"].get("deviations", ""):
            g["RESULT"]["deviations"] += (
                "\n- clip_run (audit/harness/clips/clip_run.py): "
                + ("harness-side cameras (UsdGeom.Camera under /World/ClipCams, Replicator render products and rgb "
                   f"annotators); {A.warmup} warm-up renders before the episode; per capture the harness's evaluation render "
                   f"(forward, then app.update with playSimulations off) and a Replicator step of {A.subframes} subframes "
                   "with delta_time 0 and playSimulations off, checked to step no physics; " if RENDER else "")
                + "read-only state reads (rigid poses, joint positions, reward progress) at each capture"
                + ("; --replay: the policy is replaced by setting logged states without stepping physics" if A.replay else "")
                + ".")
            INFO["harness_result_has_deviation"] = True
        if RENDER and SPEC.get("mode_sweep"):
            import carb

            st = carb.settings.get_settings()
            for m in SPEC["mode_sweep"]:
                for k, v in m["carb"].items():
                    st.set(k, v)
                for _ in range(int(m.get("warmup", 10))):
                    render_frame(env, 1)
                EP["case"] = None
                capture(env, f"mode:{m['name']}")
                FRAMES[-1]["carb_readback"] = {k: st.get(k) for k in m["carb"]}
        o_ss, o_tab = install_episode_hooks(env)
        EP.update(phys0=PHYS[0], case=None, armed=True)
        try:
            capture(env, "episode_start")
            if A.replay:
                replay(env)
            else:
                self._mod.eval_one_episode(TASK_ENV=TASK_ENV, model_client=model_client)
        finally:
            try:
                flush_pending(env)
            except Exception as e:
                INFO["errors"].append(f"final step capture: {e!r}")
            EP["armed"] = False
            INFO["physics_steps_in_episode"] = PHYS[0] - EP["phys0"]
            try:
                capture(env, "episode_end")
            except Exception as e:
                INFO["errors"].append(f"episode_end capture: {e!r}")
            env.sim_step, env.take_action_batch = o_ss, o_tab
            _write_index()


def _import(name, globals=None, locals=None, fromlist=(), level=0):
    m = _orig_import(name, globals, locals, fromlist, level)
    if name.startswith("XPolicyLab.policy.") and name.endswith(".deploy") and fromlist:
        return _DeployProxy(m)
    return m


builtins.__import__ = _import


def _write_index():
    out = {k: v for k, v in INFO.items() if not k.startswith("_")}
    with open(os.path.join(A.frames_dir, "index.json"), "w") as f:
        json.dump(out, f, indent=1, default=str)


if any(w.get("case") == "drop" for w in SPEC.get("windows", [])) and not A.replay:
    install_drop_hook()
sys.argv = [HARNESS] + rest
# `python run_harness.py` puts the harness's folder first on sys.path, and the harness imports its siblings
# (readback_spec, ...) by bare name; runpy.run_path does not, so set the same sys.path[0] as a direct run.
if sys.path and os.path.abspath(sys.path[0]) == HERE:
    sys.path[0] = os.path.dirname(HARNESS)
else:
    sys.path.insert(0, os.path.dirname(HARNESS))
try:
    runpy.run_path(HARNESS, run_name="__main__")
finally:
    _write_index()
