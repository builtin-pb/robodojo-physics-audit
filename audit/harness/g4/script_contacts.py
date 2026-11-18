"""G4 contact readout for the cover_blocks and swap_T scripts (`--contact_log`; read-only instrumentation).

Two parts:
1. `apply_link_reports`: PhysX contact reports (`PhysxContactReportAPI`, threshold 0) on both robots' link6, link7 and
   link8, authored on the stage after the robots are spawned and before PhysX parses them (`TaskEnv._setup_scene`).
   The rigid objects get the same API from the harness's existing hook (`RigidObject.__init__`). Contact reports add
   no force; they copy contact data out of PhysX after each step.
2. `ContactLog`: wraps the env's `sim_step` during the policy and reads `get_contact_report()` after every physics
   step. Per action it keeps, for every pair of named bodies that touched: steps reported, steps with a non-zero
   impulse, the most contact points in one step, the largest force (sum of |impulse| / dt over the pair's points in
   one step) and the smallest separation. Names: object labels (t0, red, cup1, ...), `r0.link7` (robot 0 link 7),
   `Table`, `Ground`, `Room`, `camera_stand`.

Caveat (#102): GPU contact reports double the impulse on convex decompositions with more than 32 contacts in a header,
so forces read here are upper bounds on such shapes; presence, counts and separations are unaffected.
"""
from __future__ import annotations

import numpy as np

LINKS = ("link6", "link7", "link8")
ENV_ROOT = "/World/envs/env_0"
_FIXTURES = (("/Table", "Table"), ("/Ground", "Ground"), ("/Room", "Room"), ("/geometry/camera_stand/", "camera_stand"))


def apply_link_reports(stage, env_root: str = ENV_ROOT, n_robots: int = 2, links=LINKS) -> list:
    from pxr import PhysxSchema

    done = []
    for i in range(n_robots):
        for ln in links:
            p = stage.GetPrimAtPath(f"{env_root}/robot{i}/{ln}")
            if p and p.IsValid():
                if not p.HasAPI(PhysxSchema.PhysxContactReportAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr().Set(0.0)
                done.append(str(p.GetPath()))
    return done


class ContactLog:
    def __init__(self, env, labels):
        import carb
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        # Isaac Lab sets disableContactProcessing at every simulation launch (the harness closes and relaunches per
        # episode), so turn it off here, at the policy's start, as the exploit and readback paths do.
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        self.env = env
        self.sim_if = get_physx_simulation_interface()
        self.to_path = PhysicsSchemaTools.intToSdfPath
        lm = env.scene_manager.layout_manager
        self.prim_to_label = {}
        for lbl in labels:
            obj = lm.get_scene_object(0, lm.get_instance_name(0, lbl))
            if obj is not None:
                self.prim_to_label[str(obj.prim_path)] = lbl
        self.dt = float(env.sim_cfg.sim.dt)
        self.rows: dict[int, dict[str, list]] = {}
        self.steps = 0
        self._names: dict[str, str | None] = {}
        self._orig = env.sim_step
        env.sim_step = self._sim_step

    def close(self) -> None:
        self.env.sim_step = self._orig

    def name(self, path: str):
        if path in self._names:
            return self._names[path]
        out = None
        for prim, lbl in self.prim_to_label.items():
            if path == prim or path.startswith(prim + "/"):
                out = lbl
                break
        if out is None and path.startswith(ENV_ROOT + "/robot"):
            rest = path[len(ENV_ROOT) + len("/robot"):].split("/")
            out = f"r{rest[0]}.{rest[1]}" if len(rest) > 1 else f"r{rest[0]}"
        if out is None:
            for frag, tag in _FIXTURES:
                if path.startswith(ENV_ROOT + frag):
                    out = tag
                    break
        self._names[path] = out if out is not None else path.split("/")[-1]
        return self._names[path]

    def _sim_step(self, *a, **kw):
        out = self._orig(*a, **kw)
        self.steps += 1
        headers, data = self.sim_if.get_contact_report()
        acc: dict[str, list] = {}
        for h in headers:
            n = int(h.num_contact_data)
            a0 = self.name(str(self.to_path(h.actor0)))
            a1 = self.name(str(self.to_path(h.actor1)))
            key = "|".join(sorted((a0, a1)))
            pts = data[h.contact_data_offset:h.contact_data_offset + n]
            imp = float(sum(float(np.linalg.norm(np.asarray(d.impulse, dtype=float))) for d in pts))
            sep = min((float(d.separation) for d in pts), default=None)
            r = acc.setdefault(key, [0, 0.0, None])
            r[0] += n
            r[1] += imp
            if sep is not None:
                r[2] = sep if r[2] is None else min(r[2], sep)
        if acc:
            t = int(self.env.take_action_cnt[0])
            row = self.rows.setdefault(t, {})
            for key, (n, imp, sep) in acc.items():
                rec = row.get(key)
                if rec is None:
                    rec = row[key] = [0, 0, 0, 0.0, None]
                rec[0] += 1
                rec[1] += int(imp > 0.0)
                rec[2] = max(rec[2], n)
                rec[3] = max(rec[3], imp / self.dt)
                if sep is not None:
                    rec[4] = sep if rec[4] is None else min(rec[4], sep)
        return out

    def export(self) -> dict:
        """Per-action rows (t -> pair -> [steps, steps_with_force, max_points, max_force_N, min_separation_mm]) and
        a per-pair episode summary."""
        rows = {}
        summ: dict[str, dict] = {}
        for t in sorted(self.rows):
            rows[str(t)] = {}
            for key, (st, stf, npts, fmax, sep) in sorted(self.rows[t].items()):
                sep_mm = None if sep is None else round(sep * 1000.0, 3)
                rows[str(t)][key] = [st, stf, npts, round(fmax, 3), sep_mm]
                s = summ.setdefault(key, {"actions": 0, "actions_with_force": 0, "steps": 0, "steps_with_force": 0,
                                          "max_force_N": 0.0, "min_separation_mm": None, "first_t": t, "last_t": t,
                                          "first_t_force": None, "last_t_force": None})
                s["actions"] += 1
                s["steps"] += st
                s["steps_with_force"] += stf
                s["max_force_N"] = max(s["max_force_N"], round(fmax, 3))
                if sep_mm is not None:
                    s["min_separation_mm"] = sep_mm if s["min_separation_mm"] is None else min(s["min_separation_mm"], sep_mm)
                s["last_t"] = t
                if stf:
                    s["actions_with_force"] += 1
                    s["first_t_force"] = t if s["first_t_force"] is None else s["first_t_force"]
                    s["last_t_force"] = t
        return {"steps_read": self.steps, "dt_s": self.dt, "fields": ["steps", "steps_with_force", "max_points",
                                                                        "max_force_N", "min_separation_mm"],
                "per_action": rows, "summary": summ}
