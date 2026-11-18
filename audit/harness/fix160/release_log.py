"""#163: per-physics-step log of each bottle from its release over the bin through the arm's retreat (read-only).

Diagnoses the one failure of the 20 held-out episodes under the #160 fix (seed 0 layout 12): bottle 00022, released
into the bin, rose back out above its release height. The per-action bottle trace cannot say whether it bounced
(restitution at its impact speed), tunnelled (CCD is off) and was pushed out by depenetration, or was struck.

``ReleaseDiag`` reuses ``HandoverDiag``'s per-step records (both grippers, link6/7/8 poses, the bottle's pose and
velocities, every contact on it from PhysX's full contact report with points, normals, impulses and separations) and
opens its window at every ``<arm>_release`` phase instead of handovers, closing it when ``<arm>_retreat_bin`` ends. Each
step also records every other bottle's pose, velocities and contacts (a bottle already in the bin can be pinched
between the falling one and the floor). The contact report is read once per step. After the policy, the authored
``physxRigidBody:maxDepenetrationVelocity`` of each bottle, the authored contact and rest offsets of its colliders, and
the physics materials bound to the dustbin's colliders are added to the constants. Harness flag ``--release_log``
(script_bottles only); written to ``<workdir>/release_s<seed>_l<layout>.json.gz``. Instrumentation only; it changes no
action or physics value, but it needs contact reports on rigid objects, so it cannot run with ``--contact_processing_off``.
"""
from __future__ import annotations

import os
import sys

import numpy as np
from pxr import PhysicsSchemaTools, Usd, UsdPhysics, UsdShade

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../handover_diag"))
from handover_log import HandoverDiag, _r  # noqa: E402

RELEASE_PHASES = ("left_release", "left_retreat_bin", "right_release", "right_retreat_bin")


class ReleaseDiag(HandoverDiag):
    def __init__(self, TASK_ENV, log: dict):
        super().__init__(TASK_ENV, {"wait_cap": False, "slew_m_s": None, "window": "release"}, log)
        self.all_labels = list(self.lm.get_labels_by_prefix(0, "bottle"))

    def _set_phase(self, phase):
        self.phase = phase
        tag, _, name = (phase or "").partition(":")
        in_window = name in RELEASE_PHASES
        if self.cur is not None and (not in_window or self.cur["tag"] != tag):
            self._close()
        if in_window and self.cur is None:
            self._open(tag)

    def _report(self):
        if self.has_full:
            return self.sim_if.get_full_contact_report()
        headers, data = self.sim_if.get_contact_report()
        return headers, data, []

    def _contacts_in(self, report, bpath):
        headers, data, anchors = report
        out = {}
        for h in headers:
            a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            if a0 == bpath:
                s, other = 1.0, a1
            elif a1 == bpath:
                s, other = -1.0, a0
            else:
                continue
            rec = out.setdefault(self._short(other), {"p": [], "a": [], "bottle_actor0": s > 0})
            for d in data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]:
                rec["p"].append(_r(d.position, 6) + _r(s * np.asarray(d.normal, dtype=float), 5)
                                + _r(s * np.asarray(d.impulse, dtype=float), 8) + [round(float(d.separation), 7)])
            if self.has_full and hasattr(h, "num_friction_anchors_data"):
                for fa in anchors[h.friction_anchors_offset:h.friction_anchors_offset + int(h.num_friction_anchors_data)]:
                    rec["a"].append(_r(fa.position, 6) + _r(s * np.asarray(fa.impulse, dtype=float), 8))
        return out

    def _record(self, k):
        try:
            rep = self._report()
            row = {"a": int(self.env.take_action_cnt[0]), "k": k, "ph": self.phase.partition(":")[2], "u": self.u,
                   "arm": [self._arm(0), self._arm(1)], "b": self._bottle(self.cur["label"]),
                   "c": self._contacts_in(rep, self.cur["prim"]),
                   "others": {l: {**self._bottle(l), "c": self._contacts_in(rep, self.bottle_paths[l])}
                              for l in self.all_labels if l != self.cur["label"]}}
            self.cur["steps"].append(row)
        except Exception:
            self._err("record")

    def install(self):
        super().install()
        for label in self.all_labels:  # name every bottle's contacts, not only those released so far
            inst = self.lm.get_instance_name(0, label)
            self.bottle_paths[label] = str(self.lm.get_scene_object(0, inst).prim_path)

    def constants(self):
        out = super().constants()
        import omni.usd

        stage = omni.usd.get_context().get_stage()

        def attr(p, name):
            a = p.GetAttribute(name)
            v = a.Get() if a and a.IsValid() else None
            return float(v) if isinstance(v, (int, float)) else v

        def colliders(root):
            pr = stage.GetPrimAtPath(root)
            return [p for p in Usd.PrimRange(pr, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
                    if pr and pr.IsValid() and p.HasAPI(UsdPhysics.CollisionAPI)]

        usd = {"bottles": {}, "dustbin": {}}
        try:
            for label, path in self.bottle_paths.items():
                cols = colliders(path)
                offs = sorted({(attr(c, "physxCollision:contactOffset"), attr(c, "physxCollision:restOffset")) for c in cols},
                              key=str)
                usd["bottles"][label] = {"maxDepenetrationVelocity": attr(stage.GetPrimAtPath(path), "physxRigidBody:maxDepenetrationVelocity"),
                                         "colliders": len(cols), "contact_rest_offsets": [list(o) for o in offs][:8]}
            bins = [p for p in stage.Traverse() if "/dustbin" in str(p.GetPath()) and p.HasAPI(UsdPhysics.CollisionAPI)]
            mats = {}
            for c in bins:
                m, _ = UsdShade.MaterialBindingAPI(c).ComputeBoundMaterial("physics")
                if m and m.GetPrim().IsValid():
                    mp = m.GetPrim()
                    mats[str(mp.GetPath())] = {a: attr(mp, a) for a in ("physics:staticFriction", "physics:dynamicFriction",
                                                                       "physics:restitution", "physxMaterial:restitutionCombineMode")}
            offs = sorted({(attr(c, "physxCollision:contactOffset"), attr(c, "physxCollision:restOffset")) for c in bins}, key=str)
            usd["dustbin"] = {"colliders": len(bins), "materials": mats, "contact_rest_offsets": [list(o) for o in offs][:8]}
        except Exception:
            self._err("usd_constants")
        out["usd_authored"] = usd
        return out
