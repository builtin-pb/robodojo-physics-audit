#!/usr/bin/env python3
"""Extract collision-mesh cross-section profiles for all 10 bottle models,
plus dustbin/00000, Simple_Room_nolight, and X5 finger pad geometry on CPU.

Run inside isaac-sim_5.1.0.sif with CUDA_VISIBLE_DEVICES="":
  CUDA_VISIBLE_DEVICES="" apptainer exec ... /isaac-sim/python.sh \
      extract_bottle_and_bin_geometry.py --assets <Assets> --out <out.json>
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
import os
from typing import Any

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial import ConvexHull

BOTTLE_IDS = (
    "00000",
    "00002",
    "00005",
    "00022",
    "00025",
    "00041",
    "00050",
    "00055",
    "00059",
    "00060",
)


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(1 << 20)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def has_collision_in_ancestry(prim: Any) -> tuple[bool, bool | None, str | None]:
    """Return (has_collision_api, collision_enabled, approximation) searching prim -> root."""
    has_api = False
    enabled: bool | None = None
    approx: str | None = None
    p = prim
    while p and p.IsValid() and str(p.GetPath()) != "/":
        if p.HasAPI(UsdPhysics.CollisionAPI):
            has_api = True
            capi = UsdPhysics.CollisionAPI(p)
            ea = capi.GetCollisionEnabledAttr()
            if enabled is None and ea and ea.IsValid() and ea.Get() is not None:
                enabled = bool(ea.Get())
        if p.HasAPI(UsdPhysics.MeshCollisionAPI):
            mc = UsdPhysics.MeshCollisionAPI(p)
            aa = mc.GetApproximationAttr()
            if approx is None and aa and aa.IsValid() and aa.Get() is not None:
                approx = str(aa.Get())
        if approx is None:
            aa2 = p.GetAttribute("physics:approximation")
            if aa2 and aa2.IsValid() and aa2.Get() is not None:
                approx = str(aa2.Get())
        p = p.GetParent()
    return has_api, enabled, approx


def extract_stage_meshes(usd_path: str, scale: tuple[float, float, float] = (1.0, 1.0, 1.0),
                         translate: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> tuple[list[dict[str, Any]], np.ndarray]:
    """Return metadata per mesh prim and concatenated collider triangles (N, 3, 3) in scaled+translated root frame."""
    stage = Usd.Stage.Open(usd_path)
    root = stage.GetDefaultPrim()
    if not root or not root.IsValid():
        children = [c for c in stage.GetPseudoRoot().GetChildren() if c.IsValid()]
        root = children[0]
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    root_inv = xcache.GetLocalToWorldTransform(root).GetInverse()

    infos: list[dict[str, Any]] = []
    collider_tris_list: list[np.ndarray] = []
    all_tris_list: list[np.ndarray] = []

    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    for prim in Usd.PrimRange(root, pred):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        img = UsdGeom.Imageable(prim)
        purpose = str(img.ComputePurpose())
        vis = str(img.ComputeVisibility())
        has_col, col_en, approx = has_collision_in_ancestry(prim)
        mesh = UsdGeom.Mesh(prim)
        pts = np.asarray(mesh.GetPointsAttr().Get() or [], dtype=np.float64)
        counts = list(mesh.GetFaceVertexCountsAttr().Get() or [])
        idx = list(mesh.GetFaceVertexIndicesAttr().Get() or [])
        if not len(pts) or not counts:
            continue
        mat4 = np.asarray(xcache.GetLocalToWorldTransform(prim) * root_inv, dtype=np.float64)
        pts_r = (np.hstack([pts, np.ones((len(pts), 1), dtype=np.float64)]) @ mat4)[:, :3]
        pts_r = pts_r * np.asarray(scale, dtype=np.float64)[None, :] + np.asarray(translate, dtype=np.float64)[None, :]

        counts_arr = np.asarray(counts, dtype=np.int64)
        idx_arr = np.asarray(idx, dtype=np.int64)
        if len(counts_arr) > 0 and np.all(counts_arr == 3):
            t_arr = pts_r[idx_arr.reshape(-1, 3)]
        else:
            tris = []
            k = 0
            for c in counts:
                face = idx[k : k + c]
                k += c
                for j in range(1, c - 1):
                    tris.append((pts_r[face[0]], pts_r[face[j]], pts_r[face[j + 1]]))
            if not tris:
                continue
            t_arr = np.asarray(tris, dtype=np.float64)
        is_collider = bool(has_col and (col_en is not False))
        infos.append({
            "path": str(prim.GetPath()),
            "purpose": purpose,
            "visibility": vis,
            "has_collision_api": has_col,
            "collision_enabled": col_en,
            "approximation": approx,
            "is_collider": is_collider,
            "vertex_count": int(len(pts_r)),
            "triangle_count": int(len(t_arr)),
            "aabb_min_m": [round(float(x), 6) for x in pts_r.min(axis=0)],
            "aabb_max_m": [round(float(x), 6) for x in pts_r.max(axis=0)],
        })
        all_tris_list.append(t_arr)
        if is_collider:
            collider_tris_list.append(t_arr)

    use_tris = np.vstack(collider_tris_list) if collider_tris_list else np.vstack(all_tris_list)
    return infos, use_tris


def slice_triangles_xy(tris: np.ndarray, z: float) -> np.ndarray:
    """Exact 2D (x, y) intersection points of triangle array (N, 3, 3) with horizontal plane Z = z."""
    zmin = tris[:, :, 2].min(axis=1)
    zmax = tris[:, :, 2].max(axis=1)
    active = tris[(zmin <= z + 1e-12) & (zmax >= z - 1e-12)]
    if len(active) == 0:
        return np.zeros((0, 2), dtype=np.float64)

    pts_out = []
    # Check each of the 3 edges (0,1), (1,2), (2,0) vectorized
    for i0, i1 in ((0, 1), (1, 2), (2, 0)):
        p0 = active[:, i0, :]
        p1 = active[:, i1, :]
        dz0 = p0[:, 2] - z
        dz1 = p1[:, 2] - z
        # Coplanar or endpoint on plane
        on0 = np.abs(dz0) <= 1e-12
        if np.any(on0):
            pts_out.append(p0[on0, :2])
        # Strict crossing (or one side strict)
        cross = ((dz0 < -1e-12) & (dz1 > 1e-12)) | ((dz0 > 1e-12) & (dz1 < -1e-12))
        if np.any(cross):
            t = dz0[cross] / (dz0[cross] - dz1[cross])
            xy = p0[cross, :2] + t[:, None] * (p1[cross, :2] - p0[cross, :2])
            pts_out.append(xy)
    if not pts_out:
        return np.zeros((0, 2), dtype=np.float64)
    return np.vstack(pts_out)


# Precompute 360 azimuth unit vectors in [0, pi) (0.5 deg step)
THETAS = np.linspace(0.0, math.pi, 360, endpoint=False)
DIRS = np.column_stack([np.cos(THETAS), np.sin(THETAS)])  # (360, 2)


def caliper_extrema_from_xy(pts_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
    """Return (proj_min[360], proj_max[360]) along DIRS (exact support function of pts_xy)."""
    if len(pts_xy) == 0:
        return None
    proj = pts_xy @ DIRS.T  # (M, 360)
    return proj.min(axis=0), proj.max(axis=0)


def analyze_bottle(asset_dir: str, model_id: str) -> dict[str, Any]:
    usd_path = os.path.join(asset_dir, "object.usdz")
    meta_path = os.path.join(asset_dir, "metadata.json")
    with open(meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)

    mesh_infos, tris = extract_stage_meshes(usd_path)
    all_pts = tris.reshape(-1, 3)
    aabb_min = all_pts.min(axis=0)
    aabb_max = all_pts.max(axis=0)
    z_bot = float(aabb_min[2])
    z_top = float(aabb_max[2])
    height_mm = (z_top - z_bot) * 1000.0

    # Also build exact 3D outer convex hull triangle array (PhysX runtime collider when approximation == "convexHull")
    uniq_pts = np.unique(all_pts, axis=0)
    ch3 = ConvexHull(uniq_pts)
    hull3_tris = uniq_pts[ch3.simplices]
    hull3_pts = uniq_pts[ch3.vertices]
    uniq_proj = uniq_pts[:, :2] @ DIRS.T
    hull3_proj = hull3_pts[:, :2] @ DIRS.T
    collider_approx_set = sorted({m["approximation"] for m in mesh_infos if m["is_collider"] and m["approximation"]})
    uses_convex_hull_in_physx = (collider_approx_set == ["convexHull"])

    # Sample slices every 0.5 mm from z_bot + 0.25 mm to z_top - 0.25 mm
    n_slices = int(math.floor((z_top - z_bot) / 0.0005))
    z_grid = np.array([z_bot + (i + 0.5) * 0.0005 for i in range(n_slices)], dtype=np.float64)
    h_above_bot_mm = (z_grid - z_bot) * 1000.0

    pmin_grid = np.full((len(z_grid), len(THETAS)), np.nan, dtype=np.float64)
    pmax_grid = np.full((len(z_grid), len(THETAS)), np.nan, dtype=np.float64)
    slice_w_min_mm = np.full(len(z_grid), np.nan, dtype=np.float64)
    slice_w_max_mm = np.full(len(z_grid), np.nan, dtype=np.float64)

    hull_pmin_grid = np.full((len(z_grid), len(THETAS)), np.nan, dtype=np.float64)
    hull_pmax_grid = np.full((len(z_grid), len(THETAS)), np.nan, dtype=np.float64)
    hull_slice_w_min_mm = np.full(len(z_grid), np.nan, dtype=np.float64)
    hull_slice_w_max_mm = np.full(len(z_grid), np.nan, dtype=np.float64)

    for idx, z in enumerate(z_grid):
        pts_xy = slice_triangles_xy(tris, float(z))
        ext = caliper_extrema_from_xy(pts_xy)
        if ext is not None:
            pmn, pmx = ext
            pmin_grid[idx] = pmn
            pmax_grid[idx] = pmx
            w = (pmx - pmn) * 1000.0
            slice_w_min_mm[idx] = float(w.min())
            slice_w_max_mm[idx] = float(w.max())

        h_xy = slice_triangles_xy(hull3_tris, float(z))
        hext = caliper_extrema_from_xy(h_xy)
        if hext is not None:
            hmn, hmx = hext
            hull_pmin_grid[idx] = hmn
            hull_pmax_grid[idx] = hmx
            hw = (hmx - hmn) * 1000.0
            hull_slice_w_min_mm[idx] = float(hw.min())
            hull_slice_w_max_mm[idx] = float(hw.max())

    def pad_window_widths(z_lo_m: float, z_hi_m: float, use_hull3: bool = False) -> tuple[float, float] | None:
        """Exact parallel-jaw caliper (min_theta, max_theta) in mm over height band [z_lo_m, z_hi_m]."""
        sw = hull_slice_w_min_mm if use_hull3 else slice_w_min_mm
        gmn = hull_pmin_grid if use_hull3 else pmin_grid
        gmx = hull_pmax_grid if use_hull3 else pmax_grid
        vsrc = hull3_pts if use_hull3 else uniq_pts
        vproj = hull3_proj if use_hull3 else uniq_proj
        mask = (z_grid >= z_lo_m - 1e-9) & (z_grid <= z_hi_m + 1e-9) & np.isfinite(sw)
        v_mask = (vsrc[:, 2] >= z_lo_m) & (vsrc[:, 2] <= z_hi_m)
        pmn_list = []
        pmx_list = []
        if np.any(mask):
            pmn_list.append(np.nanmin(gmn[mask], axis=0))
            pmx_list.append(np.nanmax(gmx[mask], axis=0))
        if np.any(v_mask):
            pmn_list.append(vproj[v_mask].min(axis=0))
            pmx_list.append(vproj[v_mask].max(axis=0))
        if not pmn_list:
            return None
        pmn = np.min(np.vstack(pmn_list), axis=0)
        pmx = np.max(np.vstack(pmx_list), axis=0)
        w = (pmx - pmn) * 1000.0
        return float(w.min()), float(w.max())

    # Build 2 mm profile table (above bottom in mm)
    profile_2mm = []
    for h_mm in range(1, int(height_mm), 2):
        idx = min(int(round(h_mm / 0.5)), len(z_grid) - 1)
        zc = z_bot + h_mm * 0.001
        w18 = pad_window_widths(max(z_bot, zc - 0.009165), min(z_top, zc + 0.009165))
        w15 = pad_window_widths(max(z_bot, zc - 0.0075), min(z_top, zc + 0.0075))
        profile_2mm.append({
            "h_above_base_mm": h_mm,
            "z_local_mm": round(zc * 1000.0, 2),
            "slice_w_min_mm": round(float(slice_w_min_mm[idx]), 3),
            "slice_w_max_mm": round(float(slice_w_max_mm[idx]), 3),
            "pad18_3mm_w_min_mm": None if w18 is None else round(w18[0], 3),
            "pad15_0mm_w_min_mm": None if w15 is None else round(w15[0], 3),
        })

    # Automatically locate shoulder transition & cap/neck vs body regions:
    # Body region: from h = 10 mm above base (above bottom base bevel; X5 18.33 mm pad fully on wall at h >= 9.2 mm)
    # up to the upper barrel / waist before shoulder taper begins.
    # Specifically, find the widest barrel diameter D_barrel in the lower 70% of height:
    lower70 = (h_above_bot_mm >= 10.0) & (h_above_bot_mm <= 0.70 * height_mm) & np.isfinite(slice_w_max_mm)
    d_barrel_max_mm = float(np.nanmax(slice_w_max_mm[lower70]))
    d_barrel_min_at_max_mm = float(slice_w_min_mm[lower70][int(np.nanargmax(slice_w_max_mm[lower70]))])

    # Shoulder top: highest h where slice_w_max_mm >= 0.85 * d_barrel_max_mm
    ge85 = np.where((h_above_bot_mm >= 10.0) & (slice_w_max_mm >= 0.85 * d_barrel_max_mm))[0]
    h_body_top_mm = float(h_above_bot_mm[ge85[-1]]) if len(ge85) else 0.75 * height_mm

    # Body graspable widths:
    # (1) Pointwise slice min across body [10 mm, h_body_top_mm]
    body_slice_mask = (h_above_bot_mm >= 10.0) & (h_above_bot_mm <= h_body_top_mm) & np.isfinite(slice_w_min_mm)
    body_slice_min_idx = np.where(body_slice_mask)[0][int(np.argmin(slice_w_min_mm[body_slice_mask]))]
    body_slice_min_mm = float(slice_w_min_mm[body_slice_min_idx])
    body_slice_min_h_mm = float(h_above_bot_mm[body_slice_min_idx])

    # (2) Parallel-jaw 18.33 mm pad min across body (pad center h_c in [10 mm, h_body_top_mm], pad completely within body/bottle)
    body_pad18_candidates = []
    body_pad15_candidates = []
    for h_c in np.arange(9.2, max(9.5, h_body_top_mm - 4.0) + 0.5, 1.0):
        zc = z_bot + h_c * 0.001
        w18 = pad_window_widths(zc - 0.009165, zc + 0.009165)
        if w18 is not None:
            body_pad18_candidates.append((w18[0], w18[1], float(h_c)))
        w15 = pad_window_widths(zc - 0.0075, zc + 0.0075)
        if w15 is not None:
            body_pad15_candidates.append((w15[0], w15[1], float(h_c)))
    body_pad18_min = min(body_pad18_candidates, key=lambda x: x[0])
    body_pad15_min = min(body_pad15_candidates, key=lambda x: x[0])

    # Also standing-away-from-table-edge constraint (slider housing 61 mm tall -> h_c >= 30.5 mm):
    body_standing_pad18 = [c for c in body_pad18_candidates if c[2] >= 30.5]
    body_standing_pad18_min = min(body_standing_pad18, key=lambda x: x[0]) if body_standing_pad18 else body_pad18_min

    # Upper region (neck & cap): h in [h_body_top_mm, height_mm - 4.0 mm] (excluding the top 4 mm cap chamfer/dome)
    has_distinct_neck_cap = (height_mm - h_body_top_mm) >= 12.0
    upper_mask = (h_above_bot_mm >= min(h_body_top_mm, height_mm - 25.0)) & (h_above_bot_mm <= height_mm - 4.0) & np.isfinite(slice_w_min_mm)
    upper_min_idx = np.where(upper_mask)[0][int(np.argmin(slice_w_min_mm[upper_mask]))]
    neck_slice_min_mm = float(slice_w_min_mm[upper_min_idx])
    neck_slice_min_h_mm = float(h_above_bot_mm[upper_min_idx])

    # Cap region specifically: top [height_mm - 18 mm, height_mm - 3 mm]
    cap_mask = (h_above_bot_mm >= height_mm - 18.0) & (h_above_bot_mm <= height_mm - 3.0) & np.isfinite(slice_w_min_mm)
    # Typical cap cylindrical wall is near the local max in the top 18 mm!
    cap_wall_idx = np.where(cap_mask)[0][int(np.argmax(slice_w_min_mm[cap_mask]))]
    cap_cyl_w_min_mm = float(slice_w_min_mm[cap_wall_idx])
    cap_cyl_w_max_mm = float(slice_w_max_mm[cap_wall_idx])
    cap_cyl_h_mm = float(h_above_bot_mm[cap_wall_idx])

    # Parallel-jaw grasp on cap/neck:
    # A parallel jaw can clamp the cap near the top (with pad extending above z_top so only >= 6 mm of cap wall is engaged:
    # z_lo in [z_top - 18 mm, z_top - 6 mm], z_hi = z_top) OR clamp across neck/cap with full 15 mm / 18.33 mm pad.
    cap_top_pinch_candidates = []
    for engage_mm in range(6, 19):
        w_cap = pad_window_widths(z_top - engage_mm * 0.001, z_top)
        if w_cap is not None:
            cap_top_pinch_candidates.append((w_cap[0], w_cap[1], engage_mm))
    cap_top_pinch_min = min(cap_top_pinch_candidates, key=lambda x: x[0])

    upper_pad15_candidates = []
    upper_pad18_candidates = []
    h_start_upper = max(10.0, h_body_top_mm - 5.0)
    for h_c in np.arange(h_start_upper, height_mm - 5.0, 1.0):
        zc = z_bot + h_c * 0.001
        w15 = pad_window_widths(zc - 0.0075, min(z_top, zc + 0.0075))
        if w15 is not None:
            upper_pad15_candidates.append((w15[0], w15[1], float(h_c)))
        w18 = pad_window_widths(zc - 0.009165, min(z_top, zc + 0.009165))
        if w18 is not None:
            upper_pad18_candidates.append((w18[0], w18[1], float(h_c)))
    upper_pad15_min = min(upper_pad15_candidates, key=lambda x: x[0]) if upper_pad15_candidates else cap_top_pinch_min
    upper_pad18_min = min(upper_pad18_candidates, key=lambda x: x[0]) if upper_pad18_candidates else cap_top_pinch_min

    # Also compute active PhysX collider widths (hull3 when approximation == "convexHull", else mesh):
    active_body_slice_min_mm = float(np.nanmin(hull_slice_w_min_mm[body_slice_mask])) if uses_convex_hull_in_physx else body_slice_min_mm
    active_body_pad18_cands = []
    for h_c in np.arange(9.2, max(9.5, h_body_top_mm - 4.0) + 0.5, 1.0):
        zc = z_bot + h_c * 0.001
        w18a = pad_window_widths(zc - 0.009165, zc + 0.009165, use_hull3=uses_convex_hull_in_physx)
        if w18a is not None:
            active_body_pad18_cands.append((w18a[0], w18a[1], float(h_c)))
    active_body_pad18_min = min(active_body_pad18_cands, key=lambda x: x[0])

    active_upper_slice_min_mm = float(np.nanmin(hull_slice_w_min_mm[upper_mask])) if uses_convex_hull_in_physx else neck_slice_min_mm
    active_upper_pad18_cands = []
    for h_c in np.arange(h_start_upper, height_mm - 5.0, 1.0):
        zc = z_bot + h_c * 0.001
        w18a = pad_window_widths(zc - 0.009165, min(z_top, zc + 0.009165), use_hull3=uses_convex_hull_in_physx)
        if w18a is not None:
            active_upper_pad18_cands.append((w18a[0], w18a[1], float(h_c)))
    active_upper_pad18_min = min(active_upper_pad18_cands, key=lambda x: x[0]) if active_upper_pad18_cands else upper_pad18_min

    # Specifically for 00022: separate the cylindrical screw-cap/neck region (h in [175, 206] mm) from the 33 mm flat bail loop (h > 206 mm)
    screw_cap_neck_note = None
    if model_id == "00022":
        sc_mask = (h_above_bot_mm >= 175.0) & (h_above_bot_mm <= 205.5) & np.isfinite(slice_w_min_mm)
        bail_mask = (h_above_bot_mm >= 210.0) & (h_above_bot_mm <= 237.0) & np.isfinite(slice_w_min_mm)
        screw_cap_neck_note = {
            "neck_groove_mesh_min_mm": round(float(np.nanmin(slice_w_min_mm[sc_mask])), 3),
            "screw_cap_cylinder_mm": round(float(np.nanmax(slice_w_min_mm[sc_mask])), 3),
            "convex_hull_at_neck_cap_min_mm": round(float(np.nanmin(hull_slice_w_min_mm[sc_mask])), 3),
            "top_bail_loop_mesh_thickness_mm": round(float(np.nanmin(slice_w_min_mm[bail_mask])), 3),
            "top_bail_loop_height_span_mm": [207.0, 240.0],
            "convex_hull_bail_wedge_half_angle_deg": round(math.degrees(math.atan((48.64 - 10.98) / (2.0 * 33.0))), 2),
        }

    # Metadata bbox thinnest side (as evaluator_v2.py gripper_holds_bottle_v2 computes it!)
    ob_verts = np.asarray(meta["geometry"]["oriented_bbox"]["vertices"], dtype=np.float64)
    ob_aabb_sides_mm = ((ob_verts.max(axis=0) - ob_verts.min(axis=0)) * 1000.0).tolist()
    ob_extents_mm = (np.asarray(meta["geometry"]["oriented_bbox"]["extents"], dtype=np.float64) * 1000.0).tolist()
    al_extents_mm = (np.asarray(meta["geometry"]["aligned_bbox"]["extents"], dtype=np.float64) * 1000.0).tolist()

    return {
        "model_id": f"Rigid/bottle/{model_id}",
        "short_id": model_id,
        "usdz_sha256": sha256_file(usd_path),
        "metadata_sha256": sha256_file(meta_path),
        "mesh_prims": mesh_infos,
        "collider_aabb_min_mm": [round(float(x * 1000.0), 3) for x in aabb_min],
        "collider_aabb_max_mm": [round(float(x * 1000.0), 3) for x in aabb_max],
        "collider_extents_mm": [round(float((aabb_max[i] - aabb_min[i]) * 1000.0), 3) for i in range(3)],
        "metadata_aligned_extents_mm": [round(x, 3) for x in al_extents_mm],
        "metadata_oriented_extents_mm": [round(x, 3) for x in ob_extents_mm],
        "evaluator_v2_local_bbox_sides_mm": [round(x, 3) for x in ob_aabb_sides_mm],
        "evaluator_v2_thinnest_mm": round(min(ob_aabb_sides_mm), 3),
        "collider_approximations": collider_approx_set,
        "uses_convex_hull_in_physx": uses_convex_hull_in_physx,
        "height_mm": round(height_mm, 3),
        "d_barrel_max_mm": round(d_barrel_max_mm, 3),
        "d_barrel_min_azimuth_mm": round(d_barrel_min_at_max_mm, 3),
        "h_body_top_mm": round(h_body_top_mm, 2),
        "has_distinct_neck_cap": bool(has_distinct_neck_cap),
        "body": {
            "slice_min_mm": round(body_slice_min_mm, 3),
            "slice_min_h_above_base_mm": round(body_slice_min_h_mm, 2),
            "pad18_3mm_min_mm": round(body_pad18_min[0], 3),
            "pad18_3mm_max_az_at_min_mm": round(body_pad18_min[1], 3),
            "pad18_3mm_min_h_mm": round(body_pad18_min[2], 2),
            "pad18_3mm_standing_midtable_min_mm": round(body_standing_pad18_min[0], 3),
            "pad18_3mm_standing_midtable_h_mm": round(body_standing_pad18_min[2], 2),
            "pad15_0mm_min_mm": round(body_pad15_min[0], 3),
            "pad15_0mm_min_h_mm": round(body_pad15_min[2], 2),
            "physx_active_slice_min_mm": round(active_body_slice_min_mm, 3),
            "physx_active_pad18_3mm_min_mm": round(active_body_pad18_min[0], 3),
        },
        "cap_neck": {
            "neck_slice_min_mm": round(neck_slice_min_mm, 3),
            "neck_slice_min_h_mm": round(neck_slice_min_h_mm, 2),
            "cap_wall_w_min_mm": round(cap_cyl_w_min_mm, 3),
            "cap_wall_w_max_mm": round(cap_cyl_w_max_mm, 3),
            "cap_wall_h_mm": round(cap_cyl_h_mm, 2),
            "cap_top_overhang_pinch_min_mm": round(cap_top_pinch_min[0], 3),
            "cap_top_overhang_engage_mm": cap_top_pinch_min[2],
            "pad15_0mm_min_mm": round(upper_pad15_min[0], 3),
            "pad15_0mm_min_h_mm": round(upper_pad15_min[2], 2),
            "pad18_3mm_min_mm": round(upper_pad18_min[0], 3),
            "pad18_3mm_min_h_mm": round(upper_pad18_min[2], 2),
            "physx_active_slice_min_mm": round(active_upper_slice_min_mm, 3),
            "physx_active_pad18_3mm_min_mm": round(active_upper_pad18_min[0], 3),
            "special_00022_bail_and_cap": screw_cap_neck_note,
        },
        "profile_2mm": profile_2mm,
    }


def analyze_dustbin_and_room(assets_root: str) -> dict[str, Any]:
    bin_dir = os.path.join(assets_root, "Object/RoboDojo/Geometry/dustbin/00000")
    bin_usd = os.path.join(bin_dir, "object.usdz")
    bin_meta_p = os.path.join(bin_dir, "metadata.json")
    with open(bin_meta_p, "r", encoding="utf-8") as f:
        bin_meta = json.load(f)

    # Dustbin at layout 0 world pose [-0.63, -0.10, 0.40], identity rotation, scale [1,1,1]
    bin_pos = (-0.63, -0.10, 0.40)
    bin_infos, bin_tris = extract_stage_meshes(bin_usd, scale=(1.0, 1.0, 1.0), translate=bin_pos)
    bpts = bin_tris.reshape(-1, 3)
    w_min = bpts.min(axis=0)
    w_max = bpts.max(axis=0)

    # Slice dustbin at key world z levels to measure outer bounds, inner cavity bounds, and wall thickness!
    cx, cy = -0.63, -0.10
    z_levels = sorted(set(
        [round(0.076 + k * 0.025, 4) for k in range(26)]
        + [round(0.240 + k * 0.005, 4) for k in range(15)]
        + [round(0.700 + k * 0.002, 4) for k in range(13)]
    ))
    bin_slices = []
    for zw in z_levels:
        if zw <= w_min[2] or zw >= w_max[2]:
            continue
        xy = slice_triangles_xy(bin_tris, zw)
        if len(xy) == 0:
            continue
        # Along x near y = cy (|y - cy| < 0.05) and along y near x = cx (|x - cx| < 0.05)
        y_band = xy[np.abs(xy[:, 1] - cy) <= 0.04]
        x_band = xy[np.abs(xy[:, 0] - cx) <= 0.04]
        rec: dict[str, Any] = {
            "z_world_m": round(zw, 4),
            "outer_x_m": [round(float(xy[:, 0].min()), 5), round(float(xy[:, 0].max()), 5)],
            "outer_y_m": [round(float(xy[:, 1].min()), 5), round(float(xy[:, 1].max()), 5)],
        }
        if len(y_band):
            left_x = y_band[y_band[:, 0] < cx, 0]
            right_x = y_band[y_band[:, 0] > cx, 0]
            if len(left_x) and len(right_x):
                rec["wall_left_x_m"] = [round(float(left_x.min()), 5), round(float(left_x.max()), 5)]
                rec["wall_right_x_m"] = [round(float(right_x.min()), 5), round(float(right_x.max()), 5)]
                rec["cavity_x_m"] = [round(float(left_x.max()), 5), round(float(right_x.min()), 5)]
                rec["right_rim_thickness_mm"] = round(float((right_x.max() - right_x.min()) * 1000.0), 3)
        if len(x_band):
            front_y = x_band[x_band[:, 1] < cy, 1]
            back_y = x_band[x_band[:, 1] > cy, 1]
            if len(front_y) and len(back_y):
                rec["wall_front_y_m"] = [round(float(front_y.min()), 5), round(float(front_y.max()), 5)]
                rec["wall_back_y_m"] = [round(float(back_y.min()), 5), round(float(back_y.max()), 5)]
                rec["cavity_y_m"] = [round(float(front_y.max()), 5), round(float(back_y.min()), 5)]
        bin_slices.append(rec)

    # Find interior bottom surface z near center (|x - cx| < 0.08, |y - cy| < 0.08)
    center_pts = bpts[(np.abs(bpts[:, 0] - cx) <= 0.08) & (np.abs(bpts[:, 1] - cy) <= 0.08)]
    center_z_vals = sorted(set(round(float(z), 5) for z in center_pts[:, 2])) if len(center_pts) else []

    # Check Simple_Room_nolight floor z at scale (0.5, 0.5, 0.5)
    room_dir = os.path.join(assets_root, "Room/Simple_Room_nolight")
    room_info = {}
    if os.path.isdir(room_dir):
        usd_files = sorted(x for x in os.listdir(room_dir) if x.endswith((".usd", ".usda", ".usdc", ".usdz")))
        if usd_files:
            r_usd = os.path.join(room_dir, usd_files[0])
            r_meshes, r_tris = extract_stage_meshes(r_usd, scale=(0.5, 0.5, 0.5), translate=(0.0, 0.0, 0.0))
            # Check triangles near [-0.9, -0.3] x [-0.4, +0.2] and z in [-0.2, +0.2]
            near_bin = r_tris[
                (r_tris[:, :, 0].max(axis=1) >= -0.95) & (r_tris[:, :, 0].min(axis=1) <= -0.30)
                & (r_tris[:, :, 1].max(axis=1) >= -0.45) & (r_tris[:, :, 1].min(axis=1) <= 0.25)
                & (r_tris[:, :, 2].min(axis=1) <= 0.20)
            ]
            room_info = {
                "usd_file": usd_files[0],
                "mesh_count": len(r_meshes),
                "collider_mesh_count": sum(1 for m in r_meshes if m["is_collider"]),
                "near_bin_floor_z_min_max_m": (
                    [round(float(near_bin[:, :, 2].min()), 5), round(float(near_bin[:, :, 2].max()), 5)]
                    if len(near_bin) else None
                ),
                "meshes_summary": r_meshes[:20],
            }

    ob = np.asarray(bin_meta["geometry"]["oriented_bbox"]["vertices"], dtype=np.float64) + np.asarray(bin_pos)[None, :]
    return {
        "usdz_sha256": sha256_file(bin_usd),
        "metadata_sha256": sha256_file(bin_meta_p),
        "layout0_pose_xyz_m": list(bin_pos),
        "collider_world_aabb_min_m": [round(float(x), 5) for x in w_min],
        "collider_world_aabb_max_m": [round(float(x), 5) for x in w_max],
        "oriented_bbox_world_min_m": [round(float(x), 5) for x in ob.min(axis=0)],
        "oriented_bbox_world_max_m": [round(float(x), 5) for x in ob.max(axis=0)],
        "table_world_bounds_m": {"x": [-0.41, 0.59], "y": [-0.60, 0.50], "z": [0.715, 0.765]},
        "ground_cube_top_z_m": 0.0,
        "center_floor_z_samples_m": [center_z_vals[0], center_z_vals[-1], len(center_z_vals)] if center_z_vals else [],
        "center_floor_z_unique_head_tail_m": center_z_vals[:8] + center_z_vals[-8:] if len(center_z_vals) > 16 else center_z_vals,
        "mesh_prims": bin_infos,
        "slices": bin_slices,
        "room": room_info,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--assets", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    bottles = {}
    for bid in BOTTLE_IDS:
        bdir = os.path.join(args.assets, "Object/RoboDojo/Rigid/bottle", bid)
        bottles[bid] = analyze_bottle(bdir, bid)

    dustbin = analyze_dustbin_and_room(args.assets)

    summary_rows = []
    for bid in BOTTLE_IDS:
        b = bottles[bid]
        summary_rows.append({
            "model": bid,
            "approx": ",".join(b["collider_approximations"]),
            "height_mm": b["height_mm"],
            "barrel_max_mm": b["d_barrel_max_mm"],
            "eval_v2_bbox_thinnest_mm": b["evaluator_v2_thinnest_mm"],
            "body_slice_min_mm": b["body"]["slice_min_mm"],
            "body_pad18_3mm_min_mm": b["body"]["pad18_3mm_min_mm"],
            "body_physx_pad18_3mm_min_mm": b["body"]["physx_active_pad18_3mm_min_mm"],
            "body_pad15_0mm_min_mm": b["body"]["pad15_0mm_min_mm"],
            "has_distinct_neck_cap": b["has_distinct_neck_cap"],
            "neck_slice_min_mm": b["cap_neck"]["neck_slice_min_mm"],
            "cap_wall_w_min_mm": b["cap_neck"]["cap_wall_w_min_mm"],
            "cap_top_pinch_min_mm": b["cap_neck"]["cap_top_overhang_pinch_min_mm"],
            "upper_pad15_0mm_min_mm": b["cap_neck"]["pad15_0mm_min_mm"],
            "upper_pad18_3mm_min_mm": b["cap_neck"]["pad18_3mm_min_mm"],
            "upper_physx_pad18_3mm_min_mm": b["cap_neck"]["physx_active_pad18_3mm_min_mm"],
            "special_00022": b["cap_neck"]["special_00022_bail_and_cap"],
        })

    out = {
        "schema": "bottles_g3_usd_geometry_v1",
        "summary_per_model": summary_rows,
        "global_minima_mm": {
            "body_slice_min_mm": min(r["body_slice_min_mm"] for r in summary_rows),
            "body_pad18_3mm_min_mm": min(r["body_pad18_3mm_min_mm"] for r in summary_rows),
            "body_pad15_0mm_min_mm": min(r["body_pad15_0mm_min_mm"] for r in summary_rows),
            "neck_slice_min_mm": min(r["neck_slice_min_mm"] for r in summary_rows),
            "cap_wall_w_min_mm": min(r["cap_wall_w_min_mm"] for r in summary_rows),
            "cap_top_pinch_min_mm": min(r["cap_top_pinch_min_mm"] for r in summary_rows),
            "upper_pad15_0mm_min_mm": min(r["upper_pad15_0mm_min_mm"] for r in summary_rows),
            "upper_pad18_3mm_min_mm": min(r["upper_pad18_3mm_min_mm"] for r in summary_rows),
        },
        "bottles": bottles,
        "dustbin_and_room": dustbin,
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    opener = gzip.open if args.out.endswith(".gz") else open
    with opener(args.out, "wt", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
    print(json.dumps({"summary_per_model": summary_rows, "global_minima_mm": out["global_minima_mm"]}, indent=2))


if __name__ == "__main__":
    main()
