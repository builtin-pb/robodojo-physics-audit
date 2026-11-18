#!/usr/bin/env python3
"""Extract deterministic geometric facts (collider bounds, outer-hull volume/area, solid and thin-shell COM and I/m)
from pinned Hugging Face RoboDojo USD assets (revision 1127bda94f172d757386945af3eb3a6754d1a4e6).

Run inside isaac-sim_5.1.0.sif (CPU-only, 0 GPU):
  CUDA_VISIBLE_DEVICES="" apptainer exec ... /isaac-sim/python.sh audit/tools/extract_usd_geometry_facts.py \
      --assets /path/to/Assets --out audit/out/63/usd_geometry_facts.json
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import time
from typing import Any

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial import ConvexHull

HF_REVISION = "1127bda94f172d757386945af3eb3a6754d1a4e6"
BOTTLE_INDICES = (0, 2, 5, 22, 25, 41, 50, 55, 59, 60)


def _sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(1 << 20)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def _find_usd(asset_dir: str) -> str:
    cands = sorted(x for x in os.listdir(asset_dir) if x.endswith((".usd", ".usda", ".usdc", ".usdz")))
    if not cands:
        raise FileNotFoundError(f"No USD file in {asset_dir}")
    return os.path.join(asset_dir, cands[0])


def _convex_hull_solid_and_shell_moments(points_xyz: np.ndarray) -> dict[str, Any]:
    """Exact closed-polyhedron surface and tetrahedral integrals on the outer convex hull of points_xyz."""
    ch = ConvexHull(points_xyz)
    hull_pts = points_xyz[ch.vertices]
    centroid = hull_pts.mean(axis=0)
    tris = []
    for simp in ch.simplices:
        v0, v1, v2 = points_xyz[simp[0]], points_xyz[simp[1]], points_xyz[simp[2]]
        if float(np.dot(np.cross(v1 - v0, v2 - v0), v0 - centroid)) < 0.0:
            v1, v2 = v2, v1
        tris.append((v0, v1, v2))
    tris_arr = np.asarray(tris, dtype=np.float64)
    v0, v1, v2 = tris_arr[:, 0], tris_arr[:, 1], tris_arr[:, 2]

    # 1. Uniform-areal-density thin shell over outer convex hull
    areas = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    total_area = float(np.sum(areas))
    com_shell = np.sum(areas[:, None] * ((v0 + v1 + v2) / 3.0), axis=0) / total_area
    u0, u1, u2 = v0 - com_shell, v1 - com_shell, v2 - com_shell
    usum = u0 + u1 + u2
    c_shell = (
        np.einsum("n,ni,nj->ij", areas / 12.0, usum, usum)
        + np.einsum("n,ni,nj->ij", areas / 12.0, u0, u0)
        + np.einsum("n,ni,nj->ij", areas / 12.0, u1, u1)
        + np.einsum("n,ni,nj->ij", areas / 12.0, u2, u2)
    ) / total_area
    i_shell = np.trace(c_shell) * np.eye(3) - c_shell
    i_shell_diag = np.diag(i_shell)
    i_shell_off = float(np.max(np.abs(i_shell - np.diag(i_shell_diag))) / np.max(i_shell_diag))

    # 2. Uniform-volume-density solid over outer convex hull
    signed_vols = np.einsum("ni,ni->n", v0, np.cross(v1, v2)) / 6.0
    total_vol = float(np.sum(signed_vols))
    com_solid = np.sum(signed_vols[:, None] * ((v0 + v1 + v2) / 4.0), axis=0) / total_vol
    w0, w1, w2 = v0 - com_solid, v1 - com_solid, v2 - com_solid
    wsum = w0 + w1 + w2
    vt = np.einsum("ni,ni->n", w0, np.cross(w1, w2)) / 6.0
    c_solid = (
        np.einsum("n,ni,nj->ij", vt / 20.0, wsum, wsum)
        + np.einsum("n,ni,nj->ij", vt / 20.0, w0, w0)
        + np.einsum("n,ni,nj->ij", vt / 20.0, w1, w1)
        + np.einsum("n,ni,nj->ij", vt / 20.0, w2, w2)
    ) / float(np.sum(vt))
    i_solid = np.trace(c_solid) * np.eye(3) - c_solid
    i_solid_diag = np.diag(i_solid)
    i_solid_off = float(np.max(np.abs(i_solid - np.diag(i_solid_diag))) / np.max(i_solid_diag))

    return {
        "hull_vertex_count": int(len(ch.vertices)),
        "hull_face_count": int(len(ch.simplices)),
        "outer_hull_volume_m3": round(total_vol, 9),
        "outer_hull_area_m2": round(total_area, 8),
        "solid_com_offset_m": [round(float(x), 6) for x in com_solid],
        "solid_inertia_over_mass_m2": [round(float(x), 8) for x in i_solid_diag],
        "solid_offdiag_max_rel": round(i_solid_off, 6),
        "shell_com_offset_m": [round(float(x), 6) for x in com_shell],
        "shell_inertia_over_mass_m2": [round(float(x), 8) for x in i_shell_diag],
        "shell_offdiag_max_rel": round(i_shell_off, 6),
        "shell_over_solid_ratio": [round(float(i_shell_diag[k] / i_solid_diag[k]), 4) for k in range(3)],
    }


def _mesh_triangles_in_root(prim: Any, root_prim: Any, xcache: Any) -> np.ndarray:
    """Fan-triangulate a UsdGeom.Mesh into an (n, 3, 3) array in the rigid-body root frame."""
    mesh = UsdGeom.Mesh(prim)
    pts = np.asarray(mesh.GetPointsAttr().Get() or [], dtype=np.float64)
    counts = list(mesh.GetFaceVertexCountsAttr().Get() or [])
    idx = list(mesh.GetFaceVertexIndicesAttr().Get() or [])
    if not len(pts) or not counts:
        return np.zeros((0, 3, 3), dtype=np.float64)
    mat4 = np.asarray(xcache.GetLocalToWorldTransform(prim) * xcache.GetLocalToWorldTransform(root_prim).GetInverse(), dtype=np.float64)
    pts_r = (np.hstack([pts, np.ones((len(pts), 1), dtype=np.float64)]) @ mat4)[:, :3]
    tris = []
    k = 0
    for c in counts:
        face = idx[k : k + c]
        k += c
        for j in range(1, c - 1):
            tris.append((pts_r[face[0]], pts_r[face[j]], pts_r[face[j + 1]]))
    return np.asarray(tris, dtype=np.float64)


def _surface_mesh_moments(tris: np.ndarray, weld_tol_m: float = 1e-7) -> dict[str, Any]:
    """Moments over the actual triangle surface (not its convex hull).

    Solid: uniform-volume-density integral over the region the surface encloses (divergence theorem; exact for a
    closed, consistently oriented surface, e.g. a box, a T or a double-walled cup whose enclosed region is the wall).
    Shell: uniform-areal-density integral over the triangles themselves (exact for any surface, open or closed).
    The closedness check welds vertices at weld_tol_m and counts edges used by exactly one face (boundary) and by
    more than two faces (non-manifold).
    """
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    cr = np.cross(v1 - v0, v2 - v0)
    areas = 0.5 * np.linalg.norm(cr, axis=1)
    keep = areas > 0.0
    v0, v1, v2, areas = v0[keep], v1[keep], v2[keep], areas[keep]

    # Closedness (topology after welding).
    allv = np.vstack([v0, v1, v2])
    keys = np.round(allv / weld_tol_m).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    n = len(v0)
    f = np.stack([inv[:n], inv[n : 2 * n], inv[2 * n :]], axis=1)
    edges = np.sort(np.vstack([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
    _, ecount = np.unique(edges, axis=0, return_counts=True)
    boundary_edges = int(np.sum(ecount == 1))
    nonmanifold_edges = int(np.sum(ecount > 2))

    # Shell over the actual surface.
    total_area = float(np.sum(areas))
    com_shell = np.sum(areas[:, None] * ((v0 + v1 + v2) / 3.0), axis=0) / total_area
    u0, u1, u2 = v0 - com_shell, v1 - com_shell, v2 - com_shell
    usum = u0 + u1 + u2
    c_shell = (
        np.einsum("n,ni,nj->ij", areas / 12.0, usum, usum)
        + np.einsum("n,ni,nj->ij", areas / 12.0, u0, u0)
        + np.einsum("n,ni,nj->ij", areas / 12.0, u1, u1)
        + np.einsum("n,ni,nj->ij", areas / 12.0, u2, u2)
    ) / total_area
    i_shell = np.trace(c_shell) * np.eye(3) - c_shell
    i_shell_diag = np.diag(i_shell)

    # Solid over the enclosed region (sign fixed so the volume is positive).
    signed_vols = np.einsum("ni,ni->n", v0, np.cross(v1, v2)) / 6.0
    total_vol = float(np.sum(signed_vols))
    sgn = 1.0 if total_vol >= 0.0 else -1.0
    signed_vols = sgn * signed_vols
    total_vol = abs(total_vol)
    out: dict[str, Any] = {
        "triangle_count": int(n),
        "welded_vertex_count": int(inv.max() + 1) if n else 0,
        "boundary_edge_count": boundary_edges,
        "nonmanifold_edge_count": nonmanifold_edges,
        "closed_manifold": bool(boundary_edges == 0 and nonmanifold_edges == 0),
        "surface_area_m2": round(total_area, 8),
        "surface_shell_com_offset_m": [round(float(x), 6) for x in com_shell],
        "surface_shell_inertia_over_mass_m2": [round(float(x), 9) for x in i_shell_diag],
        "surface_shell_offdiag_max_rel": round(float(np.max(np.abs(i_shell - np.diag(i_shell_diag))) / np.max(i_shell_diag)), 6),
        "enclosed_volume_m3": round(total_vol, 10),
    }
    if total_vol > 0.0:
        com_solid = np.sum(signed_vols[:, None] * ((v0 + v1 + v2) / 4.0), axis=0) / total_vol
        w0, w1, w2 = v0 - com_solid, v1 - com_solid, v2 - com_solid
        wsum = w0 + w1 + w2
        vt = sgn * np.einsum("ni,ni->n", w0, np.cross(w1, w2)) / 6.0
        c_solid = (
            np.einsum("n,ni,nj->ij", vt / 20.0, wsum, wsum)
            + np.einsum("n,ni,nj->ij", vt / 20.0, w0, w0)
            + np.einsum("n,ni,nj->ij", vt / 20.0, w1, w1)
            + np.einsum("n,ni,nj->ij", vt / 20.0, w2, w2)
        ) / float(np.sum(vt))
        i_solid = np.trace(c_solid) * np.eye(3) - c_solid
        i_solid_diag = np.diag(i_solid)
        out["enclosed_solid_com_offset_m"] = [round(float(x), 6) for x in com_solid]
        out["enclosed_solid_inertia_over_mass_m2"] = [round(float(x), 9) for x in i_solid_diag]
        out["enclosed_solid_offdiag_max_rel"] = round(
            float(np.max(np.abs(i_solid - np.diag(i_solid_diag))) / np.max(i_solid_diag)), 6
        )
    return out


def _box_primitive_colliders(root_prim: Any) -> dict[str, Any] | None:
    """Exact uniform-solid moments of the union of enabled UsdGeom.Cube collider primitives (axis-aligned in the root
    frame, pairwise overlap volume reported; the union moments assume zero overlap and are omitted otherwise)."""
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    root_inv = xcache.GetLocalToWorldTransform(root_prim).GetInverse()
    boxes = []
    for p in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if not (p.HasAPI(UsdPhysics.CollisionAPI) and p.IsA(UsdGeom.Cube)):
            continue
        en_attr = p.GetAttribute("physics:collisionEnabled")
        if en_attr and en_attr.IsValid() and en_attr.Get() is False:
            continue
        size = float(UsdGeom.Cube(p).GetSizeAttr().Get())
        mat4 = np.asarray(xcache.GetLocalToWorldTransform(p) * root_inv, dtype=np.float64)
        rot_scale = mat4[:3, :3]
        center = mat4[3, :3]
        # Row-vector convention: local axis k maps to row k of rot_scale; require axis alignment.
        dims = np.linalg.norm(rot_scale, axis=1) * size
        dirs = rot_scale / np.linalg.norm(rot_scale, axis=1)[:, None]
        if not np.allclose(np.abs(dirs), np.eye(3), atol=1e-9):
            return {"error": f"non-axis-aligned Cube collider {p.GetPath()}"}
        boxes.append({"prim_path": str(p.GetPath()), "cube_size_attr": size, "center_m": center, "dims_m": dims})
    if not boxes:
        return None
    overlap = 0.0
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            lo = np.maximum(boxes[i]["center_m"] - boxes[i]["dims_m"] / 2, boxes[j]["center_m"] - boxes[j]["dims_m"] / 2)
            hi = np.minimum(boxes[i]["center_m"] + boxes[i]["dims_m"] / 2, boxes[j]["center_m"] + boxes[j]["dims_m"] / 2)
            overlap += float(np.prod(np.clip(hi - lo, 0.0, None)))
    vols = np.array([float(np.prod(b["dims_m"])) for b in boxes])
    out: dict[str, Any] = {
        "boxes": [
            {
                "prim_path": b["prim_path"],
                "cube_size_attr": round(b["cube_size_attr"], 9),
                "center_m": [round(float(x), 6) for x in b["center_m"]],
                "dims_m": [round(float(x), 6) for x in b["dims_m"]],
            }
            for b in boxes
        ],
        "pairwise_overlap_volume_m3": round(overlap, 12),
        "sum_volume_m3": round(float(vols.sum()), 10),
    }
    if overlap == 0.0:
        com = np.sum(vols[:, None] * np.array([b["center_m"] for b in boxes]), axis=0) / vols.sum()
        inertia = np.zeros((3, 3))
        for b, v in zip(boxes, vols):
            a = b["dims_m"]
            ic = np.diag([(a[1] ** 2 + a[2] ** 2) / 12.0, (a[0] ** 2 + a[2] ** 2) / 12.0, (a[0] ** 2 + a[1] ** 2) / 12.0]) * v
            d = b["center_m"] - com
            inertia += ic + v * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        iom = inertia / vols.sum()
        out["union_solid_com_offset_m"] = [round(float(x), 6) for x in com]
        out["union_solid_inertia_over_mass_m2"] = [round(float(x), 9) for x in np.diag(iom)]
        out["union_solid_offdiag_max_rel"] = round(float(np.max(np.abs(iom - np.diag(np.diag(iom)))) / np.max(np.diag(iom))), 6)
    return out


def _init_cpu_vhacd() -> tuple[Any, Any]:
    """Load Isaac Sim's CPU-only omni.convexdecomposition (V-HACD) interface without starting Kit or GPU."""
    import carb

    fw = carb.get_framework()
    fw.startup([])
    fw.load_plugins(
        loaded_file_wildcards=["carb.dictionary*", "carb.settings*", "carb.tasking*", "carb.tokens*"],
        search_paths=["/isaac-sim/kit/kernel/plugins"],
    )
    from omni.convexdecomposition.bindings import _convexdecomposition

    lib_cands = [
        os.path.join("/isaac-sim/extscache", d, "bin/libomni.convexdecomposition.plugin.so")
        for d in sorted(os.listdir("/isaac-sim/extscache"))
        if d.startswith("omni.convexdecomposition-")
    ]
    if not lib_cands or not os.path.isfile(lib_cands[0]):
        raise FileNotFoundError("libomni.convexdecomposition.plugin.so not found under /isaac-sim/extscache")
    return _convexdecomposition, _convexdecomposition.acquire_convexdecomposition_interface(library_path=lib_cands[0])


def _closed_poly_moments(verts: np.ndarray, tris_idx: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Signed volume, centroid, and unit-density 3x3 inertia tensor about the origin of a closed convex hull."""
    tris = verts[tris_idx]
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    ctr = verts.mean(axis=0)
    cross = np.cross(v1 - v0, v2 - v0)
    flip = np.einsum("ij,ij->i", cross, v0 - ctr) < 0.0
    if np.any(flip):
        v1 = v1.copy()
        v2 = v2.copy()
        v1[flip], v2[flip] = v2[flip], v1[flip]
    det = np.einsum("ij,ij->i", v0, np.cross(v1, v2))
    vol = float(np.sum(det) / 6.0)
    com = np.sum(det[:, None] * (v0 + v1 + v2), axis=0) / (24.0 * max(vol, 1e-18))
    s = v0 + v1 + v2
    cov = (
        np.einsum("n,ni,nj->ij", det, v0, v0)
        + np.einsum("n,ni,nj->ij", det, v1, v1)
        + np.einsum("n,ni,nj->ij", det, v2, v2)
        + np.einsum("n,ni,nj->ij", det, s, s)
    ) / 120.0
    i_orig = np.trace(cov) * np.eye(3) - cov
    return vol, com, i_orig


def _vhacd_collider_moments(rb: Any, xcache: Any, cd_pair: tuple[Any, Any]) -> dict[str, Any] | None:
    """Compute CPU V-HACD convexDecomposition volume/moments (plus any sibling convexHull part meshes)."""
    _cd_mod, cd_iface = cd_pair
    root_inv = xcache.GetLocalToWorldTransform(rb).GetInverse()
    decomp_hulls = 0
    tot_v = 0.0
    weighted_com = np.zeros(3, dtype=np.float64)
    i_orig_sum = np.zeros((3, 3), dtype=np.float64)
    params_rec: dict[str, Any] = {}
    part_hulls: list[dict[str, Any]] = []

    for p in Usd.PrimRange(rb, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if not (p.HasAPI(UsdPhysics.CollisionAPI) and UsdGeom.Mesh(p)):
            continue
        en_attr = p.GetAttribute("physics:collisionEnabled")
        if en_attr and en_attr.IsValid() and en_attr.Get() is False:
            continue
        ap_attr = p.GetAttribute("physics:approximation")
        ap = str(ap_attr.Get()) if (ap_attr and ap_attr.IsValid() and ap_attr.Get() is not None) else "none"
        m = UsdGeom.Mesh(p)
        pts = np.asarray(m.GetPointsAttr().Get() or [], dtype=np.float64)
        cnts = np.asarray(m.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
        idxs = np.asarray(m.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
        if not len(pts) or not len(cnts):
            continue
        mat4 = np.asarray(xcache.GetLocalToWorldTransform(p) * root_inv, dtype=np.float64)
        pts_r = (np.hstack([pts, np.ones((len(pts), 1), dtype=np.float64)]) @ mat4)[:, :3]
        tris = []
        off = 0
        for c in cnts:
            for k in range(1, int(c) - 1):
                tris.append((int(idxs[off]), int(idxs[off + k]), int(idxs[off + k + 1])))
            off += int(c)
        tris_arr = np.asarray(tris, dtype=np.int64)

        if ap == "convexDecomposition":
            def _ga(nm: str) -> Any:
                a = p.GetAttribute(f"physxConvexDecompositionCollision:{nm}")
                return a.Get() if (a and a.IsValid()) else None

            vres = int(_ga("voxelResolution") or 500000)
            errp = float(_ga("errorPercentage") if _ga("errorPercentage") is not None else 10.0)
            maxh = int(_ga("maxConvexHulls") or 32)
            hvl = int(_ga("hullVertexLimit") or 64)
            sw = bool(_ga("shrinkWrap")) if _ga("shrinkWrap") is not None else False
            params_rec = {
                "prim_path": str(p.GetPath()),
                "voxel_resolution": vres,
                "error_percentage": round(errp, 6),
                "max_convex_hulls": maxh,
                "hull_vertex_limit": hvl,
                "shrink_wrap": sw,
            }
            sm = _cd_mod.SimpleMesh()
            sm.vertices = pts_r.astype(np.float32).reshape(-1).tolist()
            sm.indices = tris_arr.astype(np.uint32).reshape(-1).tolist()
            pa = _cd_mod.Parameters()
            pa.voxel_resolution = vres
            pa.error_percentage = errp
            pa.max_convex_hull_count = maxh
            pa.max_hull_vertices = hvl
            pa.voxel_fill_mode = _cd_mod.VoxelFillMode.FLOOD_FILL
            h = cd_iface.create_vhacd()
            cd_iface.begin_vhacd(h, sm, pa)
            while not cd_iface.is_complete(h):
                time.sleep(0.005)
            cnt = int(cd_iface.get_convex_hull_count(h))
            decomp_hulls += cnt
            for hi in range(cnt):
                ch = _cd_mod.SimpleMesh()
                cd_iface.get_convex_hull(h, hi, ch)
                hv = np.asarray(ch.vertices, dtype=np.float64).reshape(-1, 3)
                ht = np.asarray(ch.indices, dtype=np.int64).reshape(-1, 3)
                v_i, c_i, io_i = _closed_poly_moments(hv, ht)
                if v_i <= 1e-14:
                    continue
                tot_v += v_i
                weighted_com += v_i * c_i
                i_orig_sum += io_i
            cd_iface.release_vhacd(h)
        elif ap == "convexHull":
            ch_part = _convex_hull_solid_and_shell_moments(pts_r)
            part_hulls.append(
                {
                    "prim_path": str(p.GetPath()),
                    "volume_m3": round(float(ch_part["outer_hull_volume_m3"]), 10),
                    "solid_com_offset_m": ch_part["solid_com_offset_m"],
                    "solid_inertia_over_mass_m2": ch_part["solid_inertia_over_mass_m2"],
                }
            )

    if decomp_hulls == 0:
        return None
    com_tot = weighted_com / max(tot_v, 1e-18)
    i_com = (i_orig_sum - tot_v * (float(np.dot(com_tot, com_tot)) * np.eye(3) - np.outer(com_tot, com_tot))) / max(tot_v, 1e-18)
    i_diag = np.diag(i_com)
    out: dict[str, Any] = {
        "parameters": params_rec,
        "hull_count": decomp_hulls,
        "decomp_volume_m3": round(tot_v, 10),
        "solid_com_offset_m": [round(float(x), 6) for x in com_tot],
        "solid_inertia_over_mass_m2": [round(float(x), 9) for x in i_diag],
        "solid_offdiag_max_rel": round(float(np.max(np.abs(i_com - np.diag(i_diag))) / max(float(np.max(i_diag)), 1e-18)), 6),
    }
    if part_hulls:
        pvols = np.array([ph["volume_m3"] for ph in part_hulls], dtype=np.float64)
        pcoms = np.array([ph["solid_com_offset_m"] for ph in part_hulls], dtype=np.float64)
        pcom_u = np.sum(pvols[:, None] * pcoms, axis=0) / float(pvols.sum())
        pi_u = np.zeros((3, 3), dtype=np.float64)
        for ph, pv, pc in zip(part_hulls, pvols, pcoms):
            d = pc - pcom_u
            pi_u += np.diag(ph["solid_inertia_over_mass_m2"]) * pv + pv * (float(np.dot(d, d)) * np.eye(3) - np.outer(d, d))
        pi_u /= float(pvols.sum())
        out["convex_hull_part_colliders"] = {
            "parts": part_hulls,
            "sum_volume_m3": round(float(pvols.sum()), 10),
            "union_solid_com_offset_m": [round(float(x), 6) for x in pcom_u],
            "union_solid_inertia_over_mass_m2": [round(float(x), 9) for x in np.diag(pi_u)],
        }
    return out


def _rigid_model_facts(assets_root: str, aid: str, *, vhacd_pair: tuple[Any, Any] | None = None) -> dict[str, Any]:
    """Generic per-model facts for any Rigid asset: collider records, visual-mesh surface moments, box-primitive
    collider moments and outer-hull moments, all in the rigid-body root frame."""
    rdir = os.path.join(assets_root, "Object/RoboDojo", aid)
    usd_path = _find_usd(rdir)
    meta_path = os.path.join(rdir, "metadata.json")
    with open(meta_path, encoding="utf-8") as mf:
        meta = json.load(mf)
    st = Usd.Stage.Open(usd_path)
    rb = st.GetDefaultPrim()
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    col_info = _extract_collider_aabb(st, rb)
    mesh_rows = []
    vis_tris = []
    disabled_col_tris = []
    enabled_col_tris = []
    sdf_col_tris = []
    vis_shas: set[str] = set()
    enabled_col_shas: set[str] = set()
    sdf_col_shas: set[str] = set()
    dedup_tris = []
    seen_shas: set[str] = set()
    all_pts = []
    for p in Usd.PrimRange(rb, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if not UsdGeom.Mesh(p):
            continue
        tris = _mesh_triangles_in_root(p, rb, xcache)
        if not len(tris):
            continue
        is_col = p.HasAPI(UsdPhysics.CollisionAPI)
        en_attr = p.GetAttribute("physics:collisionEnabled")
        enabled = not (en_attr and en_attr.IsValid() and en_attr.Get() is False)
        ap_attr = p.GetAttribute("physics:approximation")
        ap = str(ap_attr.Get()) if (ap_attr and ap_attr.IsValid() and ap_attr.Get() is not None) else "none"
        psha = hashlib.sha256(np.ascontiguousarray(tris, dtype=np.float64).tobytes()).hexdigest()
        mesh_rows.append(
            {
                "prim_path": str(p.GetPath()),
                "collision_api": bool(is_col),
                "collision_enabled": bool(is_col and enabled),
                "points_sha256": psha,
            }
        )
        all_pts.append(tris.reshape(-1, 3))
        if psha not in seen_shas:
            seen_shas.add(psha)
            dedup_tris.append(tris)
        if not is_col:
            vis_tris.append(tris)
            vis_shas.add(psha)
        elif not enabled:
            disabled_col_tris.append(tris)
        if is_col and enabled and psha not in enabled_col_shas:
            enabled_col_shas.add(psha)
            enabled_col_tris.append(tris)
        if is_col and enabled and ap == "sdf" and psha not in sdf_col_shas:
            sdf_col_shas.add(psha)
            sdf_col_tris.append(tris)
    phys = meta.get("physics") or {}
    out: dict[str, Any] = {
        "usd_relpath": f"Assets/Object/RoboDojo/{aid}/{os.path.basename(usd_path)}",
        "usd_sha256": _sha256_file(usd_path),
        "metadata_sha256": _sha256_file(meta_path),
        "metadata_aligned_bbox_min_m": [float(x) for x in np.min(np.asarray(meta["geometry"]["aligned_bbox"]["vertices"]), axis=0)],
        "metadata_aligned_bbox_extents_m": [float(x) for x in meta["geometry"]["aligned_bbox"]["extents"]],
        "metadata_declared_mass_kg": phys.get("mass"),
        "metadata_declared_friction": phys.get("friction"),
        "collider_union_min_m": col_info["collider_union_min_m"],
        "collider_union_max_m": col_info["collider_union_max_m"],
        "collider_union_extents_m": col_info["collider_union_extents_m"],
        "enabled_gprim_collider_count": col_info["enabled_gprim_collider_count"],
        "colliders": col_info["colliders"],
        "meshes": mesh_rows,
    }
    if vis_tris:
        out["visual_surface"] = _surface_mesh_moments(np.vstack(vis_tris))
    elif disabled_col_tris:
        out["visual_surface"] = _surface_mesh_moments(np.vstack(disabled_col_tris))
    elif dedup_tris:
        out["visual_surface"] = _surface_mesh_moments(np.vstack(dedup_tris))
    if enabled_col_tris and vis_shas and enabled_col_shas != vis_shas:
        out["collider_mesh_surface"] = _surface_mesh_moments(np.vstack(enabled_col_tris))
    if sdf_col_tris and len(enabled_col_shas) > len(sdf_col_shas):
        out["sdf_primary_surface"] = _surface_mesh_moments(np.vstack(sdf_col_tris))
    box_info = _box_primitive_colliders(rb)
    if box_info is not None:
        out["box_primitive_colliders"] = box_info
    if all_pts:
        out["outer_hull_all_mesh_points"] = _convex_hull_solid_and_shell_moments(np.vstack(all_pts))
    if vhacd_pair is not None:
        cd_info = _vhacd_collider_moments(rb, xcache, vhacd_pair)
        if cd_info is not None:
            out["convex_decomposition"] = cd_info
    return out


def _extract_collider_aabb(stage: Any, root_prim: Any) -> dict[str, Any]:
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    root_inv = xcache.GetLocalToWorldTransform(root_prim).GetInverse()
    bc = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide, UsdGeom.Tokens.render],
        useExtentsHint=False,
    )
    mins = np.array([np.inf, np.inf, np.inf], dtype=np.float64)
    maxs = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64)
    bbox_cache_mins = np.array([np.inf, np.inf, np.inf], dtype=np.float64)
    bbox_cache_maxs = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64)
    col_records = []
    for p in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if p.HasAPI(UsdPhysics.CollisionAPI) and bool(UsdGeom.Gprim(p)):
            en_attr = p.GetAttribute("physics:collisionEnabled")
            if en_attr and en_attr.IsValid() and en_attr.Get() is False:
                continue
            ap_attr = p.GetAttribute("physics:approximation")
            approx = str(ap_attr.Get()) if (ap_attr and ap_attr.IsValid() and ap_attr.Get() is not None) else "none"
            rng = bc.ComputeRelativeBound(p, root_prim).ComputeAlignedRange()
            if not rng.IsEmpty():
                bbox_cache_mins = np.minimum(bbox_cache_mins, np.array(rng.GetMin(), dtype=np.float64))
                bbox_cache_maxs = np.maximum(bbox_cache_maxs, np.array(rng.GetMax(), dtype=np.float64))
            mesh = UsdGeom.Mesh(p)
            pts_raw = mesh.GetPointsAttr().Get() if mesh else None
            if pts_raw is not None and len(pts_raw) > 0:
                pts = np.asarray(pts_raw, dtype=np.float64)
                mat4 = np.asarray(xcache.GetLocalToWorldTransform(p) * root_inv, dtype=np.float64)
                pts_r = (np.hstack([pts, np.ones((len(pts), 1), dtype=np.float64)]) @ mat4)[:, :3]
                lo, hi = pts_r.min(axis=0), pts_r.max(axis=0)
            elif not rng.IsEmpty():
                lo, hi = np.array(rng.GetMin(), dtype=np.float64), np.array(rng.GetMax(), dtype=np.float64)
            else:
                continue
            mins = np.minimum(mins, lo)
            maxs = np.maximum(maxs, hi)
            col_records.append(
                {
                    "prim_path": str(p.GetPath()),
                    "approximation": approx,
                    "local_min_m": [round(float(x), 6) for x in lo],
                    "local_max_m": [round(float(x), 6) for x in hi],
                    "extents_m": [round(float(hi[k] - lo[k]), 6) for k in range(3)],
                }
            )
    if not col_records:
        return {
            "enabled_gprim_collider_count": 0,
            "collider_union_min_m": None,
            "collider_union_max_m": None,
            "collider_union_extents_m": None,
            "bbox_cache_rotated_obb_extents_m": None,
            "colliders": [],
        }
    return {
        "enabled_gprim_collider_count": len(col_records),
        "collider_union_min_m": [round(float(x), 6) for x in mins],
        "collider_union_max_m": [round(float(x), 6) for x in maxs],
        "collider_union_extents_m": [round(float(maxs[k] - mins[k]), 6) for k in range(3)],
        "bbox_cache_rotated_obb_extents_m": [round(float(bbox_cache_maxs[k] - bbox_cache_mins[k]), 6) for k in range(3)],
        "colliders": col_records,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--assets", required=True, help="Path to RoboDojo Assets directory at HF revision 1127bda9")
    ap.add_argument("--out", required=True, help="Output JSON path")
    ap.add_argument(
        "--models",
        default="",
        help="Comma-separated Rigid asset ids (e.g. Rigid/cube/00004,Rigid/t/00000) to extract generically into "
        "'rigid_objects'; with it, the default bottle/geometry/robot sections are skipped unless --with-defaults",
    )
    ap.add_argument("--with-defaults", action="store_true", help="With --models, also extract the default sections")
    ap.add_argument(
        "--vhacd-models",
        default="",
        help="Optional comma-separated Rigid asset ids for which CPU V-HACD convex_decomposition volume/moments are extracted",
    )
    args = ap.parse_args()

    assets_root = os.path.abspath(args.assets)
    models = [m.strip() for m in args.models.split(",") if m.strip()]
    vhacd_set = {m.strip() for m in args.vhacd_models.split(",") if m.strip()}
    vhacd_pair = _init_cpu_vhacd() if vhacd_set else None
    if models:
        rigid_ids = [aid for aid in models if not aid.startswith("Geometry/")]
        geom_ids = [aid for aid in models if aid.startswith("Geometry/")]
        rigid_out = {
            aid: _rigid_model_facts(assets_root, aid, vhacd_pair=(vhacd_pair if aid in vhacd_set else None))
            for aid in rigid_ids
        }
        geom_models_out: dict[str, Any] = {}
        for gaid in geom_ids:
            gf = _rigid_model_facts(assets_root, gaid)
            meta_ext = gf["metadata_aligned_bbox_extents_m"]
            col_ext = gf["collider_union_extents_m"]
            gf["collider_vs_metadata_rel_diff"] = (
                [round(abs(col_ext[k] - meta_ext[k]) / max(meta_ext[k], 1e-9), 6) for k in range(3)]
                if col_ext is not None
                else None
            )
            geom_models_out[gaid] = gf
        if not args.with_defaults:
            payload_m: dict[str, Any] = {
                "schema": "usd_geometry_facts_v1",
                "hf_dataset": "RoboDojo-Benchmark/RoboDojo",
                "hf_dataset_revision": HF_REVISION,
                "generator": "audit/tools/extract_usd_geometry_facts.py --models",
                "rigid_objects": rigid_out,
            }
            if geom_models_out:
                payload_m["geometry_objects"] = geom_models_out
            os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
            encoded = (json.dumps(payload_m, indent=2) + "\n").encode("utf-8")
            if args.out.endswith(".gz"):
                with open(args.out, "wb") as bf:
                    with gzip.GzipFile(filename="", mode="wb", fileobj=bf, compresslevel=9, mtime=0) as gf:
                        gf.write(encoded)
            else:
                with open(args.out, "wb") as bf:
                    bf.write(encoded)
            msg = f"{len(rigid_out)} rigid objects" + (f", {len(geom_models_out)} geometry objects" if geom_models_out else "")
            print(f"Wrote {args.out} ({msg})")
            return
    else:
        rigid_out = {}
    bottles_out: dict[str, Any] = {}

    for idx in BOTTLE_INDICES:
        aid = f"Rigid/bottle/{idx:05d}"
        bdir = os.path.join(assets_root, "Object/RoboDojo", aid)
        usd_path = _find_usd(bdir)
        meta_path = os.path.join(bdir, "metadata.json")
        with open(meta_path, encoding="utf-8") as mf:
            meta = json.load(mf)
        st = Usd.Stage.Open(usd_path)
        rb = st.GetDefaultPrim()
        xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
        root_inv = xcache.GetLocalToWorldTransform(rb).GetInverse()
        all_pts = []
        for p in Usd.PrimRange(rb, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            m = UsdGeom.Mesh(p)
            if m:
                pts = np.asarray(m.GetPointsAttr().Get() or [], dtype=np.float64)
                if len(pts):
                    mat4 = np.asarray(xcache.GetLocalToWorldTransform(p) * root_inv, dtype=np.float64)
                    pts_r = (np.hstack([pts, np.ones((len(pts), 1), dtype=np.float64)]) @ mat4)[:, :3]
                    all_pts.append(pts_r)
        pts_cat = np.vstack(all_pts)
        col_info = _extract_collider_aabb(st, rb)
        hull_info = _convex_hull_solid_and_shell_moments(pts_cat)
        bottles_out[aid] = {
            "usd_relpath": f"Assets/Object/RoboDojo/{aid}/{os.path.basename(usd_path)}",
            "usd_sha256": _sha256_file(usd_path),
            "metadata_sha256": _sha256_file(meta_path),
            "metadata_aligned_bbox_extents_m": [float(x) for x in meta["geometry"]["aligned_bbox"]["extents"]],
            "collider_aabb": {
                "enabled_gprim_collider_count": col_info["enabled_gprim_collider_count"],
                "collider_union_extents_m": col_info["collider_union_extents_m"],
                "approximations": sorted({c["approximation"] for c in col_info["colliders"]}),
            },
            **hull_info,
        }

    geometry_out: dict[str, Any] = {}
    for gaid in ("Geometry/dustbin/00000", "Geometry/camera_stand/00000"):
        gdir = os.path.join(assets_root, "Object/RoboDojo", gaid)
        usd_path = _find_usd(gdir)
        meta_path = os.path.join(gdir, "metadata.json")
        with open(meta_path, encoding="utf-8") as mf:
            meta = json.load(mf)
        st = Usd.Stage.Open(usd_path)
        rb = st.GetDefaultPrim()
        col_info = _extract_collider_aabb(st, rb)
        meta_ext = [float(x) for x in meta["geometry"]["aligned_bbox"]["extents"]]
        col_ext = col_info["collider_union_extents_m"]
        rel_diff = [round(abs(col_ext[k] - meta_ext[k]) / max(meta_ext[k], 1e-9), 6) for k in range(3)]
        geometry_out[gaid] = {
            "usd_relpath": f"Assets/Object/RoboDojo/{gaid}/{os.path.basename(usd_path)}",
            "usd_sha256": _sha256_file(usd_path),
            "metadata_sha256": _sha256_file(meta_path),
            "metadata_aligned_bbox_extents_m": meta_ext,
            "collider_union_min_m": col_info["collider_union_min_m"],
            "collider_union_max_m": col_info["collider_union_max_m"],
            "collider_union_extents_m": col_ext,
            "bbox_cache_rotated_obb_extents_m": col_info["bbox_cache_rotated_obb_extents_m"],
            "collider_vs_metadata_rel_diff": rel_diff,
            "enabled_gprim_collider_count": col_info["enabled_gprim_collider_count"],
            "colliders": col_info["colliders"],
        }

    x5_usd_path = os.path.join(assets_root, "Robots/x5/ARX.usd")
    st_x5 = Usd.Stage.Open(x5_usd_path)
    x5_links = []
    for p in Usd.PrimRange(st_x5.GetPseudoRoot(), Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            cols = [
                c
                for c in Usd.PrimRange(p, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
                if c.HasAPI(UsdPhysics.CollisionAPI)
            ]
            ap_list = sorted(
                {
                    str(c.GetAttribute("physics:approximation").Get())
                    for c in cols
                    if c.GetAttribute("physics:approximation") and c.GetAttribute("physics:approximation").Get() is not None
                }
            )
            x5_links.append(
                {
                    "link": p.GetName(),
                    "prim_path": str(p.GetPath()),
                    "collision_api_count": len(cols),
                    "approximations": ap_list,
                }
            )

    payload = {
        "schema": "usd_geometry_facts_v1",
        "hf_dataset": "RoboDojo-Benchmark/RoboDojo",
        "hf_dataset_revision": HF_REVISION,
        "generator": "audit/tools/extract_usd_geometry_facts.py",
        "bottles": bottles_out,
        "geometry_objects": geometry_out,
        "robots": {
            "x5": {
                "usd_relpath": "Assets/Robots/x5/ARX.usd",
                "usd_sha256": _sha256_file(x5_usd_path),
                "links": x5_links,
            }
        },
    }
    if rigid_out:
        payload["rigid_objects"] = rigid_out

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")
    print(f"Wrote {args.out} ({len(bottles_out)} bottles, {len(geometry_out)} geometry objects, {len(x5_links)} x5 links)")


if __name__ == "__main__":
    main()
