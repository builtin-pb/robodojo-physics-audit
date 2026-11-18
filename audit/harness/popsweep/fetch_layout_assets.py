#!/usr/bin/env python3
"""Fetch what a set of RoboDojo evaluation layouts needs from the Hugging Face dataset, at a pinned revision.

    HF_REV=<sha> python3 -I fetch_layout_assets.py <asset root> <layout json>... [--dry-run]

Per instance in Rigid, Dynamic, Geometry, Articulation, Garment, Fluid: the whole model folder
`Assets/Object/RoboDojo/<Kind>/<cat>/<idx:05d>` (`Clutter` when `type == "cluttered"`, falling back to the
section folder when the Clutter folder has no `object.usdz`, as `layout_manager.py:183-188` does). Room folders;
only the `.mdl` files of table and ground material folders (textures are visual only); `visual_usd_path` and
`visual_mdl_path` files. The background HDR is skipped (visual only, as in every harness run so far).
Standard library only; downloads data files, runs no RoboDojo code. Existing files are kept.
"""
import json, os, sys, urllib.parse as P, urllib.request as U

REV = os.environ.get("HF_REV", "1127bda94f172d757386945af3eb3a6754d1a4e6")
API = f"https://huggingface.co/api/datasets/RoboDojo-Benchmark/RoboDojo/tree/{REV}/"
RES = f"https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/resolve/{REV}/"
SECTIONS = ("Rigid", "Dynamic", "Geometry", "Articulation", "Garment", "Fluid")
args = [a for a in sys.argv[1:] if not a.startswith("--")]
DRY = "--dry-run" in sys.argv
ROOT, LAYOUTS = args[0], args[1:]


def tree(path, recursive=True):
    out, url = [], API + P.quote(path) + ("?recursive=true" if recursive else "")
    while url:
        with U.urlopen(url) as r:
            out += json.load(r)
            nxt = [s.split(";")[0].strip()[1:-1] for s in (r.headers.get("Link") or "").split(",") if 'rel="next"' in s]
        url = nxt[0] if nxt else None
    return [e for e in out if e["type"] == "file"]


def get(path):
    dst = os.path.join(ROOT, path)
    if os.path.exists(dst):
        return 0
    if DRY:
        print("would fetch", path)
        return 0
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    U.urlretrieve(RES + P.quote(path), dst + ".part")
    os.replace(dst + ".part", dst)
    return os.path.getsize(dst)


folders, mdl_folders, files = set(), set(), set()
for lp in LAYOUTS:
    lay = json.load(open(lp))
    for sec in SECTIONS:
        for cat, insts in (lay.get(sec) or {}).items():
            for inst in insts:
                idx = int(inst["category_idx"])
                kind = "Clutter" if inst.get("type") == "cluttered" else sec
                folders.add((f"Assets/Object/RoboDojo/{kind}/{cat}/{idx:05d}", f"Assets/Object/RoboDojo/{sec}/{cat}/{idx:05d}"))
                for k in ("visual_usd_path", "visual_mdl_path"):
                    v = (inst.get("visual") or {}).get(k)
                    if isinstance(v, str) and v:
                        for pre in ("$RoboDojo_ASSETS/", "$Robodojo_ASSETS/"):
                            v = v.replace(pre, "Assets/")
                        files.add(v.lstrip("./"))
    if isinstance(lay.get("Room"), dict) and lay["Room"].get("default"):
        folders.add((f"Assets/Room/{lay['Room']['default']}", None))
    if isinstance(lay.get("Table"), dict) and lay["Table"].get("default"):
        mdl_folders.add(f"Assets/Material/{lay['Table']['default']}")
    g = ((lay.get("Ground") or {}).get("materials") or {}).get("default")
    if g:
        mdl_folders.add(f"Assets/Material/{g}")
total = 0
for folder, fallback in sorted(folders, key=lambda x: x[0]):
    names = [e["path"] for e in tree(folder)]
    for p in names:
        total += get(p)
    if fallback and folder != fallback and not any(p.endswith("/object.usdz") for p in names):
        for e in tree(fallback):
            total += get(e["path"])
    print("ok", folder, len(names), "files", round(total / 1e6, 1), "MB so far", flush=True)
for folder in sorted(mdl_folders):
    for e in tree(folder, recursive=False):
        if e["path"].endswith(".mdl"):
            total += get(e["path"])
    print("ok", folder, "(.mdl only)", flush=True)
for p in sorted(files):
    try:
        total += get(p)
        print("ok", p, flush=True)
    except Exception as ex:
        print("MISSING", p, repr(ex)[:120], flush=True)
print("done", len(folders), "folders", len(mdl_folders), "material folders", len(files), "visual files", round(total / 1e6, 1), "MB new")
