#!/usr/bin/env python3
"""Generate earbuds COM/inertia sensitivity parameter tables from docs/params (#18, #200).

Usage:
  python3 -I audit/out/63/population/coverage_t1/earbuds_probe_params.py docs/params <out_root>

Writes two parameter-table copies under <out_root>:
  - <out_root>/usd_surface_shell: replaces Rigid/earbuds/0000{0,1,2} COM and I/m with the USD concave-mesh
    area-weighted thin-shell centroid and I/m (`visual_surface.{surface_shell_com_offset_m, surface_shell_inertia_over_mass_m2}`
    in `audit/out/63/usd_geometry_facts_population_t1.json`).
  - <out_root>/vhacd_solid: replaces Rigid/earbuds/0000{0,1,2} COM and I/m with the uniform-density 64-hull V-HACD
    decomposition centroid and I/m at each USD's authored `PhysxConvexDecompositionCollisionAPI` settings (head-heavy
    end of a stem wireless earbud, matching shipped parse-time COM within 0.2 mm).

Every other value across classes.json, pairs.json, assets.json, and robots.json stays byte-identical to docs/params.
"""
from __future__ import annotations

import copy
import hashlib
import json
import shutil
import sys
from pathlib import Path

VARIANTS: dict[str, dict[str, tuple[list[float], list[float]]]] = {
    "usd_surface_shell": {
        "Rigid/earbuds/00000": ([-3.3e-05, -0.002655, -0.001912], [0.00043784, 0.00027926, 0.00043522]),
        "Rigid/earbuds/00001": ([0.002266, -0.000514, 0.007834], [0.00015946, 0.00023488, 0.00017944]),
        "Rigid/earbuds/00002": ([-1.8e-05, -0.000851, 0.004962], [0.00017024, 0.00016449, 0.00007239]),
    },
    "vhacd_solid": {
        "Rigid/earbuds/00000": ([0.000102, -0.004539, -0.001454], [0.00028627, 0.00016723, 0.00028761]),
        "Rigid/earbuds/00001": ([-0.002275, -0.001031, 0.008789], [0.00011803, 0.00016616, 0.00011489]),
        "Rigid/earbuds/00002": ([0.000137, -0.000790, 0.007002], [0.00010365, 0.00009901, 0.00004613]),
    },
}


def table_dir_sha256(d: Path) -> str:
    h = hashlib.sha256()
    for fn in ("classes.json", "pairs.json", "assets.json", "robots.json"):
        h.update(fn.encode("utf-8") + b"\0")
        h.update((d / fn).read_bytes() + b"\0")
    return h.hexdigest()


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("Usage: python3 -I earbuds_probe_params.py <docs/params> <out_root>")
    src = Path(sys.argv[1])
    out_root = Path(sys.argv[2])
    out_root.mkdir(parents=True, exist_ok=True)

    assets_base = json.loads((src / "assets.json").read_text())
    for vname, overrides in sorted(VARIANTS.items()):
        vdir = out_root / vname
        vdir.mkdir(parents=True, exist_ok=True)
        for fn in ("classes.json", "pairs.json", "robots.json"):
            shutil.copyfile(src / fn, vdir / fn)
        adoc = copy.deepcopy(assets_base)
        for aid, (com_v, iom_v) in overrides.items():
            adoc["assets"][aid]["com_offset_by_fill"]["empty"]["value"] = list(com_v)
            adoc["assets"][aid]["inertia_over_mass_by_fill"]["empty"]["value"] = list(iom_v)
        (vdir / "assets.json").write_text(json.dumps(adoc, indent=2) + "\n")
        print(f"{vname}: {vdir} (dir_sha256={table_dir_sha256(vdir)[:16]}...)")


if __name__ == "__main__":
    main()
