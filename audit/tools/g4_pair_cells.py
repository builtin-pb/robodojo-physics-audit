"""Friction-sweep cells for swap_T's three contact pairs at fully corrected physics (stdlib, deterministic).

Usage: python3 -I audit/tools/g4_pair_cells.py <docs/params dir> <out cells.json> [<lane3 fit_population.json.gz>]

swap_T's bodies that touch carry three material classes: the finger pads (pad_hard), the table (table_laminate) and
the T blocks (abs_toy_plastic). Under `average` combine every pair is the mean of its two unary values, so the three
pairs pad-table (PL), pad-T (PA) and T-table (AL) fix the three unaries exactly:
    pad = PL + PA - AL,  table = PL + AL - PA,  T = PA + AL - PL.
A cell moves one pair to an end of its band and holds the other two at the map's realised values. A cell is feasible
when every unary is >= 0 and kinetic <= static per class (PhysX and the loader). Where the exact cell is infeasible,
the script says so and builds the nearest declared alternative:
- `kinetic_floor`: the moved pair's kinetic is raised to the smallest value that keeps kinetic <= static per class;
- `pad_only`: only the pad unary moves (pad-T at the end; pad-table moves with it), as `--pad_friction` did for bottles.
Each cell's `class_friction` string is the harness's `--class_friction` argument.

With the lane-3 population fit (`audit/out/lane3_dryrun/out/fit_population.json.gz` on main, #59), two more cells:
- `PL_v3`: pad-table at the population joint fit's realised value, the other two pairs held at v2b (exact solve);
- `MAP_v3`: the three classes at the population fit's unaries (all three pairs at that map's values).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

CLS = {"pad": "pad_hard", "table": "table_laminate", "T": "abs_toy_plastic"}
PAIRS = {"PL": "pad_hard__table_laminate", "PA": "abs_toy_plastic__pad_hard", "AL": "abs_toy_plastic__table_laminate"}


def unaries(PL, PA, AL):
    return {"pad": PL + PA - AL, "table": PL + AL - PA, "T": PA + AL - PL}


def feasible(us, uk):
    return all(us[c] >= 0.0 and uk[c] >= 0.0 and uk[c] <= us[c] + 1e-12 for c in us)


def main(params_dir: str, out_path: str, v3_path: str | None = None) -> int:
    classes = json.load(open(Path(params_dir) / "classes.json"))
    pairs = json.load(open(Path(params_dir) / "pairs.json"))["pairs"]
    u_s, u_k = classes["unary_values"]["static"], classes["unary_values"]["kinetic"]
    for c in CLS.values():
        assert classes["unary_values"]["friction_combine_mode"][c] == "average", c
    nom = {"s": {"pad": u_s[CLS["pad"]], "table": u_s[CLS["table"]], "T": u_s[CLS["T"]]},
           "k": {"pad": u_k[CLS["pad"]], "table": u_k[CLS["table"]], "T": u_k[CLS["T"]]}}
    real = {}
    for kind in ("s", "k"):
        n = nom[kind]
        real[kind] = {"PL": (n["pad"] + n["table"]) / 2, "PA": (n["pad"] + n["T"]) / 2, "AL": (n["T"] + n["table"]) / 2}
    bands = {p: {"s": pairs[k]["mu_static"]["range"], "k": pairs[k]["mu_kinetic"]["range"],
                 "status_s": pairs[k]["mu_static"].get("status"), "status_k": pairs[k]["mu_kinetic"].get("status")}
             for p, k in PAIRS.items()}
    cells = []
    for pair in ("PL", "PA", "AL"):
        for end, idx in (("lo", 0), ("hi", 1)):
            tgt = {"s": dict(real["s"]), "k": dict(real["k"])}
            tgt["s"][pair] = bands[pair]["s"][idx]
            tgt["k"][pair] = bands[pair]["k"][idx]
            us, uk = unaries(**tgt["s"]), unaries(**tgt["k"])
            mode, note = "exact", "moved pair at both band ends; the other two pairs held at the map's realised values"
            if not feasible(us, uk) and all(us[c] >= 0.0 for c in us):
                # the static cell is feasible; scan the moved pair's kinetic for the feasible value nearest its band end
                kb = bands[pair]["k"]
                grid = [kb[0] + (kb[1] - kb[0]) * j / 20000.0 for j in range(20001)]
                ok = [x for x in grid if feasible(us, unaries(**dict(tgt["k"], **{pair: x})))]
                if ok:
                    best = min(ok, key=lambda x: abs(x - kb[idx]))
                    tgt["k"][pair] = round(best, 6)
                    uk = unaries(**tgt["k"])
                    mode = "kinetic_floor"
                    note = (f"exact cell infeasible at the kinetic band end {kb[idx]} (a class would need kinetic > static); "
                            f"kinetic set to {tgt['k'][pair]}, the nearest value that keeps kinetic <= static per class; "
                            "the other two pairs held at the map's realised values")
            if not feasible(us, uk):
                assert pair == "PA", pair
                # only the pad unary moves: pad-T at the band end, pad-table follows; T-table held
                us = {"pad": 2 * tgt["s"]["PA"] - nom["s"]["T"], "table": nom["s"]["table"], "T": nom["s"]["T"]}
                uk = {"pad": 2 * tgt["k"]["PA"] - nom["k"]["T"], "table": nom["k"]["table"], "T": nom["k"]["T"]}
                assert feasible(us, uk), (pair, end, us, uk)
                mode = "pad_only"
                note = ("exact cell infeasible (holding pad-table and T-table needs a negative or inverted table unary); "
                        "only the pad unary moves, so pad-table moves with pad-T (dormant only if the pads never touch the table)")
            us = {c: round(v, 6) for c, v in us.items()}
            uk = {c: round(v, 6) for c, v in uk.items()}
            realised = {}
            for kind, u in (("s", us), ("k", uk)):
                realised[kind] = {"PL": round((u["pad"] + u["table"]) / 2, 6), "PA": round((u["pad"] + u["T"]) / 2, 6),
                                  "AL": round((u["T"] + u["table"]) / 2, 6)}
            changed = [c for c in CLS if abs(us[c] - nom["s"][c]) > 1e-9 or abs(uk[c] - nom["k"][c]) > 1e-9]
            cf = ";".join(f"{CLS[c]}={us[c]:.6f},{uk[c]:.6f}" for c in ("pad", "table", "T") if c in changed)
            cells.append({"cell": f"{pair}_{end}", "pair": PAIRS[pair], "end": end, "mode": mode, "note": note,
                          "unary_static": {CLS[c]: us[c] for c in CLS}, "unary_kinetic": {CLS[c]: uk[c] for c in CLS},
                          "realised_static": {PAIRS[p]: v for p, v in realised["s"].items()},
                          "realised_kinetic": {PAIRS[p]: v for p, v in realised["k"].items()},
                          "class_friction": cf})
    v3_info = None
    if v3_path:
        import gzip
        import hashlib

        raw = open(v3_path, "rb").read()
        v3 = json.loads(gzip.decompress(raw))
        v3s = {c: float(v3["u_static"][CLS[c]]) for c in CLS}
        v3k = {c: float(v3["u_kinetic"][CLS[c]]) for c in CLS}
        v3_info = {"path": "audit/out/lane3_dryrun/out/fit_population.json.gz (main, commit 8dd1a6c)",
                   "sha256": hashlib.sha256(raw).hexdigest(), "u_static": {CLS[c]: v for c, v in v3s.items()},
                   "u_kinetic": {CLS[c]: v for c, v in v3k.items()}}
        pl3 = {"s": (v3s["pad"] + v3s["table"]) / 2, "k": (v3k["pad"] + v3k["table"]) / 2}
        extra = []
        tgt_s, tgt_k = dict(real["s"], PL=pl3["s"]), dict(real["k"], PL=pl3["k"])
        us, uk = unaries(**tgt_s), unaries(**tgt_k)
        assert feasible(us, uk)
        extra.append(("PL_v3", PAIRS["PL"], "v3", "exact", "pad-table at the population joint fit's realised value (#59 lane 3); the other two pairs held at v2b", us, uk))
        extra.append(("MAP_v3", "all three", "v3", "map", "pad, table and T unaries of the population joint fit (#59 lane 3): every pair at that map's value", v3s, v3k))
        for name, pair, end, mode, note, us, uk in extra:
            us = {c: round(v, 6) for c, v in us.items()}
            uk = {c: round(v, 6) for c, v in uk.items()}
            realised = {}
            for kind, u in (("s", us), ("k", uk)):
                realised[kind] = {"PL": round((u["pad"] + u["table"]) / 2, 6), "PA": round((u["pad"] + u["T"]) / 2, 6),
                                  "AL": round((u["T"] + u["table"]) / 2, 6)}
            cf = ";".join(f"{CLS[c]}={us[c]:.6f},{uk[c]:.6f}" for c in ("pad", "table", "T"))
            cells.append({"cell": name, "pair": pair, "end": end, "mode": mode, "note": note,
                          "unary_static": {CLS[c]: us[c] for c in CLS}, "unary_kinetic": {CLS[c]: uk[c] for c in CLS},
                          "realised_static": {PAIRS[p]: v for p, v in realised["s"].items()},
                          "realised_kinetic": {PAIRS[p]: v for p, v in realised["k"].items()},
                          "class_friction": cf})
    out = {"tool": "audit/tools/g4_pair_cells.py", "params_dir": params_dir, "lane3_fit": v3_info,
           "nominal_unary": {kind: {CLS[c]: v for c, v in nom[kind].items()} for kind in nom},
           "map_realised": {kind: {PAIRS[p]: round(v, 6) for p, v in real[kind].items()} for kind in real},
           "bands": {PAIRS[p]: b for p, b in bands.items()}, "cells": cells}
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    for c in cells:
        print(f"{c['cell']:7s} {c['mode']:13s} PL {c['realised_static'][PAIRS['PL']]:.4f}/{c['realised_kinetic'][PAIRS['PL']]:.4f} "
              f"PA {c['realised_static'][PAIRS['PA']]:.4f}/{c['realised_kinetic'][PAIRS['PA']]:.4f} "
              f"AL {c['realised_static'][PAIRS['AL']]:.4f}/{c['realised_kinetic'][PAIRS['AL']]:.4f}  --class_friction '{c['class_friction']}'")
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:4]))
