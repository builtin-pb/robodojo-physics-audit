import json, sys
OUT = sys.argv[1]
dec = ("pad_hard__pet_plastic", "pad_hard__table_laminate", "abs_toy_plastic__pad_hard", "hardwood__table_laminate",
       "abs_toy_plastic__table_laminate", "hardwood__polyolefin_plastic")
short = {"pad_hard__pet_plastic": "pad-PET", "pad_hard__table_laminate": "pad-table", "abs_toy_plastic__pad_hard": "pad-T",
         "hardwood__table_laminate": "cube-table", "abs_toy_plastic__table_laminate": "T-table", "hardwood__polyolefin_plastic": "cup-cube"}
for f in ("fit_tiers_T0_T1_drop_.json", "fit_tiers_T0_T1_T2_drop_.json", "fit_population.json", "fit_standard_only.json", "fit_population_drop_foam_rubber_elastomer.json",
          "fit_population_drop_rubber_elastomer.json", "fit_population_drop_foam.json"):
    r = json.load(open(OUT + "/" + f)); dp = r["done_pairs"]
    parts = []
    for k in dec:
        d = dp[k]
        parts.append(f"{short[k]} {d['joint'][0]-d['v2b'][0]:+.4f}/{d['joint'][1]-d['v2b'][1]:+.4f}")
    mx = max(max(abs(d['joint'][0]-d['v2b'][0]), abs(d['joint'][1]-d['v2b'][1])) for d in dp.values())
    print(f"{f:50s} pad_hard {r['u_static']['pad_hard']:.4f}/{r['u_kinetic']['pad_hard']:.4f} table {r['u_static']['table_laminate']:.4f} alu {r['u_static']['aluminium']:.4f} | " + "; ".join(parts) + f" | max|d| {mx:.4f}")
