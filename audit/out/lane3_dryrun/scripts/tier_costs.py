import json, sys, collections
R, OUT = sys.argv[1], sys.argv[2]
pc = json.load(open(OUT + '/per_config.json'))
cen = json.load(open(OUT + '/census_pairs.json'))
local = json.load(open(OUT + '/asset_local.json'))
v2b = json.load(open(R + '/docs/params/assets.json'))['assets']
cum_rows, cum_dl = set(), set()
for tier in ('T0', 'T1', 'T2', 'T3'):
    ts = sorted(t for t, x in pc.items() if x['tier'] == tier)
    rows, dl, dyn = set(), set(), set()
    for t in ts:
        for mid, m in cen[t]['models'].items():
            if mid not in v2b:
                rows.add(mid)
                if not local.get(mid): dl.add(mid)
            if m['section'] in ('Rigid', 'Articulation', 'Dynamic'): dyn.add(mid)
    cum_rows |= rows; cum_dl |= dl
    print(f"{tier}: {len(ts)} configs; new asset rows {len(rows)} (cumulative {len(cum_rows)}); USD downloads {len(dl)} (cumulative {len(cum_dl)}); dynamic models {len(dyn)}")
    print('   ', ', '.join(ts))
# T1 class list
cls = collections.Counter()
for t, x in pc.items():
    if x['tier'] == 'T1':
        for c in x['classes']: cls[c] += 1
print('T1 classes:', dict(cls))
