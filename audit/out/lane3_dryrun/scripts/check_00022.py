import json, sys
OUT, REPO = sys.argv[1], sys.argv[2]
cen = json.load(open(OUT + '/census_pairs.json'))
for t, r in sorted(cen.items()):
    if 'Rigid/bottle/00022' in r['models']:
        print('00022 in', t, r['models']['Rigid/bottle/00022'])
    steel = [m for m, d in r['models'].items() if d['material_class'] == 'steel']
    if steel and r['in_scope']:
        print(f"{t:36s} steel models: {len(steel)}")
a = json.load(open(REPO + '/docs/params/assets.json'))['assets']['Rigid/bottle/00022']['material_override']
print({k: a[k] for k in ('static_friction', 'dynamic_friction', 'status')}, a['notes'][:600])
