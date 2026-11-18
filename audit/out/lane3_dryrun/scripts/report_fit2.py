import json, sys
OUT = sys.argv[1]
P = json.load(open(f'{OUT}/fit_population.json'))
S = json.load(open(f'{OUT}/fit_standard_only.json'))
print('population configs', len(P['configs']), 'standard-only configs', len(S['configs']))
print('standard-only counted', S['counted_primary'], S['counted_sensitivity'], 'union', S['union_pair_count'])
print('standard-only classes', S['carried_primary_classes'])
dec = ['pad_hard__pet_plastic', 'pad_hard__table_laminate', 'abs_toy_plastic__pad_hard', 'hardwood__table_laminate',
       'abs_toy_plastic__table_laminate', 'hardwood__polyolefin_plastic', 'hardwood__pad_hard', 'pad_hard__polyolefin_plastic',
       'polyolefin_plastic__table_laminate', 'pet_plastic__table_laminate', 'Rigid/bottle/00022__pad_hard', 'aluminium__pet_plastic']
print(f"{'pair':36s} {'nominal':12s} {'v2b':14s} {'pop joint':14s} {'d_pop':14s} {'std-only':14s} {'d_std':14s}")
for pk in dec:
    d = P['done_pairs'][pk]; s = S['done_pairs'][pk]
    print(f"{pk:36s} {d['nominal'][0]:.3f}/{d['nominal'][1]:.3f}  {d['v2b'][0]:.4f}/{d['v2b'][1]:.4f} {d['joint'][0]:.4f}/{d['joint'][1]:.4f} "
          f"{d['joint'][0]-d['v2b'][0]:+.4f}/{d['joint'][1]-d['v2b'][1]:+.4f} {s['joint'][0]:.4f}/{s['joint'][1]:.4f} {s['joint'][0]-s['v2b'][0]:+.4f}/{s['joint'][1]-s['v2b'][1]:+.4f}")
# max abs delta over all done pairs
for name, R in (('population', P), ('standard_only', S)):
    dp = R['done_pairs']
    vals = [(max(abs(d['joint'][0]-d['v2b'][0]), abs(d['joint'][1]-d['v2b'][1])), k) for k, d in dp.items()]
    vals.sort(reverse=True)
    n_ge_01 = sum(1 for v, k in vals if v >= 0.01); n_ge_02 = sum(1 for v, k in vals if v >= 0.02); n_ge_05 = sum(1 for v, k in vals if v >= 0.05)
    print(name, 'done pairs', len(vals), '>=0.01:', n_ge_01, '>=0.02:', n_ge_02, '>=0.05:', n_ge_05, 'max', vals[0])
# pulls on pad_hard and table_laminate (joint population): implied unary = 2*nom - u_partner
for cls in ('pad_hard', 'table_laminate', 'pet_plastic', 'aluminium'):
    us = P['u_static']; uk = P['u_kinetic']
    rows = []
    for pk, r in P['joint_rows'].items():
        if pk.startswith('Rigid/'):
            continue
        a, b = pk.split('__')
        if cls not in (a, b) or not r['counted'] or r['membership'] == 'sensitivity':
            continue
        other = b if a == cls else a
        if other == cls:
            imp_s = r['nominal'][0]; imp_k = r['nominal'][1]
        else:
            imp_s = 2 * r['nominal'][0] - us[other]; imp_k = 2 * r['nominal'][1] - uk[other]
        rows.append((imp_s, imp_k, other, r['nominal'], r['n_configs']))
    rows.sort()
    print(f"== {cls}: joint unary {us[cls]:.4f}/{uk[cls]:.4f}; v2b-era partners vs new partners (implied static/kinetic = 2*nom - u_partner)")
    for imp_s, imp_k, other, nom, nc in rows:
        tag = 'v2b' if other in ('abs_device_housing','abs_toy_plastic','aluminium','ground','hardwood','pad_hard','pet_plastic','polyolefin_plastic','table_laminate','steel') else 'NEW'
        print(f"   {other:20s} {tag:4s} nom {nom[0]:.3f}/{nom[1]:.3f} implied {imp_s:.3f}/{imp_k:.3f} configs={nc}")
# top known costs in joint population
kc = []
for pk, r in P['joint_rows'].items():
    if r['counted'] and r['membership'] != 'sensitivity':
        ds = r['realised'][0]-r['nominal'][0]; dk = r['realised'][1]-r['nominal'][1]
        kc.append((max(abs(ds), abs(dk)), pk, ds, dk, r['n_configs']))
kc.sort(reverse=True)
print('top 15 joint deviations from nominal (population objective):')
for v, pk, ds, dk, nc in kc[:15]:
    print(f"   {pk:40s} {ds:+.3f}/{dk:+.3f} configs={nc}")
for tag in ('stagewise_A_steel_free', 'stagewise_B_all12_frozen'):
    sw = P[tag]
    print(tag, sw['feasible'], sw.get('sse_population_objective'), 'active', sw.get('active'), 'steel', sw.get('u_static', {}).get('steel'), sw.get('u_kinetic', {}).get('steel'))
print('joint', P['joint_sse_population_objective'])
