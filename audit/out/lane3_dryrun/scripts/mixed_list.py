import json, sys, collections
R, OUT = sys.argv[1], sys.argv[2]
cl = {m['id']: m for m in json.load(open(R + '/audit/out/63/classes.json'))['models']}
pc = json.load(open(OUT + '/per_config.json'))
where = collections.defaultdict(list)
for t, x in pc.items():
    for m in x['mixed_unclear_models']:
        where[m].append(t)
for m in sorted(where):
    d = cl[m]
    print(f"{m:34s} conf={d['confidence']:6s} mass={d['declared_mass_kg']} bbox={[round(v,3) for v in (d['bbox_extents_m'] or [])]} id='{(d['identity'] or '')[:60]}' cap='{(d.get('caption_name') or '')[:30]}' alts={d['alternative_material_classes']} configs={len(where[m])} inscope={sum(1 for t in where[m] if pc[t]['in_scope'])}")
# hollow flagged dynamic models in T1 configs
hol = collections.Counter()
for t, x in pc.items():
    if x['tier'] != 'T1': continue
    cen = json.load(open(OUT + '/census_pairs.json'))[t]
    n = sum(1 for mid, m in cen['models'].items() if m['section'] in ('Rigid',) and 'hollow_contents_dependent' in cl[mid]['flags'] and m['class_source'] == 'class_table')
    hol[t] = n
print('hollow-flagged new dynamic models in T1 configs:', dict(hol))
