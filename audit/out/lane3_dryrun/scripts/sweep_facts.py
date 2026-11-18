import json, sys, collections
R, OUT = sys.argv[1], sys.argv[2]
s = json.load(open(R + '/audit/out/population_sweep/summary.json'))
facts = {}
for t, c in sorted(s['configs'].items()):
    sdf, decomp, nocol, mats, hulls = [], [], [], [], collections.Counter()
    for lab, o in (c.get('objects') or {}).items():
        e = o.get('engine') or {}
        apps = e.get('approximations') or []
        for a in apps: hulls[a] += 1
        if any('sdf' in str(a).lower() for a in apps):
            sdf.append(o.get('model'))
        if any('convexdecomposition' in str(a).lower() for a in apps):
            decomp.append(o.get('model'))
        if (e.get('colliders_enabled') == 0) or (e.get('colliders') == 0):
            nocol.append((o.get('model'), o.get('type'), e.get('colliders'), e.get('colliders_enabled')))
        um = e.get('unique_shape_materials') or []
        if um and any(tuple(m) != (0.5, 0.5, 0.0) for m in [tuple(x) if isinstance(x, list) else (x,) for x in um]):
            mats.append((o.get('model'), um))
    facts[t] = {'status': c.get('status'), 'error': c.get('error'), 'sdf_models_layout0': sorted(set(sdf)),
                'convex_decomposition_models_layout0': sorted(set(m for m in decomp if m)),
                'no_collider_layout0': nocol, 'non_default_materials_layout0': mats, 'approx_counts': dict(hulls),
                'readback_failures': c.get('readback_failures')}
json.dump(facts, open(OUT + '/sweep_facts.json', 'w'), indent=1, sort_keys=True)
for t, f in facts.items():
    print(f"{t:36s} {f['status']:24s} sdf={f['sdf_models_layout0']} nocol={f['no_collider_layout0']} decomp={len(f['convex_decomposition_models_layout0'])} mats={[m for m, _ in f['non_default_materials_layout0']]}")
an = [a for a in s['anomalies'] if a['code'] in ('NO_COLLIDER', 'MATERIAL_NOT_DEFAULT', 'JOINT_LIMIT_ABSURD')]
for a in an: print(a)
