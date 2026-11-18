import json, sys, collections, itertools
R = sys.argv[1]
cv = json.load(open(R + '/audit/out/63/class_values.json'))
cls = cv['classes']
for c, d in sorted(cls.items()):
    den = d.get('density_kg_m3') or {}
    rst = d.get('restitution') or {}
    print(f"{c:20s} dens={den.get('value')} {den.get('range')} {den.get('status')} | rest={rst.get('value')} {rst.get('range')} {rst.get('status')} | extra={[k for k in d if k not in ('density_kg_m3','restitution','notes')]}")
keys = {p['key'] for p in cv['pairs']}
names = sorted(cls)
missing = []
for a, b in itertools.combinations_with_replacement(names, 2):
    k = f"{a}__{b}"
    if k not in keys:
        missing.append(k)
print('n missing', len(missing))
print(missing)
sm = collections.defaultdict(list)
for p in cv['pairs']:
    sm[p['scenario_membership']].append(p['key'])
print('fixed', sm['fixed'])
print('default', sm['default'])
p = json.load(open(R + '/docs/params/pairs.json'))
for pk, pd in p['pairs'].items():
    if pd.get('origin') == 'analogue_167':
        print('analogue', pk, pd['scenario_membership'], pd['mu_static']['value'], pd['mu_static']['range'], pd['mu_static'].get('status'), pd['mu_kinetic']['value'], pd['mu_kinetic']['range'], pd['mu_kinetic'].get('status'))
print(json.dumps(cv['unary_values_default_scenario'])[:2000])
