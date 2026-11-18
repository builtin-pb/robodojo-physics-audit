import json, sys, collections
R = sys.argv[1]
cv = json.load(open(R + '/audit/out/63/class_values.json'))
txt_fields = ('basis', 'derivation', 'reasoning', 'reason_v2', 'why', 'notes', 'conditions', 'v3_change')
def has_basis(e):
    if not isinstance(e, dict): return False
    for f in txt_fields:
        if e.get(f): return True
    for s in e.get('sources') or []:
        if s.get('conditions') or s.get('quote'): return True
    return False
cnt = collections.Counter(); ex = {}
for p in cv['pairs']:
    for fld in ('mu_static', 'mu_kinetic'):
        e = p[fld]; s = e.get('range_status') or e.get('status')
        hb = has_basis(e)
        cnt[(s, hb)] += 1
        if s == 'unsourced' and not hb and len(ex) < 3: ex[p['key']] = e
        if s == 'unsourced' and hb and 'u_' + p['key'] not in ex and len([k for k in ex if k.startswith('u_')]) < 2:
            ex['u_' + p['key']] = {k: e[k] for k in e if k in txt_fields or k == 'sources'}
print('pair friction leaves (status, has basis text):', dict(cnt))
for k, v in ex.items(): print(k, json.dumps(v)[:700])
cc = collections.Counter()
for c, d in cv['classes'].items():
    for fld in ('density_kg_m3', 'restitution'):
        e = d.get(fld)
        if e is None: cc[(fld, 'missing')] += 1; continue
        cc[(fld, e.get('status'), has_basis(e))] += 1
print('class leaves:', dict(cc))
r = cv['classes']['abs_toy_plastic']['restitution']; print('abs_toy restitution', json.dumps(r)[:400])
print('class notes abs_toy', cv['classes']['abs_toy_plastic'].get('notes'))
OUT = sys.argv[2]
fp = json.load(open(OUT + '/fit_population.json'))
union_keys = set(k for k in fp['joint_rows'])
nob = []
for p in cv['pairs']:
    for fld in ('mu_static', 'mu_kinetic'):
        e = p[fld]; s = e.get('range_status') or e.get('status')
        if s == 'unsourced' and not has_basis(e):
            nob.append((p['key'], fld, p['key'] in union_keys))
pk_nob = sorted({k for k, f, u in nob})
print('unsourced pair leaves without basis:', len(nob), 'pairs', len(pk_nob), 'of which formable in scope:', len({k for k, f, u in nob if u}))
print(pk_nob)
rn = [c for c, d in cv['classes'].items() if d.get('restitution') and d['restitution'].get('status') == 'unsourced' and not has_basis(d['restitution'])]
print('class restitution unsourced without basis text:', rn)
