import json, sys, collections
R = sys.argv[1]
cv = json.load(open(R + '/audit/out/63/class_values.json'))
print('class_values keys', list(cv.keys()))
for k, v in cv.items():
    if k not in ('classes', 'pairs'):
        print(' ', k, json.dumps(v)[:400])
cls = cv['classes']
print('n classes', len(cls), sorted(cls))
c0 = cls['abs_toy_plastic']
print('class keys', list(c0.keys()))
for k, v in c0.items():
    print('   ', k, json.dumps(v)[:500])
prs = cv['pairs']
print('n pairs', len(prs))
print('pair keys', list(prs[0].keys()))
print(json.dumps(prs[0])[:2500])
kinds = collections.Counter(p['kind'] for p in prs)
print(kinds)
orig = collections.Counter(p['origin'] for p in prs)
print(orig)
sm = collections.Counter(p['scenario_membership'] for p in prs)
print(sm)
st = collections.Counter((p['mu_static'].get('status'), p['mu_kinetic'].get('status')) for p in prs)
print(st)
