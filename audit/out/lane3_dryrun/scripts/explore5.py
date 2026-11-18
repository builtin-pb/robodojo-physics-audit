import json, sys, collections
R = sys.argv[1]
c = json.load(open(R + '/docs/params/classes.json'))
print('status_vocabulary', json.dumps(c['status_vocabulary'])[:1500])
print('scenarios', json.dumps(c['scenarios'])[:1500])
p = json.load(open(R + '/docs/params/pairs.json'))
st = collections.Counter()
for pk, pd in p['pairs'].items():
    for f in ('mu_static', 'mu_kinetic'):
        st[(f, pd[f].get('status'), pd[f].get('value_status'), pd[f].get('range_status'), pd.get('scenario_membership'))] += 1
for k, v in sorted(st.items(), key=lambda kv: str(kv[0])): print(k, v)
for t in ('cover_blocks', 'swap_T'):
    s = json.load(open(R + f'/audit/out/63/{t}/{t}.json'))
    print(t, 'keys', list(s.keys()))
    for k in s:
        if k not in ('assets', 'pairs'):
            print('  ', k, json.dumps(s[k])[:700])
