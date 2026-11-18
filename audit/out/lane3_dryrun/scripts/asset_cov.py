import json, os, sys, collections
OUT, W = sys.argv[1], sys.argv[2]
cen = json.load(open(OUT + '/census_pairs.json'))
roots = [W + '/robodojo/Assets/Object/RoboDojo', W + '/tmp/rev73b/hf_obj_cache/Assets/Object/RoboDojo']
for r in roots:
    print(r, os.path.isdir(r), sorted(os.listdir(r)) if os.path.isdir(r) else None)
need = {}
for t, r in cen.items():
    if not r['in_scope']:
        continue
    for mid, d in r['models'].items():
        need.setdefault(mid, set()).add(t)
have = {}
for mid in need:
    found = None
    for r in roots:
        dd = os.path.join(r, mid)
        if os.path.isdir(dd):
            fs = [f for f in os.listdir(dd) if f.startswith('object.usd')]
            if fs:
                found = os.path.join(dd, fs[0]); break
    have[mid] = found
by_sec = collections.Counter(); by_sec_have = collections.Counter()
for mid, f in have.items():
    s = mid.split('/')[0]
    by_sec[s] += 1
    if f: by_sec_have[s] += 1
print('in-scope distinct models', len(need), 'local USD', sum(1 for f in have.values() if f))
for s in sorted(by_sec): print(' ', s, by_sec_have[s], '/', by_sec[s])
json.dump({k: v for k, v in have.items()}, open(OUT + '/asset_local.json', 'w'), indent=0)
