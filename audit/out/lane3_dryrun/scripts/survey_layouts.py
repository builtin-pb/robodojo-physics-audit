import json, sys, collections, re
W = sys.argv[1]; R = sys.argv[2]
man = json.load(open(W + '/tmp/census_layouts/manifest.json'))
cl = json.load(open(R + '/audit/out/63/classes.json'))
ids = {m['id'] for m in cl['models']}
types = collections.Counter()
scene_keys = collections.Counter()
missing_ids = collections.Counter()
sec_set = collections.Counter()
table_phys = collections.Counter()
n = 0
for rel, rec in man.items():
    d = json.load(open(rec['path']))
    n += 1
    for sec, v in d.items():
        sec_set[sec] += 1
        if isinstance(v, dict) and sec in ('Rigid', 'Geometry', 'Articulation', 'Garment', 'Fluid', 'Dynamic'):
            for cat, lst in v.items():
                for o in lst:
                    clut = o.get('type') == 'cluttered'
                    pt = (o.get('physics') or {}).get('type')
                    types[(sec, clut, pt)] += 1
                    prefix = 'Clutter' if clut else sec
                    mid = f"{prefix}/{cat}/{int(o['category_idx']):05d}"
                    if mid not in ids:
                        missing_ids[mid] += 1
        else:
            if sec in ('Table', 'Ground', 'Room'):
                scene_keys[(sec, tuple(sorted(v.keys())) if isinstance(v, dict) else type(v).__name__)] += 1
                if sec == 'Table' and isinstance(v, dict):
                    table_phys[json.dumps({k: v[k] for k in v if k not in ('default_pos','default_ori','material','scale','size')})[:200]] += 1
print('files', n)
print('sections', sec_set)
for k, v in sorted(types.items(), key=lambda kv: str(kv[0])): print(k, v)
print('missing ids', len(missing_ids), list(missing_ids.items())[:20])
for k, v in scene_keys.items(): print(k, v)
for k, v in table_phys.most_common(10): print(v, k)
