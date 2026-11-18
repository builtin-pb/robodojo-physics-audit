import json, sys, collections
R, OUT = sys.argv[1], sys.argv[2]
cen = json.load(open(OUT + '/census_pairs.json'))
cl = {m['id']: m for m in json.load(open(R + '/audit/out/63/classes.json'))['models']}
for cls in ('rubber_elastomer', 'foam'):
    where = collections.defaultdict(list)
    for t, r in cen.items():
        if not r['in_scope']: continue
        for mid, m in r['models'].items():
            if m['material_class'] == cls:
                where[mid].append(t)
    print(cls, len(where), 'models')
    for mid, ts in sorted(where.items()):
        print(f"   {mid:30s} conf={cl[mid]['confidence']:6s} id='{cl[mid]['identity'][:50]}' configs={len(ts)} ({', '.join(sorted(ts)[:3])}{'...' if len(ts)>3 else ''})")
