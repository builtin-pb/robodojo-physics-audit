import json, sys, collections
R = sys.argv[1]
g = json.load(open(R + '/audit/gates.json'))
for name, v in sorted(g['tasks'].items()):
    print(f"{name:36s} gate={v['gate']} deliv={v.get('delivered')} | {v.get('notes','')[:260]}")
