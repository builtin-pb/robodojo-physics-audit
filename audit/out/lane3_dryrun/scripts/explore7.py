import json, sys, collections
R = sys.argv[1]
g = json.load(open(R + '/audit/gates.json'))
print(type(g), list(g.keys())[:20] if isinstance(g, dict) else len(g))
for k, v in g.items():
    if k != 'configs' and k != 'tasks':
        print(k, json.dumps(v)[:1200])
cf = g.get('configs') or g.get('tasks')
print(type(cf), len(cf))
items = cf.items() if isinstance(cf, dict) else [(x.get('task_name') or x.get('config'), x) for x in cf]
for i, (name, v) in enumerate(items):
    if i < 2:
        print(name, json.dumps(v)[:3000])
