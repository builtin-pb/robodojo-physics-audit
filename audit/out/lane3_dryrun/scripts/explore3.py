import json, sys, collections
R = sys.argv[1]
cl = json.load(open(R + '/audit/out/63/classes.json'))
print('classes.json keys', list(cl.keys()))
for k, v in cl.items():
    if k != 'models':
        print(' ', k, json.dumps(v)[:600])
ms = cl['models']
print('n models', len(ms))
print('model keys', list(ms[0].keys()))
print(json.dumps(ms[0])[:1500])
cnt = collections.Counter(m['material_class'] for m in ms)
print(cnt)
conf = collections.Counter(m['confidence'] for m in ms)
print(conf)
typ = collections.Counter(m['type'] for m in ms)
print(typ)
flags = collections.Counter(f for m in ms for f in m.get('flags', []))
print(flags)
