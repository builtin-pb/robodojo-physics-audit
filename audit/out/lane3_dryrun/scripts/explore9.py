import json, sys, collections
R = sys.argv[1]
s = json.load(open(R + '/audit/out/population_sweep/summary.json'))
print(type(s), list(s.keys())[:30] if isinstance(s, dict) else len(s))
def walk(d, path='', depth=0, maxd=4):
    if depth > maxd: return
    if isinstance(d, dict):
        for k, v in list(d.items())[:12]:
            t = type(v).__name__
            ln = len(v) if hasattr(v, '__len__') and not isinstance(v, str) else ''
            print('  '*depth + f'{k}: {t}{ln} ' + (json.dumps(v)[:150] if not isinstance(v,(dict,list)) else ''))
            walk(v, path+'/'+k, depth+1, maxd)
    elif isinstance(d, list) and d:
        walk(d[0], path+'[0]', depth+1, maxd)
walk(s, maxd=5)
