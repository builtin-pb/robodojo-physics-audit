import json, sys
R = sys.argv[1]
pop = json.load(open(R + '/audit/out/31/population.json'))
print('top keys', list(pop.keys()))
cfgs = pop['configs']
print('n configs', len(cfgs))
c0 = cfgs[0]
print('config keys', list(c0.keys()))
def show(d, depth=0, maxd=3):
    for k, v in d.items():
        if isinstance(v, dict):
            print('  '*depth + f'{k}: dict({len(v)})')
            if depth < maxd: show(v, depth+1, maxd)
        elif isinstance(v, list):
            print('  '*depth + f'{k}: list({len(v)}) ' + (json.dumps(v[0])[:200] if v else ''))
        else:
            print('  '*depth + f'{k}: {json.dumps(v)[:200]}')
show(c0)
