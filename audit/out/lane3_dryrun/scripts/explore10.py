import json, sys, collections
R = sys.argv[1]
s = json.load(open(R + '/audit/out/population_sweep/summary.json'))
c = s['configs']['fasten_screws']
print(list(c.keys()))
for k, v in c.items():
    if k not in ('do_nothing',):
        print(k, json.dumps(v)[:2500])
print(json.dumps(s['counts']['configs_by_code'])[:3000])
