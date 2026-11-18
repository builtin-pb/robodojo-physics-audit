import json, sys
OUT = sys.argv[1]; MODE = sys.argv[2] if len(sys.argv) > 2 else 'population'
r = json.load(open(f'{OUT}/fit_{MODE}.json'))
print('configs', len(r['configs']))
print('unbanded formable pairs', json.dumps(r['unbanded_formable_pairs'], indent=0)[:3000])
print('unclassed (franka) pairs', r['unclassed_pairs'])
print('joint SSE pop objective', r['joint_sse_population_objective'])
print('unaries joint vs v2b:')
import fractions
for c in sorted(r['u_static']):
    print(f"  {c:20s} {r['u_static'][c]:.6f} {r['u_kinetic'][c]:.6f}")
dec = ['pad_hard__pet_plastic', 'pad_hard__table_laminate', 'abs_toy_plastic__pad_hard', 'hardwood__table_laminate',
       'abs_toy_plastic__table_laminate', 'hardwood__polyolefin_plastic']
dp = r['done_pairs']
def row(pk):
    d = dp[pk]
    ds = d['joint'][0] - d['v2b'][0]; dk = d['joint'][1] - d['v2b'][1]
    return f"{pk:42s} nom {d['nominal'][0]:.3f}/{d['nominal'][1]:.3f} v2b {d['v2b'][0]:.4f}/{d['v2b'][1]:.4f} joint {d['joint'][0]:.4f}/{d['joint'][1]:.4f} delta {ds:+.4f}/{dk:+.4f} cnt={d['counted_v2b']} {d['membership']}"
print('DECISIVE')
for pk in dec: print(row(pk))
print('ALL DONE PAIRS (sorted by |delta|)')
keys = sorted(dp, key=lambda k: -max(abs(dp[k]['joint'][0]-dp[k]['v2b'][0]), abs(dp[k]['joint'][1]-dp[k]['v2b'][1])))
for pk in keys: print(row(pk))
for tag in ('stagewise_A_steel_free', 'stagewise_B_all12_frozen'):
    sw = r[tag]
    print('=====', tag, 'feasible', sw['feasible'])
    if sw['feasible']:
        print(' new', sw['new_classes'])
        print(' active', sw['active'])
        print(' sse', sw['sse_population_objective'], 'done max delta', sw['done_pairs_max_abs_delta'])
        for c in sw['new_classes']:
            print(f"   {c:20s} sw {sw['u_static'][c]:.6f}/{sw['u_kinetic'][c]:.6f}  joint {r['u_static'][c]:.6f}/{r['u_kinetic'][c]:.6f}")
    else:
        print(json.dumps(sw)[:2000])
