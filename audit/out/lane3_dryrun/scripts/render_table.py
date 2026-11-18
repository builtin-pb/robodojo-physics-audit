import json, sys
OUT = sys.argv[1]
pc = json.load(open(OUT + '/per_config.json'))
ab = {'abs_device_housing': 'housing', 'abs_toy_plastic': 'ABS', 'aluminium': 'Al', 'engineered_wood': 'engwood',
      'fabric_textile': 'fabric', 'foam': 'foam', 'food_organic': 'food', 'glass': 'glass', 'glazed_ceramic': 'ceramic',
      'hardwood': 'hardwood', 'mixed_unclear': 'mixed', 'paper_cardboard': 'paper', 'pet_plastic': 'PET',
      'polyolefin_plastic': 'PP', 'rubber_elastomer': 'rubber', 'softwood': 'softwood', 'steel': 'steel', 'liquid': 'liquid'}
def codes(x, t):
    c = []
    if not x['in_scope']: c.append('O')
    if x['franka_arms']: c.append('F')
    if x['articulations']: c.append(f"A{x['articulations']}")
    if x['conveyor']: c.append('C')
    if t == 'fasten_screws': c.append('S')
    if t == 'plug_in_charger': c.append('B')
    if x['mixed_unclear_models']: c.append(f"M{len(x['mixed_unclear_models'])}")
    if x['hollow_dynamic_models'] and t not in ('put_bottles_into_dustbin', 'cover_blocks', 'swap_T') and x['in_scope']: c.append(f"H{len(x['hollow_dynamic_models'])}")
    return ' '.join(c) or '-'
order = {'T0': 0, 'T1': 1, 'T2': 2, 'T3': 3, 'out': 4}
print('| config | tier | models (dyn) | object classes | leaves | src/est/scn/uns/miss % | blockers |')
print('|---|---|---|---|---|---|---|')
for t, x in sorted(pc.items(), key=lambda kv: (order[kv[1]['tier']], kv[0])):
    cls = [c for c in x['classes']]
    cs = ', '.join(ab.get(c, c) for c in cls) if len(cls) <= 5 else f"{len(cls)} classes"
    s = x['shares']
    sh = '/'.join(f"{s.get(k, 0):.0f}" for k in ('sourced', 'estimated', 'scenario', 'unsourced', 'missing'))
    print(f"| {t} | {x['tier']} | {x['n_models']} ({x['n_dynamic_models']}) | {cs} | {x['leaves_total']} | {sh} | {codes(x, t)} |")
