import json, sys
REPO, OUT = sys.argv[1], sys.argv[2]
cen = json.load(open(OUT + '/census_pairs.json'))
p = json.load(open(REPO + '/docs/params/pairs.json'))['realised_vs_nominal']
for task in ('put_bottles_into_dustbin', 'cover_blocks', 'swap_T'):
    mine = {}
    for pk, r in cen[task]['pairs'].items():
        mine[pk] = r['load_bearing']
    v2 = {}
    for pk, r in p['class_pairs'].items():
        if task in r['formable_in_tasks']:
            v2[pk] = None
    for pk, r in p['asset_override_pairs'].items():
        if task in r['formable_in_tasks']:
            v2[pk] = None
    # load-bearing in v2b per task: counted rows' formability string names tasks
    only_mine = sorted(set(mine) - set(v2))
    only_v2 = sorted(set(v2) - set(mine))
    print(task, 'mine', len(mine), 'v2b', len(v2), 'only_mine', only_mine, 'only_v2b', only_v2)
    print('  band-only (finger-on-finger) mine:', sorted(k for k, lb in mine.items() if not lb))
