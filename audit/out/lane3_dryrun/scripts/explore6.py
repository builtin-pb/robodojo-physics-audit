import json, sys, collections
R = sys.argv[1]
for t in ('cover_blocks', 'swap_T'):
    s = json.load(open(R + f'/audit/out/63/{t}/{t}.json'))
    cs = s['count_summary']
    print('=====', t, {k: cs[k] for k in ('sourced','estimated','unsourced','total_parameter_fields')})
    for st, keys in cs['keys'].items():
        grp = collections.Counter()
        for k in keys:
            parts = k.split('.')
            if parts[0] == 'pairs':
                grp[('pairs', parts[-1])] += 1
            elif parts[0] == 'classes':
                grp[('classes', parts[-1])] += 1
            elif parts[0] == 'assets':
                grp[('assets', parts[-1])] += 1
            else:
                grp[(parts[0], parts[-1])] += 1
        print(' ', st, dict(grp))
    # list pair keys
    pk = sorted({k.split('.')[1] for st, keys in cs['keys'].items() for k in keys if k.startswith('pairs.')})
    print('  pairs counted', len(pk), pk)
    ck = sorted({k.split('.')[1] for st, keys in cs['keys'].items() for k in keys if k.startswith('classes.')})
    print('  classes counted', ck)
    ak = sorted({k.split('.')[1] for st, keys in cs['keys'].items() for k in keys if k.startswith('assets.')})
    print('  assets counted', ak)
    other = sorted({k for st, keys in cs['keys'].items() for k in keys if not k.split('.')[0] in ('pairs','classes','assets')})
    print('  other', other)
