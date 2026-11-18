#!/usr/bin/env python3
"""Mirror RoboDojo's evaluation layout files (scored and spare, all seeds) at the pinned HF revision, for params_classes_build.py.

    python3 -I audit/tools/fetch_eval_layouts.py <dst dir> [--population git:80f3d16:audit/out/31/population.json] [--dry-run]

Paths come from population.json (seed_to_layout_template, scored and spare layout ids per seed): 8,070 small JSON files, about 100 MB.
Standard library only; downloads data files, runs no RoboDojo code. Existing files are kept.
"""
import argparse, concurrent.futures as cf, json, os, subprocess, time, urllib.error, urllib.parse, urllib.request

ap = argparse.ArgumentParser()
ap.add_argument('dst')
ap.add_argument('--population', default='git:80f3d16:audit/out/31/population.json')
ap.add_argument('--dry-run', action='store_true', help='print the path count and the first paths, download nothing')
args = ap.parse_args()
here = os.path.dirname(os.path.abspath(__file__))
if args.population.startswith('git:'):
    rev, path = args.population[4:].split(':', 1)
    root = subprocess.run(['git', '-C', here, 'rev-parse', '--show-toplevel'], capture_output=True, text=True, check=True).stdout.strip()
    pop = json.loads(subprocess.run(['git', '-C', root, 'show', f'{rev}:{path}'], capture_output=True, check=True).stdout)
else:
    pop = json.load(open(args.population))
rev = pop['pinned_dataset']['sha']
paths = []
for c in pop['configs']:
    lay = c['layouts']
    for seed in lay['official_leaderboard_seeds']:
        for lid in lay['scored_layout_ids'] + lay['spare_layout_ids']:
            paths.append(lay['seed_to_layout_template'].format(seed=seed, layout_id=lid))
assert len(paths) == pop['counts']['layout_files_all_seeds'], len(paths)
if args.dry_run:
    print(len(paths), 'paths at', rev); print('\n'.join(paths[:3])); raise SystemExit
RES = f'https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/resolve/{rev}/'


def fetch(p):
    dst = os.path.join(args.dst, p)
    if os.path.exists(dst):
        return 0, None
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    delay = 2.0
    for _ in range(8):
        try:
            with urllib.request.urlopen(RES + urllib.parse.quote(p), timeout=60) as r:
                data = r.read()
            json.loads(data)
            with open(dst + '.part', 'wb') as f:
                f.write(data)
            os.replace(dst + '.part', dst)
            return len(data), None
        except urllib.error.HTTPError as e:
            if e.code not in (429, 500, 502, 503, 504):
                return 0, f'{p}: HTTP {e.code}'
            ra = e.headers.get('Retry-After')
            time.sleep(float(ra) if ra and ra.isdigit() else delay); delay = min(delay * 2, 120)
        except Exception:
            time.sleep(delay); delay = min(delay * 2, 120)
    return 0, f'{p}: gave up'


total, errors = 0, []
with cf.ThreadPoolExecutor(6) as ex:
    for i, (n, err) in enumerate(ex.map(fetch, paths), 1):
        total += n
        if err:
            errors.append(err)
        if i % 1000 == 0:
            print(i, 'files,', round(total / 1e6, 1), 'MB new,', len(errors), 'errors', flush=True)
print('done:', len(paths), 'files,', round(total / 1e6, 1), 'MB new,', len(errors), 'errors')
for e in errors[:50]:
    print(e)
