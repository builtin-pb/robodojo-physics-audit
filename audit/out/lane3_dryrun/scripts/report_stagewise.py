import json, sys
OUT = sys.argv[1]
P = json.load(open(f'{OUT}/fit_population.json'))
J = P['joint_rows']; S = P['stagewise_B_all12_frozen']['rows']
def inb(r):
    return (r['band_s'][0] - 1e-9 <= r['realised'][0] <= r['band_s'][1] + 1e-9) and (r['band_k'][0] - 1e-9 <= r['realised'][1] <= r['band_k'][1] + 1e-9)
print('joint rows in band:', sum(inb(r) for r in J.values()), '/', len(J), ' stagewise rows in band:', sum(inb(r) for r in S.values()), '/', len(S))
def dev(r): return max(abs(r['realised'][0]-r['nominal'][0]), abs(r['realised'][1]-r['nominal'][1]))
cnt = lambda r: r['counted'] and r['membership'] != 'sensitivity'
newkc = sorted([k for k in S if cnt(S[k]) and dev(S[k]) >= 0.05 - 1e-12 and not dev(J[k]) >= 0.05 - 1e-12])
print('pairs that become known costs (|dev|>=0.05) only under stage-wise:', len(newkc))
for k in newkc: print(f"   {k:40s} nom {S[k]['nominal'][0]:.3f}/{S[k]['nominal'][1]:.3f} joint {J[k]['realised'][0]:.4f}/{J[k]['realised'][1]:.4f} stagewise {S[k]['realised'][0]:.4f}/{S[k]['realised'][1]:.4f} configs={S[k]['n_configs']}")
gone = sorted([k for k in S if cnt(S[k]) and dev(J[k]) >= 0.05 - 1e-12 and not dev(S[k]) >= 0.05 - 1e-12])
print('known costs only under joint:', len(gone), gone)
# biggest stagewise-vs-joint differences on counted primary pairs (new pairs)
diffs = sorted(((max(abs(S[k]['realised'][0]-J[k]['realised'][0]), abs(S[k]['realised'][1]-J[k]['realised'][1])), k) for k in S if cnt(S[k])), reverse=True)
print('top 12 stagewise minus joint (counted primary):')
for d, k in diffs[:12]:
    print(f"   {k:40s} nom {S[k]['nominal'][0]:.3f}/{S[k]['nominal'][1]:.3f} joint {J[k]['realised'][0]:.4f}/{J[k]['realised'][1]:.4f} stagewise {S[k]['realised'][0]:.4f}/{S[k]['realised'][1]:.4f} configs={S[k]['n_configs']}")
# SSE split: pairs among v2b-12 classes vs pairs involving a new class
V = set(P['stagewise_B_all12_frozen']['u_static']) - set(P['stagewise_B_all12_frozen']['new_classes'])
def split(R):
    a = b = 0.0; na = nb = 0
    for k, r in R.items():
        if not cnt(r): continue
        cls = k.split('__') if not k.startswith('Rigid/') else ['steel', k.split('__')[1]]
        ds = r['realised'][0]-r['nominal'][0]; dk = r['realised'][1]-r['nominal'][1]
        if all(c in V for c in cls): a += ds*ds+dk*dk; na += 1
        else: b += ds*ds+dk*dk; nb += 1
    return round(a, 4), na, round(b, 4), nb
print('SSE split (old-old pairs, n, pairs with a new class, n): joint', split(J), 'stagewise', split(S))
