#!/usr/bin/env python3
"""Build the #63 material class table (v3) from committed inputs and the HF layout files.

Standard library only. Reads RoboDojo data files (layout and population JSON); never imports or runs RoboDojo code.

    python3 -I audit/tools/params_classes_build.py --layouts <dir> [--out audit/out/63]

Inputs (a path, or git:<rev>:<path> read with `git show` from this repository):
  --step2-classes  step-2 labels with HF captions (default git:6ac666c:audit/out/63/classes.json)
  --draft-values   step-3 values with the second-agent citation check (default git:0daca3c:audit/out/63/class_values_draft.json)
  --population     frozen population #31 (default git:80f3d16:audit/out/31/population.json)
  --bottle-slice   PR #84 bottle slice, used only for alignment_84.json (default git:753d7d7:audit/out/63/bottles/put_bottles_into_dustbin.json)
  --layouts        directory holding the population.json layout paths (Assets/Eval_Layout/RoboDojo/arx_x5/{0,1,2}/<task>_<id>.json) from HF
                   RoboDojo-Benchmark/RoboDojo at revision 1127bda94f172d757386945af3eb3a6754d1a4e6 (8,070 files)
Outputs in --out: classes.json, class_values.json, feasibility.json, layout_mass_checks.json, alignment_84.json,
classes.md, citation_check.md, build_summary.json.
"""
import argparse, collections, copy, decimal, fractions, hashlib, itertools, json, os, re, subprocess

ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
ap.add_argument('--step2-classes', default='git:6ac666c:audit/out/63/classes.json')
ap.add_argument('--draft-values', default='git:0daca3c:audit/out/63/class_values_draft.json')
ap.add_argument('--population', default='git:80f3d16:audit/out/31/population.json')
ap.add_argument('--bottle-slice', default='git:753d7d7:audit/out/63/bottles/put_bottles_into_dustbin.json')
ap.add_argument('--layouts', required=True)
ap.add_argument('--out', default=None, help='output directory (default: audit/out/63 of this repository)')
args = ap.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))
_root = []


def root():
    """This repository's top level; asked of git only when a git: input or the default output directory needs it."""
    if not _root:
        _root.append(subprocess.run(['git', '-C', HERE, 'rev-parse', '--show-toplevel'], capture_output=True, text=True, check=True).stdout.strip())
    return _root[0]


OUT = args.out or os.path.join(root(), 'audit/out/63')
HF_REV = '1127bda94f172d757386945af3eb3a6754d1a4e6'


def load(spec):
    if spec.startswith('git:'):
        rev, path = spec[4:].split(':', 1)
        return json.loads(subprocess.run(['git', '-C', root(), 'show', f'{rev}:{path}'], capture_output=True, check=True).stdout)
    return json.load(open(spec))


old = load(args.step2_classes)
ver = load(args.draft_values)
pop = load(args.population)
slice84 = load(args.bottle_slice) if args.bottle_slice else None
models = copy.deepcopy(old['models'])
by = {m['id']: m for m in models}
RANK = {'low': 0, 'medium': 1, 'high': 2}
log = []   # per-model change records


def setm(m, conf, alts, why, rule, version='v3'):
    log.append({'id': m['id'], 'class': m['material_class'], 'confidence_from': m['confidence'], 'confidence_to': conf,
                'alternatives': alts, 'reason': why, 'rule': rule, 'version': version})
    for a in alts:
        if a not in m['alternative_material_classes'] and a != m['material_class']:
            m['alternative_material_classes'].append(a)
    m['confidence'] = conf
    m.setdefault(f'{version}_changes', []).append({'rule': rule, 'reason': why})


def cap(m, level, alts, why, rule, version='v3'):
    """Lower confidence to `level` (never raise) and add alternatives; no-op if neither changes."""
    lower = RANK[m['confidence']] > RANK[level]
    new_alts = [a for a in alts if a != m['material_class'] and a not in m['alternative_material_classes']]
    if lower or new_alts:
        setm(m, level if lower else m['confidence'], alts, why, rule, version)


def reclass(m, cls, conf, alts, why, rule):
    log.append({'id': m['id'], 'class_from': m['material_class'], 'class': cls, 'confidence_from': m['confidence'],
                'confidence_to': conf, 'alternatives': alts, 'reason': why, 'rule': rule, 'version': 'v3'})
    m['material_class'] = cls; m['confidence'] = conf
    m['alternative_material_classes'] = [a for a in alts if a != cls]
    m.setdefault('v3_changes', []).append({'rule': rule, 'reason': why})


for m in models:
    m.setdefault('alternative_material_classes', [])

# ================================================================ 1. labels: v2 rules (unchanged), then v3 rules
POLY_TOK = re.compile(r'\b(PET|PETE|HDPE|LDPE|PE|PP|PS|polypropylene|polyethylene|polyolefin|terephthalate)\b', re.I)
PET_TOK = re.compile(r'\b(PET|PETE|terephthalate|polyester)\b', re.I)
POLYOL_TOK = re.compile(r'\b(HDPE|LDPE|PE|PP|polypropylene|polyethylene|polyolefin)\b', re.I)
PLASTIC_WORD = re.compile(r'plastic|polymer', re.I)

# v2 1a. named bottles (#84 review)
b = by
setm(b['Rigid/bottle/00055'], 'low', ['polyolefin_plastic'],
     'Caption says only "plastic / smooth plastic / molded polymer"; the word "plastic" is not evidence of PET and "water/sports/squeeze bottle" is as often PP/HDPE (#84 review, section 3).', 'named_bottle', 'v2')
b['Rigid/bottle/00055']['reason'] = 'drink-bottle shape; polymer not stated in caption (PET assumed from category prior only)'
setm(b['Rigid/bottle/00059'], 'low', ['polyolefin_plastic'],
     'Caption says only "plastic / smooth plastic / molded polymer"; "single-serve sports bottle" does not fix the polymer (#84 review, section 3).', 'named_bottle', 'v2')
b['Rigid/bottle/00059']['reason'] = 'drink-bottle shape; polymer not stated in caption (PET assumed from category prior only)'
setm(b['Rigid/bottle/00005'], 'low', ['polyolefin_plastic'],
     'Caption reads "finger cap / rubber / flexible polymer" and contradicts a 17 cm bottle; neither identity nor polymer is established.', 'named_bottle', 'v2')
b['Rigid/bottle/00005']['reason'] = 'caption (finger cap, rubber) contradicts bottle bbox; class is a placeholder'
m22 = b['Rigid/bottle/00022']
m22['reason'] = ('stainless/metal body per caption ("stainless steel / shiny metal body / plastic cap with rubber seal"); '
                 'declared 0.22 kg over a 24 cm bottle bbox fits a metal bottle, not thin PET. '
                 'IKEA identity and PP cap are caption-derived, not verified. Possibly coated (needs per-asset friction override).')
m22['v2_changes'] = [{'rule': 'named_bottle', 'reason': 'Reason rewritten: dropped stale "drink bottles are PET" (it contradicted the steel class); class steel and confidence medium unchanged.'}]
log.append({'id': m22['id'], 'class': 'steel', 'confidence_from': 'medium', 'confidence_to': 'medium', 'alternatives': [],
            'reason': m22['v2_changes'][0]['reason'], 'rule': 'named_bottle', 'version': 'v2'})
for i in ('Rigid/bottle/00050', 'Rigid/bottle/00060'):
    cap(b[i], 'low', ['polyolefin_plastic'], 'Already low (caption names a cap, not a bottle); added polyolefin alternative since "plastic" alone does not identify PET.', 'plastic_word_only', 'v2')

# v2 1b. "plastic" alone is not evidence of a specific polymer (pet_plastic, polyolefin_plastic)
ALT_BY_TOKEN = [(re.compile(r'steel|\bmetal\b', re.I), 'steel'), (re.compile(r'\bPS\b|ABS', re.I), 'abs_toy_plastic'),
                (re.compile(r'pulp|wood', re.I), 'softwood'), (re.compile(r'alumin', re.I), 'aluminium'),
                (re.compile(r'\bPET\b', re.I), 'pet_plastic')]
CONTAINER_CATS = {'bottle', 'shampoo', 'body_lotion', 'toothpaste', 'glue', 'yogurt'}
for m in models:
    if m['material_class'] not in ('pet_plastic', 'polyolefin_plastic'):
        continue
    cls = m['material_class']
    cm = (m['caption_material'] or ''); cn = (m['caption_name'] or ''); rs = m['reason']
    cap_text = cm + ' ' + cn
    tok = PET_TOK if cls == 'pet_plastic' else POLYOL_TOK
    alts = ['polyolefin_plastic'] if cls == 'pet_plastic' else []
    if cls == 'polyolefin_plastic':
        for rx, a in ALT_BY_TOKEN:   # alternatives only; adding an alternative cannot overstate evidence
            if rx.search(rs) or rx.search(cap_text):
                alts.append(a)
        if not alts:
            alts = ['abs_toy_plastic'] + (['pet_plastic'] if m['category'] in CONTAINER_CATS else [])
    if tok.search(cap_text) or m['id'] in ('Rigid/bottle/00055', 'Rigid/bottle/00059', 'Rigid/bottle/00005', 'Rigid/bottle/00022'):
        continue
    if m['caption_status'] in ('used', 'mismatch') and PLASTIC_WORD.search(cap_text) and not POLY_TOK.search(cap_text):
        if re.search(r'transparent|clear', cm, re.I) and cls == 'pet_plastic':
            cap(m, 'medium', alts, 'Caption says "transparent/clear plastic bottle"; clarity is a weak cue for PET but no polymer is named, so capped at medium (not high) with polyolefin alternative.', 'plastic_word_only_clear_cue', 'v2')
        else:
            cap(m, 'low', alts, 'Class rests on the caption word "plastic" (no polymer named); the word alone is not evidence of a specific polymer.', 'plastic_word_only', 'v2')
    elif m['caption_status'] == 'none' or not cap_text.strip():
        named = POLY_TOK.findall(rs)
        multi = len(set(x.upper() for x in named)) > 1 or re.search(r'\bor\b|/', rs)
        if not named or re.search(r'unknown|not determinable|\bor\b|plastic frame', rs, re.I) and not (named and not multi):
            cap(m, 'low', alts, f'No caption; class rests on the category prior "{rs}", which names no single polymer.', 'category_prior_generic', 'v2')
        elif multi:
            cap(m, 'medium', alts, f'No caption; the category admits several polymers ("{rs}"), so the specific class cannot be high.', 'category_prior_multi', 'v2')
    if m['caption_status'] == 'used' and not cm.strip() and not POLY_TOK.search(rs):
        cap(m, 'low', alts, f'Description gives identity but no material; reason "{rs}" names no polymer.', 'category_prior_generic', 'v2')
    elif m['caption_status'] == 'used' and not cm.strip() and POLY_TOK.search(rs) and RANK[m['confidence']] > 1 and re.search(r'/|\bor\b', rs):
        cap(m, 'medium', alts, f'Description gives identity but no material; the category admits several polymers ("{rs}").', 'category_prior_multi', 'v2')
    if m['id'] == 'Rigid/bottle/00061':
        cap(m, 'low', alts, 'Description "matte green water bottle" gives no material; matte opaque sport bottles are at least as often PP/HDPE; "drink bottles are PET" is a category prior only.', 'category_prior_generic', 'v2')
        m['reason'] = 'matte green water bottle; polymer unknown (PET assumed from category prior only)'
    if m['id'] == 'Clutter/bottle/00000':
        cap(m, 'low', alts, 'No caption and reason says "bottle type not determinable"; medium was inconsistent.', 'category_prior_generic', 'v2')

# ---- v3 1c. specific reclassifications (review items 5 and 7)
reclass(by['Rigid/tape_roll/00000'], 'polyolefin_plastic', 'low', ['paper_cardboard'],
        'Description "white packing tape": the grasped outer surface of a packing-tape roll is the tape film (BOPP, a polypropylene), not the card core; '
        'declared friction 0.25 fits a smooth polymer film. Card core kept as alternative (#73 review item 5).', 'outer_contact_material')
by['Rigid/tape_roll/00000']['reason'] = 'packing-tape roll: outer contact is the tape film (BOPP polypropylene, typical for packing tape); card core inside'
cup = by['Rigid/cup/00001']
reclass(cup, 'polyolefin_plastic', 'medium', ['glazed_ceramic'],
        'Used only in cover_blocks. Real ARX X5 frames of cover_blocks episode 0 (audit/out/63/real_frames/, #63 comment 2026-11-11T19:16:17Z) show moulded plastic cups '
        'with microwave/dishwasher icons and a recycling mark on the base, read as PP. The synthetic mesh caption says "glazed ceramic / smooth shiny surface"; '
        'the class follows the real task, the caption class is the alternative (#73 review item 5).', 'real_frames_override')
cup['reason'] = 'real cover_blocks cups are moulded PP (real-robot frames, #63); synthetic mesh caption says glazed ceramic'
cup['flags'] = sorted(set(cup['flags']) | {'real_vs_sim_material_conflict'})
cup['real_vs_sim'] = {'real': 'polyolefin_plastic (moulded PP cup, real frames cover_blocks_ep0_*.jpg)', 'sim_caption': 'glazed_ceramic ("glazed ceramic / smooth shiny surface / non-porous finish")',
                      'note': 'the resin-code digit is not legible in the committed 640x480 frames; PP rests on Bo\'s reading in #63 and the microwave-safe moulding'}
tp = by['Clutter/toothpaste/00001']
reclass(tp, 'aluminium', 'medium', ['polyolefin_plastic'],
        'Caption names the tube material: "Marvis toothpaste ... flexible aluminum tube / plastic cap / glossy printed film". Class now follows the caption '
        '(v2 kept polyolefin from the step-1 guess "laminate/PE tube", which is not caption evidence); laminate tubes kept as alternative.', 'caption_names_material')
tp['reason'] = 'caption: "flexible aluminum tube" (Marvis); printed/lacquered aluminium tube surface'
# PR #84 alignment of bottle labels (section 6.1 of the review; see alignment_84.json)
cap(by['Rigid/bottle/00005'], 'low', ['abs_toy_plastic', 'rubber_elastomer'],
    'Adopted PR #84\'s alternatives: unidentified opaque cylinder whose caption says "rubber / soft silicone / flexible polymer".', 'align_84')
for i in ('Rigid/bottle/00050', 'Rigid/bottle/00060'):
    cap(by[i], 'low', ['abs_toy_plastic'], 'Adopted PR #84\'s abs_toy_plastic alternative: untextured cylinder, caption names a plastic cap, no polymer.', 'align_84')
m25 = by['Rigid/bottle/00025']
setm(m25, 'high', [], 'Adopted PR #84\'s high: the caption names the product ("Dongfang Shuye / Oriental Tree Leaf drink", Nongfu Spring bottled tea), which is sold in '
     'PET bottles; the class rests on the identified product, not on the word "plastic". polyolefin alternative dropped.', 'align_84')
m25['alternative_material_classes'] = []
m25['reason'] = 'caption identifies Nongfu Spring "Dongfang Shuye" bottled tea, a PET-bottled product'
by['Rigid/bottle/00047']['reason'] = 'caption names an aluminium can ("soda can / aluminum can"); 5.8 x 14.6 cm fits a slim 330 ml can, although the category is bottle'
for i in ('Rigid/bottle/00050', 'Rigid/bottle/00060'):
    by[i]['reason'] = 'untextured cylinder in the bottle category; caption names a plastic cap (mismatch); PET is the bottle-category prior only'
by['Rigid/bottle/00041']['reason'] = 'caption "NFC juice bottle ... clear plastic bottle"; clarity favours PET, polymer not named'
by['Rigid/bottle/00055']['evidence'] = 'caption material "plastic / smooth plastic / molded polymer" names no polymer; consistent with pet_plastic or polyolefin_plastic'
by['Rigid/shovel/00000']['reason'] = 'caption: "lavender plastic shovel" (polymer not named); moulded garden-toy trowel'

# ---- v3 1d. class evidence from captions only (never from an earlier agent's reason)
PLASTICS = {'abs_toy_plastic', 'abs_device_housing', 'pet_plastic', 'polyolefin_plastic'}
ABS = {'abs_toy_plastic', 'abs_device_housing'}
WOODS = {'softwood', 'hardwood', 'engineered_wood'}
METALS = {'steel', 'aluminium'}
TOKENS = [  # (group, regex, classes, specific, material kind)
    ('pet', r'\b(PET|PETE)\b|terephthalate', {'pet_plastic'}, True, 'polymer'),
    ('polyolefin', r'\b(HDPE|LDPE|PE|PP|BOPP)\b|polypropylene|polyethylene|polyolefin', {'polyolefin_plastic'}, True, 'polymer'),
    ('family_polymer', r'\b(ABS|PS|PVC|PC|PMMA|POM|PLA)\b|polystyrene|polycarbonate|acrylic|nylon|vinyl|melamine|resin', ABS, True, 'polymer'),
    ('plastic', r'plastic|polymer', PLASTICS, False, 'polymer'),
    ('steel', r'stainless|steel|\biron\b|brass|chrome|nickel|copper|bronze|\btin\b|zinc|die-?cast', {'steel'}, True, 'metal'),
    ('aluminium', r'alumin', {'aluminium'}, True, 'metal'),
    ('metal', r'(?<!non-)\bmetal|\balloy', METALS, False, 'metal'),
    ('glass', r'\bglass\b', {'glass'}, True, 'glass'),
    ('ceramic', r'ceramic|porcelain|stoneware|earthenware|terracotta|\bglaze', {'glazed_ceramic'}, True, 'ceramic'),
    ('engineered_wood', r'\bMDF\b|plywood|particle ?board|chipboard', {'engineered_wood'}, True, 'wood'),
    ('wood', r'wood|bamboo|\boak\b|\bpine\b|beech|maple|walnut|birch|timber|rattan|wicker|\breed\b', WOODS, False, 'wood'),
    ('rubber', r'rubber|silicone|elastomer|\bTP[RUE]\b|latex', {'rubber_elastomer'}, True, 'rubber'),
    ('foam', r'foam|sponge', {'foam'}, True, 'foam'),
    ('fabric', r'fabric|cloth|cotton|polyester|denim|wool|felt|plush|textile|canvas|leather|knit|woven|fleece|velvet|linen|suede|\bmesh\b', {'fabric_textile'}, True, 'fabric'),
    ('paper', r'paper|cardboard|\bcard\b|carton|kraft|corrugated|cardstock', {'paper_cardboard'}, True, 'paper'),
]
TOKENS = [(g, re.compile(rx, re.I), cs, sp, kd) for g, rx, cs, sp, kd in TOKENS]
KIND_CLASSES = collections.defaultdict(set)
for g, rx, cs, sp, kd in TOKENS:
    KIND_CLASSES[kd] |= cs


def phrase_hits(ph):
    return [(g, cs, sp, kd) for g, rx, cs, sp, kd in TOKENS if rx.search(ph)]


def caption_evidence(m):
    """Label the caption's support for the assigned class. Captions only: caption_name/caption_material from HF description.json."""
    cls = m['material_class']
    if m['caption_status'] == 'none':
        return 'no_caption', []
    if m['caption_status'] == 'mismatch':
        return 'caption_mismatch', []
    phrases = [p.strip() for p in (m.get('caption_material') or '').split('/') if p.strip()] or [m.get('caption_name') or '']
    hits = [phrase_hits(p) for p in phrases]
    first = next((h for h in hits if h), None)
    other_kinds = sorted({kd for h in hits for (_, _, _, kd) in h} - ({kd for (_, _, _, kd) in first} if first else set()))
    if first is None:
        return 'caption_identity_only', other_kinds
    if cls in ('food_organic', 'liquid', 'mixed_unclear'):
        return 'caption_identity_only', other_kinds
    named = set().union(*[cs for (_, cs, _, _) in first])
    if cls not in named:
        return 'caption_material_conflict', other_kinds
    if any(sp and cls in cs for (_, cs, sp, _) in first):
        return 'caption_material_specific', other_kinds
    return 'caption_material_generic', other_kinds


# multi-material categories: real instances commonly differ in contact class. (cap level, alternatives). The assigned class
# is the most common one where the cap is "medium"; "low" where no class dominates.
MULTI = {
    'audio': ('medium', ['fabric_textile', 'aluminium']), 'baseball_bat': ('medium', ['aluminium']),
    'binoculars': ('medium', ['rubber_elastomer', 'aluminium']), 'book_shelf': ('medium', ['softwood', 'hardwood']),
    'bowl': ('medium', ['glass', 'polyolefin_plastic', 'steel']), 'box': ('medium', ['polyolefin_plastic', 'engineered_wood']),
    'camera': ('medium', ['aluminium', 'rubber_elastomer']), 'camera_stand': ('medium', ['steel', 'abs_device_housing']),
    'can': ('medium', ['steel']), 'card': ('medium', ['abs_toy_plastic']), 'car': ('medium', ['steel']),
    'chessman': ('low', ['abs_toy_plastic', 'softwood']), 'chocolate_bag': ('medium', ['polyolefin_plastic']),
    'clip': ('medium', ['abs_toy_plastic']), 'clock': ('medium', ['steel', 'softwood']), 'corkscrew': ('medium', ['abs_toy_plastic']),
    'dog_toy': ('low', ['fabric_textile', 'abs_toy_plastic']), 'drawer': ('medium', ['softwood', 'polyolefin_plastic']),
    'earbuds': ('medium', ['rubber_elastomer']), 'earth_globe': ('medium', ['paper_cardboard']), 'goblet': ('medium', ['steel', 'glazed_ceramic']),
    'hammer': ('medium', ['hardwood', 'abs_toy_plastic', 'rubber_elastomer']), 'headphone': ('medium', ['foam', 'rubber_elastomer']),
    'headphones': ('medium', ['foam', 'rubber_elastomer']), 'headset': ('medium', ['foam', 'rubber_elastomer']),
    'keyboard': ('medium', ['aluminium']), 'mahjong': ('medium', ['hardwood']), 'mallet': ('medium', ['rubber_elastomer']),
    'minecraft_blocks': ('medium', ['foam', 'paper_cardboard']), 'monitor': ('medium', ['aluminium']),
    'mousemat': ('low', ['fabric_textile', 'rubber_elastomer']), 'noodles': ('medium', ['foam']), 'number_card': ('low', ['abs_toy_plastic']),
    'oscilloscope': ('medium', ['steel']), 'pen': ('medium', ['aluminium']), 'peeler': ('medium', ['abs_toy_plastic']),
    'phone': ('medium', ['glass', 'aluminium']), 'photo': ('low', ['glass', 'engineered_wood', 'abs_toy_plastic']),
    'piggy_bank': ('medium', ['abs_toy_plastic']), 'piler': ('medium', ['rubber_elastomer', 'abs_toy_plastic']),
    'plier': ('medium', ['rubber_elastomer', 'abs_toy_plastic']), 'pliers': ('medium', ['rubber_elastomer', 'abs_toy_plastic']),
    'plate': ('medium', ['glass', 'abs_toy_plastic']), 'pokeball': ('medium', ['fabric_textile']), 'power_bank': ('medium', ['aluminium']),
    'puzzle_pieces': ('medium', ['softwood']), 'saw': ('medium', ['hardwood']), 'scale': ('medium', ['glass', 'steel']),
    'scissor': ('medium', ['abs_toy_plastic']), 'speaker': ('medium', ['fabric_textile', 'rubber_elastomer']),
    'tape_measure': ('medium', ['rubber_elastomer']), 'test_tube': ('medium', ['abs_toy_plastic', 'polyolefin_plastic']),
    'toolbox': ('medium', ['steel']), 'usb_multip': ('medium', ['aluminium']), 'whisk': ('medium', ['abs_toy_plastic', 'rubber_elastomer']),
    'bottle': ('medium', ['pet_plastic']), 'shoe': ('medium', ['rubber_elastomer']),
    'shovel': ('medium', ['hardwood', 'abs_toy_plastic', 'aluminium']), 'mouse': ('medium', ['rubber_elastomer', 'aluminium']),
}
GENERIC_ALT = {'plastic': ['abs_toy_plastic'], 'metal': ['steel', 'aluminium'], 'wood': ['softwood', 'hardwood']}


def caption_classes(m):
    """Classes the caption's first material phrase names (specific groups, or a default member for a generic family)."""
    phrases = [p.strip() for p in (m.get('caption_material') or '').split('/') if p.strip()] or [m.get('caption_name') or '']
    first = next((h for h in (phrase_hits(p) for p in phrases) if h), [])
    out = []
    for g, cs, sp, kd in first:
        for c_ in (sorted(cs) if sp else GENERIC_ALT.get(g, [])):
            if c_ not in out:
                out.append(c_)
    return out, next((p for p in phrases if phrase_hits(p)), '')
NO_MATERIAL_EVIDENCE = {'no_caption', 'caption_mismatch', 'caption_identity_only'}
for m in models:
    ev, other = caption_evidence(m)
    m['class_evidence'] = ev
    if other:
        m['caption_other_material_kinds'] = other
    else:
        m.pop('caption_other_material_kinds', None)
cup['class_evidence'] = 'real_robot_frames'
m25['class_evidence'] = 'caption_names_product'
for m in models:
    if m['material_class'] == 'liquid':
        m['class_evidence'] = 'physics_type_fluid'
    if m['material_class'] == 'mixed_unclear' or m['category'] not in MULTI or m['id'] == 'Rigid/bottle/00025':
        continue
    if m['category'] == 'bottle' and m['material_class'] != 'aluminium':
        continue   # plastic bottles are handled by the polymer rules above; only the can-shaped 00047 is a multi-material case
    lvl, alts = MULTI[m['category']]
    ev = m['class_evidence']
    own_kind = next((kd for g, rx, cs, sp, kd in TOKENS if m['material_class'] in cs), None)
    multi_caption = ev in ('caption_material_generic', 'caption_material_specific') and any(k != own_kind for k in m.get('caption_other_material_kinds', []))
    if ev in NO_MATERIAL_EVIDENCE:
        cap(m, lvl, alts, f'{ev.replace("_", " ")}: class rests on the category "{m["category"]}", whose real instances are commonly made of '
            f'{", ".join([m["material_class"]] + [a for a in alts if a != m["material_class"]])}; capped at {lvl} with those alternatives (#73 review item 5).', 'multi_material_category')
    elif ev == 'caption_material_generic' and m['material_class'] in METALS:
        cap(m, 'medium', alts, f'Caption names only "metal" ({m["caption_name"]}) on the multi-material category "{m["category"]}": steel versus aluminium alloy, and a non-metal handle, are all open; '
            'capped at medium with those alternatives (#88 item 3).', 'multi_material_generic_metal')
    elif multi_caption:
        extra = sorted(set().union(*[KIND_CLASSES[k] for k in m['caption_other_material_kinds']]) & set(alts)) or alts
        cap(m, 'medium', extra, f'Caption lists several contact materials ({m["caption_material"]}) on a multi-material category; capped at medium with alternatives.', 'multi_material_caption')
for m in models:   # the caption names another class's material: step 2 kept the class on bbox/mass/category, so medium at most, caption class as alternative
    if m['class_evidence'] == 'caption_material_conflict':
        alts, phrase = caption_classes(m)
        cap(m, 'medium', alts, f'Caption material "{phrase}" names {", ".join(alts)}, not {m["material_class"]}; the class rests on category, bbox and mass, '
            f'so it is at most medium with the caption class as alternative (#73 review item 5).', 'caption_conflict')

# ---- v3 1e. polymer subtype for polyolefin models: captions and category only
PP_CATS = {'dustbin', 'toolbox', 'basket', 'basket_grasp', 'yogurt', 'broom_shovel', 'shovel', 'watering_can', 'pen_holder', 'palette', 'handled_paint', 'tape_roll'}
PE_CATS = {'shampoo', 'body_lotion', 'toothpaste', 'glue'}
sub_counts = collections.Counter()
for m in models:
    for k in ('polymer_subtype', 'polymer_subtype_basis'):
        m.pop(k, None)
    if m['material_class'] != 'polyolefin_plastic':
        continue
    txt = ((m['caption_material'] or '') + ' ' + (m['caption_name'] or '')) if m['caption_status'] == 'used' else ''
    if re.search(r'\bPP\b|polypropylene', txt, re.I) and not re.search(r'\bPE\b|HDPE|LDPE|polyethylene', txt, re.I):
        st, basis = 'pp', 'named in caption'
    elif re.search(r'\bPE\b|HDPE|LDPE|polyethylene', txt, re.I) and not re.search(r'\bPP\b|polypropylene', txt, re.I):
        st, basis = 'pe', 'named in caption'
    elif m['category'] in PP_CATS:
        st, basis = 'pp_probable', 'category: rigid moulded item or film (PP typical); not stated in caption'
    elif m['category'] in PE_CATS:
        st, basis = 'pe_probable', 'category: squeeze tube/bottle (PE typical); not stated in caption'
    else:
        st, basis = 'unspecified', 'no polymer evidence in caption or category'
    if m['id'] == 'Rigid/cup/00001':
        st, basis = 'pp', 'real-robot frames of cover_blocks read as moulded PP cups (#63); synthetic caption says ceramic'
    if m['id'] == 'Rigid/tape_roll/00000':
        st, basis = 'pp_probable', 'packing-tape film is typically BOPP; caption says only "white packing tape"'
    m['polymer_subtype'] = st; m['polymer_subtype_basis'] = basis; sub_counts[st] += 1

# ---- v3 1f. stale reasons: where the class changed after step 1, the step-1 reason no longer explains it
stale = 0
for m in models:
    if m['material_class'] != m['prior_class'] and m['id'] not in ('Rigid/bottle/00022', 'Rigid/bottle/00047', 'Rigid/cup/00001', 'Rigid/tape_roll/00000', 'Clutter/toothpaste/00001', 'Rigid/shovel/00000'):
        m['prior_reason'] = m['reason']
        src = m['caption_material'] or m['caption_name']
        m['reason'] = f'class from caption: "{src}" (step-1 class {m["prior_class"]} replaced in step 2)'
        stale += 1
for i in ('Rigid/bottle/00047', 'Clutter/toothpaste/00001', 'Rigid/shovel/00000'):
    by[i].setdefault('prior_reason', {'Rigid/bottle/00047': 'drink bottles are PET', 'Clutter/toothpaste/00001': 'laminate/PE tube', 'Rigid/shovel/00000': 'steel blade'}[i])
fam_evidence = collections.Counter((m['class_evidence'], m['confidence']) for m in models if m['material_class'] in ABS)

# ================================================================ 2. class values (value and range carry separate statuses)
RANKV = {'sourced': 2, 'estimated': 1, 'unsourced': 0}
weaker = lambda a, b_: a if RANKV[a] <= RANKV[b_] else b_
vstat = lambda f: (f.get('check') or {}).get('verified_status') or f.get('status')


def within(rng, span, tol=0.01):
    return span is not None and rng[0] >= span[0] * (1 - tol) and rng[1] <= span[1] * (1 + tol)


def valfield(f):
    """Draft field -> v3 field: value_status, range_status and status (the weaker of the two)."""
    d = {k: copy.deepcopy(f[k]) for k in f if k not in ('check', 'status')}
    s = vstat(f)
    span = (f.get('check') or {}).get('confirmed_source_span')
    d['value_status'] = s
    d['range_status'] = s if s != 'sourced' else ('sourced' if within(f['range'], span) else 'estimated')
    if s == 'sourced':
        d['confirmed_source_span'] = span
    d['status'] = weaker(d['value_status'], d['range_status'])
    return d


def restatus(d, vs=None, rs=None, note=None):
    if vs: d['value_status'] = vs
    if rs: d['range_status'] = rs
    d['status'] = weaker(d['value_status'], d['range_status'])
    if d['value_status'] != 'sourced':
        d.pop('confirmed_source_span', None)
    if note: d.setdefault('v3_change', []).append(note)


def plain(value, rng, vs, rs, **kw):
    d = {'value': value, 'range': list(rng), 'value_status': vs, 'range_status': rs}
    d['status'] = weaker(vs, rs); d.update(kw)
    return d


V3FETCH = 'fetched 2026-11-11 with curl, raw HTML table cell'
cls_desc = dict(old['classes'])
cls_desc['pet_plastic'] += ' Assigned only where the caption names PET, shows clarity or identifies a PET-bottled product; "plastic" alone is not evidence of PET.'
cls_desc['polyolefin_plastic'] = ('PP/HDPE/LDPE containers, tubes, films and tools: waxy low-friction semi-gloss surface, bulk 910-960 kg/m3; hollow packaging and bins. '
                                  'Kept as one class: available friction sources do not separate PP from PE. Per-model polymer_subtype records pp/pe where a caption or the real frames state it.')
SURF = {
    'pad_hard': {'description': 'Gripper finger face: smooth hard thermoplastic (dark grey), per Bo\'s reading of real ARX X5 frames; moderate confidence.', 'role': 'finger_pad', 'scenario_membership': 'default'},
    'pad_elastomer': {'description': 'Gripper finger face with adhered silicone/rubber/TPU grip pad (Shore A 60-90).', 'role': 'finger_pad', 'scenario_membership': 'sensitivity'},
    'table_laminate': {'description': 'Table top: smooth white melamine/laminate board.', 'role': 'table', 'scenario_membership': 'default'},
    'table_wood': {'description': 'Table top: finished/lacquered mahogany or hardwood (matches the simulation visual shader).', 'role': 'table', 'scenario_membership': 'sensitivity'},
    'ground': {'description': 'Floor under the table (layout Ground prim, material_0564). Real floor material is unidentified. Layout declares static=dynamic friction 1.0, restitution 0.0, both combine modes "multiply".', 'role': 'ground', 'scenario_membership': 'fixed'},
}
OBJ = list(old['classes'])
class_vals = {}
for c, v in ver['classes'].items():
    class_vals[c] = {k: valfield(x) for k, x in v.items() if isinstance(x, dict) and 'value' in x}
    if 'notes' in v:
        class_vals[c]['notes'] = v['notes']
for c in ('mixed_unclear', 'liquid'):
    class_vals[c] = {'notes': 'No density/friction value assigned: ' + ('particles, no solid contact surface' if c == 'liquid' else 'no single contact material')}
# densities: v3 decisions (review items 2 and 7); every range_status is decided against the fetched span of the class's members
D = lambda c: class_vals[c]['density_kg_m3']
restatus(D('pet_plastic'), 'sourced', 'estimated', 'Point 1350 is the single sourced number (1.35); the [1330, 1400] amorphous-to-crystalline band is an estimate (review item 2; PR #84 agrees).')
pol = D('polyolefin_plastic'); pol['range'] = [910, 960]
pol['sources'] = [{'url': 'https://www.engineeringtoolbox.com/density-solids-d_1265.html', 'quote': 'LDPE - low density poly ethylene | 0.91 ; HDPE - (PEH) - high density poly ethylene | 0.96 ; PP - poly propylene | 0.91 - 0.94',
                   'conditions': f'bulk solid resins, 10^3 kg/m3; {V3FETCH}'}]
pol['confirmed_source_span'] = [910, 960]
restatus(pol, 'sourced', 'sourced', 'Range [900, 960] -> [910, 960], the exact source span (adopted from PR #84; 900 had no source).')
st = D('steel'); st['range'] = [7480, 8750]
st['sources'] = st.get('sources', []) + [
    {'url': 'https://www.engineeringtoolbox.com/density-solids-d_1265.html', 'quote': 'Brasses | 8.4 - 8.75', 'conditions': f'10^3 kg/m3; brass is a member of this class; {V3FETCH}'},
    {'url': 'https://www.engineeringtoolbox.com/metal-alloys-densities-d_50.html', 'quote': 'Stainless Steel | 7480 - 8000', 'conditions': f'kg/m3; stainless is a member of this class; {V3FETCH}'}]
st['confirmed_source_span'] = [7480, 8750]
restatus(st, 'sourced', 'sourced', 'Range [7700, 8500] (estimated) -> [7480, 8750], the fetched span of the class members (stainless 7480-8000, steel 7820, brass 8400-8750). '
         'PR #84\'s [7820, 7820] is a point for one bottle, narrower than the class.')
sw = D('softwood'); sw['confirmed_source_span'] = [320, 780]
for x in sw.get('sources', []):
    if x['quote'].lower().startswith('basswood'):
        x['check'] = dict(x.get('check') or {}, verdict='confirmed_under_class_definition', note_v3='basswood (linden) is a softwood-class member by definition (classes.md); botanically a hardwood')
restatus(sw, 'sourced', 'sourced', 'Basswood (linden) 320-590 is a member of this class by definition (classes.md), so the range is inside the fetched member span.')
ew = D('engineered_wood'); ew['confirmed_source_span'] = [540, 800]
restatus(ew, 'sourced', 'sourced', 'Plywood 540 (Engineering ToolBox) and MDF 710-800 (SwissKrono) are both members; range [540, 800] equals their span.')
gl = D('glass'); gl['confirmed_source_span'] = [2210, 2800]
for x in gl.get('sources', []):
    if 'Pyrex' in x['quote']:
        x['check'] = dict(x.get('check') or {}, verdict='superseded', note_v3='re-fetched 2026-11-11: the page row reads "Glass, Pyrex | 2.21", not 2.25')
gl['sources'].append({'url': 'https://www.engineeringtoolbox.com/density-solids-d_1265.html', 'quote': 'Glass, Pyrex | 2.21', 'conditions': f'10^3 kg/m3; {V3FETCH}'})
restatus(gl, 'sourced', 'sourced', 'Glass, common 2.4-2.8 and Pyrex 2.21 (Engineering ToolBox, re-fetched) span [2210, 2800]; 2200 is within 1%.')
restatus(D('rubber_elastomer'), 'sourced', 'estimated', 'Point 1100 matches "Rubber, soft commercial | 1.1"; the top 1500 exceeds the fetched rubber rows (to 1.2) and is an estimate for filled TPR/PVC elastomers.')
# restitution (review item 7: see alignment_84.json)
class_vals['ground'] = {'restitution': plain(0.0, [0.0, 0.3], 'unsourced', 'unsourced',
                                             basis='Real floor unidentified (visual Wood_Tiles_Fineline). v2 had [0, 0] = the shipped layout declaration, which belongs to the shipped-vs-corrected diff, not to a real-world band; '
                                                   'adopted PR #84\'s [0.0, 0.3]. Shipped value: restitution 0.0, multiply combine.')}

# v3.1 (#88 item 4): a restitution field whose own check says conditions_mismatch cannot stay "estimated"; status follows the verdict
for c, x in class_vals.items():
    f = x.get('restitution')
    if f and f['status'] == 'estimated' and any((s_.get('check') or {}).get('verdict') == 'conditions_mismatch' for s_ in f.get('sources', [])):
        restatus(f, 'unsourced', 'unsourced', 'v3.1 (#88 item 4): its only source has check verdict conditions_mismatch (' + '; '.join(s_['check']['note'] for s_ in f['sources']) +
                 '), so the value and band are unsourced, not estimated.')


# v3.1 (#88 item 5): restitution under the average combine rule. Every value here is unsourced.
def set_rest(c, value, rng, why, full=None):
    f = class_vals[c]['restitution']
    f['value'], f['range'] = value, list(rng)
    restatus(f, 'unsourced', 'unsourced', f'v3.1 (#88 item 5): {value} {list(rng)}; {why}')
    if full:
        f['fill_overrides'] = {'full': {'value': full[0], 'range': list(full[1]), 'value_status': 'unsourced', 'range_status': 'unsourced', 'status': 'unsourced',
                                        'why': 'bottle or container filled with liquid (#85): the contents damp the wall, so the bounce is lower than the empty shell'}}
        f['value_is_for_fill'] = 'empty'


TABLE_WHY = ('under restitution_combine_mode "average" a 0.40 table forces e_eff >= 0.20 on foam, fabric and full bottles, so the table gets a low counterface value')
set_rest('table_laminate', 0.10, [0.05, 0.40], TABLE_WHY)
set_rest('table_wood', 0.10, [0.05, 0.40], TABLE_WHY + '; the "wooden block on concrete" source stays attached as a mismatched anchor')
SHELL_WHY = 'moderate empty-shell value; a filled container is a per-asset fill override (#85)'
set_rest('pet_plastic', 0.25, [0.15, 0.55], SHELL_WHY, full=(0.10, [0.05, 0.20]))
set_rest('polyolefin_plastic', 0.25, [0.15, 0.55], SHELL_WHY, full=(0.10, [0.05, 0.20]))
PAD_WHY = ('grasp closing speeds stay under the 0.5 m/s bounce threshold, so pad restitution is dormant except in fast grazes and conveyor tasks')
set_rest('pad_hard', 0.10, [0.0, 0.35], PAD_WHY)
set_rest('pad_elastomer', 0.10, [0.0, 0.30], PAD_WHY)

# ================================================================ 3. pairs: canonical, deduplicated, revised
canon = lambda a, b_: tuple(sorted((a, b_)))
cand = collections.defaultdict(list)
for p in ver['pairs']:
    cand[canon(p['a'], p['b'])].append(p)
pairs, dups = {}, []
for k, lst in cand.items():
    lst2 = sorted(lst, key=lambda p: (RANKV[vstat(p['mu_static'])] + RANKV[vstat(p['mu_kinetic'])], 1 if p['group'] != 'G4' else 0), reverse=True)
    keep = lst2[0]
    e = {'a': k[0], 'b': k[1], 'origin': 'existing', 'source_group': keep['group'],
         'mu_static': valfield(keep['mu_static']), 'mu_kinetic': valfield(keep['mu_kinetic'])}
    if len(lst2) > 1:
        e['superseded_duplicates'] = [{'group': q['group'], 'mu_static': [q['mu_static']['value'], q['mu_static']['range'], vstat(q['mu_static'])],
                                       'mu_kinetic': [q['mu_kinetic']['value'], q['mu_kinetic']['range'], vstat(q['mu_kinetic'])]} for q in lst2[1:]]
        dups.append(k)
    pairs[k] = e
n_input_pairs = len(ver['pairs'])

# v2: PP-PP downgraded and widened
pp = pairs[('polyolefin_plastic', 'polyolefin_plastic')]
pp['mu_static'].update({'value': 0.3, 'range': [0.1, 0.71]}); restatus(pp['mu_static'], 'estimated', 'estimated')
pp['mu_static']['reason_v2'] = ('v1: 0.2 [0.1,0.35] "sourced" from a PE-PE row (EngineeringToolbox, printed film) and an unlabelled PP column. '
                                'Lee 1995 (Table K.1) self-mated moulded PP is 0.218-0.388 textured and 0.307-0.706 smooth, above the v1 top. '
                                'Band now spans the PE/PP handbook low end 0.1 to 0.71; nominal 0.3 sits at the textured-top / smooth-bottom overlap and at HDPE 0.29. Downgraded to estimated: no source tests mixed PP/PE surfaces of these assets.')
pp['mu_kinetic'].update({'value': 0.25, 'range': [0.08, 0.6]}); restatus(pp['mu_kinetic'], 'estimated', 'estimated')
pp['mu_kinetic']['reason_v2'] = 'Kinetic scaled from static (about 0.8x top, 0.8x nominal); no kinetic source. Widened with the static band.'
pp['mu_static'].setdefault('sources', []).append({'url': 'Lee 1995 (Table K.1), as cited in #84 review section 3', 'quote': 'self-mated moulded PP: 0.218-0.388 textured, 0.307-0.706 smooth',
                                                   'conditions': 'as reported by the #84 review (fetched there); not re-fetched here', 'check': {'verdict': 'reported_by_review'}})

# v3 (review item 2): condition-mismatched "sourced" friction -> estimated; kinetic nominals and sources made consistent
WOOD_NOTE = ('Sources measure bare, unfinished, dry wood (mostly oak on oak); softwood is painted/lacquered and hardwood and table_wood are sealed/oiled or lacquered, '
             'so the number is a proxy: estimated, not sourced (#73 review item 2).')
STATIC_ONLY = ('Wood Clean Wood Clean and Dry 0.25 - 0.5', 'The coefficient of friction between two dry samples of wood particles')
downgraded = []
for key, flds in {('hardwood', 'hardwood'): 'sk', ('hardwood', 'softwood'): 'sk', ('hardwood', 'table_wood'): 'sk', ('softwood', 'softwood'): 'sk',
                  ('softwood', 'table_wood'): 'sk', ('softwood', 'steel'): 's'}.items():
    for f in flds:
        fld = pairs[key]['mu_static' if f == 's' else 'mu_kinetic']
        restatus(fld, 'estimated', 'estimated', WOOD_NOTE); downgraded.append(f'{key[0]}__{key[1]}:{"mu_static" if f == "s" else "mu_kinetic"}')
        if f == 'k':
            fld['sources'] = [s for s in fld.get('sources', []) if not s['quote'].startswith(STATIC_ONLY)]
            if fld['value'] == 0.3:
                fld['value'] = 0.32
                fld['v3_change'].append('Kinetic nominal 0.30 -> 0.32: 0.30 sat below every cited kinetic number (bare oak 0.32-0.48); the two static-only quotes '
                                        '(Engineering ToolBox "Clean Wood 0.25 - 0.5", arXiv 1405.3049 inclined plane) were removed from the kinetic sources.')
alal = pairs[('aluminium', 'aluminium')]['mu_kinetic']
restatus(alal, 'estimated', 'estimated', 'Sources are clean, bare, dry aluminium (galling regime): Engineering ToolBox gives kinetic 0.4, Roymech gives dry sliding 1.4 for the same pair; '
         'the class is anodised or coated alloy. 0.4 [0.3, 0.6] is an estimate for coated surfaces, not a sourced value (#73 review item 2).')
downgraded.append('aluminium__aluminium:mu_kinetic')
absabs = pairs[('abs_toy_plastic', 'abs_toy_plastic')]['mu_static']
restatus(absabs, 'estimated', 'estimated', 'Sources are polystyrene on polystyrene and an unlabelled Professional Plastics column (no counterface, no static/kinetic label), '
         'rejected for PP and PET by the #84 review; ABS on ABS is unmeasured: estimated (#73 review item 2).')
downgraded.append('abs_toy_plastic__abs_toy_plastic:mu_static')

# ---- co-occurrence: exact, from every layout file of all 54 configs and 3 seeds
cfgs = {c['task_name']: c for c in pop['configs']}
cls_of = {m['id']: m['material_class'] for m in models}
SEC = ('Rigid', 'Geometry', 'Articulation', 'Garment', 'Fluid', 'Dynamic')
pop_ok = {}   # (task, model) -> set of scored-allowed ids from population.json
for t, c in cfgs.items():
    s = set()
    for o in c['assets']['objects']:
        kind = o['asset_dir'].split('/')[3]
        for i_ in o['indices_scored_all_seeds']:
            s.add(f"{kind}/{o['category']}/{i_:05d}")
    pop_ok[t] = s


def model_id(sec, cat, inst):
    """As env/scene_manager/layout_manager.py:164-167: a 'cluttered' instance loads Clutter/<cat>/<idx>."""
    kind = 'Clutter' if inst.get('type') == 'cluttered' else sec
    return f"{kind}/{cat}/{inst.get('category_idx', 0):05d}"


layouts = []   # (task, seed, layout_id, scored, [(section, category, inst, model_id)])
integrity = {'files': 0, 'unparsed': [], 'not_in_population': collections.Counter(), 'unknown_models': collections.Counter()}
blobs = collections.defaultdict(set)
layout_files = []   # (seed, task, layout id, path): exactly the files population.json lists, in seed then file-name order
for t, c in cfgs.items():
    lay = c['layouts']
    for seed in lay['official_leaderboard_seeds']:
        for lid in lay['scored_layout_ids'] + lay['spare_layout_ids']:
            rel = lay['seed_to_layout_template'].format(seed=seed, layout_id=lid)
            layout_files.append((seed, t, lid, rel))
layout_files.sort(key=lambda x: (x[0], os.path.basename(x[3])))
for seed, t, lid, rel in layout_files:
    f = os.path.join(args.layouts, rel)
    raw = open(f, 'rb').read()
    blobs[t].add(hashlib.sha256(raw).hexdigest())
    d = json.loads(raw)
    scored = lid in cfgs[t]['layouts']['scored_layout_ids']
    inst = []
    for sec in SEC:
        for cat, lst in (d.get(sec) or {}).items():
            for o in lst:
                mid = model_id(sec, cat, o)
                inst.append((sec, cat, o, mid))
                if mid not in cls_of:
                    integrity['unknown_models'][mid] += 1
                if scored and mid not in pop_ok[t]:
                    integrity['not_in_population'][(t, mid)] += 1
    layouts.append((t, seed, lid, scored, inst))
    integrity['files'] += 1
integrity['layouts_per_seed'] = dict(collections.Counter(s for _, s, *_ in layouts))
integrity['scored_layouts'] = sum(1 for x in layouts if x[3])
integrity['blob_count_mismatch_vs_population'] = {t: [len(blobs[t]), cfgs[t]['layouts']['unique_lfs_blobs_all_seeds']] for t in cfgs if len(blobs[t]) != cfgs[t]['layouts']['unique_lfs_blobs_all_seeds']}
integrity['scored_instances_not_in_population_scored_assets'] = sum(integrity['not_in_population'].values())
integrity['not_in_population'] = [f'{t} {m}: {n}' for (t, m), n in integrity['not_in_population'].most_common(20)]
integrity['unknown_models'] = dict(integrity['unknown_models'])
assert integrity['files'] == pop['counts']['layout_files_all_seeds'], integrity['files']
assert integrity['scored_layouts'] == pop['counts']['scored_episodes_all_3_seeds'], integrity['scored_layouts']

SURFC = list(SURF)


def pairs_in(inst):
    cl = [cls_of[mid] for sec, cat, o, mid in inst if sec not in ('Garment', 'Fluid') and mid in cls_of and cls_of[mid] != 'liquid']
    seen = set()
    for i in range(len(cl)):
        for j in range(i + 1, len(cl)):
            seen.add(canon(cl[i], cl[j]))
    for a in set(cl):
        for s in SURFC:
            seen.add(canon(a, s))
    return seen


ep_scored = collections.Counter(); ep_spare = collections.Counter(); ep_seed = collections.defaultdict(collections.Counter)
ep_cfg = collections.defaultdict(collections.Counter)
for t, seed, lid, scored, inst in layouts:
    for k in pairs_in(inst):
        if scored:
            ep_scored[k] += 1; ep_seed[k][seed] += 1; ep_cfg[k][t] += 1
        else:
            ep_spare[k] += 1
cooc = set(ep_scored) | set(ep_spare)


# ---- fill co-occurring pairs missing from the table (v2 rule)
def getp(a, b_):
    return pairs.get(canon(a, b_))


def exact_mean3(xs):
    """Mean of decimal literals rounded to 3 places (ties to even, as round() does) in exact arithmetic: float sum() differs between Python versions (3.12 compensates), so ties at .xx25 and .xx75 must not depend on it."""
    return float(round(sum((fractions.Fraction(decimal.Decimal(repr(x))) for x in xs), fractions.Fraction(0)) / len(xs), 3))


def hull_estimate(a, b_, field):
    refs = [(x, s, getp(x, s)[field]) for x in (a, b_) for s in ('pad_hard', 'table_laminate') if getp(x, s)]
    if len(refs) < 4:
        return None
    lo = min(r[2]['range'][0] for r in refs); hi = max(r[2]['range'][1] for r in refs)
    return plain(exact_mean3([r[2]['value'] for r in refs]), [lo, hi], 'estimated', 'estimated',
                 derivation='hull of the two classes\' bands against pad_hard and table_laminate (' + ', '.join(f'{x}-{s}' for x, s, _ in refs) + '); nominal = mean of their four nominals',
                 reasoning='No pair measurement exists. Both classes were rated against the same hard smooth counterfaces; a mixed pair is expected to fall between their per-counterface values. The hull is deliberately wide.')


UNS_BAND = {'mu_static': ([0.15, 0.8], 0.4), 'mu_kinetic': ([0.1, 0.7], 0.3)}
OBJC = [c for c in OBJ if c != 'liquid']
new_pairs = []
for a, b_ in itertools.combinations_with_replacement(sorted(OBJC + SURFC), 2):
    k = canon(a, b_)
    if (a in SURFC and b_ in SURFC) or k in pairs or k not in cooc:
        continue
    e = {'a': k[0], 'b': k[1], 'origin': 'new_v2'}
    for fld in ('mu_static', 'mu_kinetic'):
        if 'mixed_unclear' in k:
            r, nom = UNS_BAND[fld]
            e[fld] = plain(nom, r, 'unsourced', 'unsourced', reasoning='mixed_unclear has no assignable contact material; band is a placeholder spanning typical dry polymer/wood/metal contacts.')
        elif 'ground' in k:
            other = k[0] if k[1] == 'ground' else k[1]
            ps = [p[fld] for p in (getp(other, 'table_laminate'), getp(other, 'table_wood')) if p]
            if len(ps) == 2:
                e[fld] = plain(round(sum(p['value'] for p in ps) / 2, 3), [min(p['range'][0] for p in ps), max(p['range'][1] for p in ps)], 'unsourced', 'unsourced',
                               reasoning='Floor material unidentified. Placeholder: hull of the class\'s table_laminate and table_wood bands (both hard smooth horizontal surfaces).')
            else:
                e[fld] = plain(UNS_BAND[fld][1], UNS_BAND[fld][0], 'unsourced', 'unsourced', reasoning='No reference bands available.')
        else:
            e[fld] = hull_estimate(k[0], k[1], fld) or plain(UNS_BAND[fld][1], UNS_BAND[fld][0], 'unsourced', 'unsourced', reasoning='Missing reference bands for the hull rule; generic placeholder.')
    pairs[k] = e; new_pairs.append(k)

# v3 (review item 1): aluminium-steel from the Engineering ToolBox / Roymech row instead of the plastic-counterface hull
als = pairs[('aluminium', 'steel')]
AL_SRC = [{'url': 'https://www.engineeringtoolbox.com/friction-coefficients-d_778.html', 'quote': 'Aluminum | Mild Steel | Clean and Dry | 0.61 | 0.47',
           'conditions': f'bare aluminium on mild steel, clean and dry, static then kinetic; {V3FETCH}'},
          {'url': 'https://roymech.org/Useful_Tables/Tribology/co_of_frict.html', 'quote': 'Aluminum | Mild Steel | 0,61 | 0,47',
           'conditions': f'dry static then dry sliding; {V3FETCH}'}]
als['origin'] = 'v3'
als['mu_static'] = plain(0.5, [0.25, 0.7], 'estimated', 'estimated', sources=AL_SRC,
                         derivation='Bare clean dry Al on mild steel is 0.61 static (two compilations). The class covers anodised or coated alloy against bare or plated steel, '
                                    'unmeasured, so the nominal sits below 0.61 and the band runs from coated contacts (0.25) to above the bare value (0.7).',
                         v3_change=['v2 hull [0.2, 0.5] from plastic/laminate counterfaces capped metal-metal below the bare value 0.61 and, with aluminium__aluminium >= 0.4 and '
                                    'steel__steel >= 0.5, forced the global static margin to 0.025 (#73 review item 1).'])
als['mu_kinetic'] = plain(0.4, [0.2, 0.55], 'estimated', 'estimated', sources=AL_SRC,
                          derivation='Bare clean dry Al on mild steel kinetic 0.47; coated contacts lower; nominal 0.4 and band [0.2, 0.55] keep kinetic below static.',
                          v3_change=['v2 hull [0.15, 0.45] excluded the bare value 0.47.'])


def membership(k):
    s = [x for x in k if x in SURF]
    return SURF[s[0]]['scenario_membership'] if s else 'all'


for k, e in pairs.items():
    e['key'] = f'{k[0]}__{k[1]}'
    e['kind'] = 'object-surface' if any(x in SURF for x in k) else 'object-object'
    e['scenario_membership'] = membership(k)
    e['cooccurrence'] = {'episodes_scored_3seeds': ep_scored.get(k, 0), 'episodes_scored_by_seed': [ep_seed[k][s] for s in (0, 1, 2)],
                         'configs_scored': len(ep_cfg[k]), 'episodes_scored_by_config': dict(sorted(ep_cfg[k].items())),
                         'episodes_spare_3seeds': ep_spare.get(k, 0), 'cooccurs_in_scored_episodes': ep_scored.get(k, 0) > 0}
    if not ep_scored.get(k) and not ep_spare.get(k):
        e['cooccurrence']['note'] = 'never co-occurs in any layout of the 54 configs (kept from v1; not needed by any task)'
    for fld in ('mu_static', 'mu_kinetic'):
        f = e[fld]
        assert f['status'] in RANKV and f['value_status'] in RANKV and f['range_status'] in RANKV, (k, fld)
        assert f['range'][0] <= f['value'] <= f['range'][1], (k, fld, f['value'], f['range'])
    assert e['mu_kinetic']['value'] <= e['mu_static']['value'] + 1e-9, (k, 'nominal kinetic > static')

# ================================================================ 4. unary feasibility: joint static/kinetic LP (pure-python two-phase simplex)


def lp_max(c, A, b_, eps=1e-9):
    """maximise c.x subject to A x <= b, x >= 0 (b of any sign). Dense two-phase simplex with Bland's rule.
    Returns (x, value) or (None, None) if infeasible."""
    m, n = len(A), len(c)
    arts = [i for i in range(m) if b_[i] < 0]
    k = len(arts); N = n + m + k
    T, basis, ai = [], [], 0
    for i in range(m):
        row = [0.0] * (N + 1)
        if b_[i] >= 0:
            row[:n] = A[i]; row[n + i] = 1.0; row[-1] = b_[i]; basis.append(n + i)
        else:
            row[:n] = [-v for v in A[i]]; row[n + i] = -1.0; row[n + m + ai] = 1.0; row[-1] = -b_[i]; basis.append(n + m + ai); ai += 1
        T.append(row)

    def pivot(r, col, z):
        pv = T[r][col]; T[r] = [v / pv for v in T[r]]
        tr = T[r]
        for i in range(len(T)):
            if i != r:
                f = T[i][col]
                if abs(f) > 1e-15:
                    T[i] = [vi - f * vr for vi, vr in zip(T[i], tr)]
        f = z[col]
        if abs(f) > 1e-15:
            z[:] = [vi - f * vr for vi, vr in zip(z, tr)]
        basis[r] = col

    def run(z, ncols):
        while True:
            col = next((j for j in range(ncols) if z[j] < -eps), None)
            if col is None:
                return
            row, best = None, None
            for i in range(len(T)):
                if T[i][col] > eps:
                    r = T[i][-1] / T[i][col]
                    if best is None or r < best - 1e-12 or (abs(r - best) <= 1e-12 and basis[i] < basis[row]):
                        best, row = r, i
            if row is None:
                raise RuntimeError('unbounded LP')
            pivot(row, col, z)

    if k:
        z = [0.0] * (N + 1)
        for j in range(n + m, N):
            z[j] = 1.0
        for i in range(m):
            if basis[i] >= n + m:
                z = [zj - tj for zj, tj in zip(z, T[i])]
        run(z, N)
        if z[-1] < -1e-7:
            return None, None
        for i in range(len(T)):   # drive degenerate artificials out of the basis
            if basis[i] >= n + m:
                col = next((j for j in range(n + m) if abs(T[i][j]) > 1e-9), None)
                if col is not None:
                    pivot(i, col, z)
        keep = [i for i in range(len(T)) if basis[i] < n + m]
        T[:] = [T[i][:n + m] + [T[i][-1]] for i in keep]; basis[:] = [basis[i] for i in keep]
    cc = list(c) + [0.0] * m
    z = [-v for v in cc] + [0.0]
    for i, bi in enumerate(basis):
        if cc[bi]:
            z = [zj + cc[bi] * tj for zj, tj in zip(z, T[i])]
    run(z, n + m)
    x = [0.0] * (n + m)
    for i, bi in enumerate(basis):
        x[bi] = T[i][-1]
    return x[:n], z[-1]


SHIFT, UMAX = 2.0, 3.0


class Problem:
    """Band rows lo + t_f <= sum(coef*u) <= hi - t_f for field f in {s, k}; coupling u_k[c] <= u_s[c] for every class.
    Margins are in effective-friction units (the realised pair coefficient)."""

    def __init__(self, rows, classes, fixed=None):
        self.rows, self.classes, self.fixed = rows, classes, fixed or {}
        self.var = {}
        for c in classes:
            for f in 'sk':
                if (f, c) not in self.fixed:
                    self.var[(f, c)] = len(self.var)
        self.var['Ts'] = len(self.var); self.var['Tk'] = len(self.var)

    def _lin(self, f, coefs):
        row = [0.0] * len(self.var); const = 0.0
        for c, w in coefs.items():
            if (f, c) in self.fixed:
                const += w * self.fixed[(f, c)]
            else:
                row[self.var[(f, c)]] += w
        return row, const

    def _band_rows(self, idx, margin):
        A, b_ = [], []
        for i in idx:
            r = self.rows[i]; f = r['field']
            lin, const = self._lin(f, r['coefs'])
            tv = margin.get(f)
            if r['lo'] is not None:
                row = [-v for v in lin]; rhs = -(r['lo'] - const)
                if tv is None: row[self.var['T' + f]] += 1.0; rhs += SHIFT
                else: rhs -= tv
                A.append(row); b_.append(rhs)
            if r['hi'] is not None:
                row = lin[:]; rhs = r['hi'] - const
                if tv is None: row[self.var['T' + f]] += 1.0; rhs += SHIFT
                else: rhs -= tv
                A.append(row); b_.append(rhs)
        return A, b_

    def solve(self, objective, margin, idx_all=None):
        """objective: 'joint' (Ts=Tk=t), 's' or 'k'. margin: {'s': fixed value or None, 'k': ...}. Constraint generation over band rows."""
        idx_all = list(range(len(self.rows))) if idx_all is None else idx_all
        active = set(i for i in idx_all if self.rows[i]['seed'])
        nv = len(self.var)
        while True:
            A, b_ = self._band_rows(sorted(active), margin)
            for c in self.classes:   # coupling and bounds
                lk, ck = self._lin('k', {c: 1.0}); ls, cs = self._lin('s', {c: 1.0})
                A.append([x - y for x, y in zip(lk, ls)]); b_.append(cs - ck)
                for f in 'sk':
                    if (f, c) not in self.fixed:
                        row = [0.0] * nv; row[self.var[(f, c)]] = 1.0; A.append(row); b_.append(UMAX)
            for T_ in ('Ts', 'Tk'):
                row = [0.0] * nv; row[self.var[T_]] = 1.0; A.append(row); b_.append(SHIFT + 1.0)
            if objective == 'joint':
                row = [0.0] * nv; row[self.var['Ts']] = 1.0; row[self.var['Tk']] = -1.0; A.append(row); b_.append(0.0)
                A.append([-v for v in row]); b_.append(0.0)
            cvec = [0.0] * nv; cvec[self.var['Ts' if objective in ('joint', 's') else 'Tk']] = 1.0
            x, val = lp_max(cvec, A, b_)
            if x is None:
                return None
            u = {f: {c: (self.fixed[(f, c)] if (f, c) in self.fixed else x[self.var[(f, c)]]) for c in self.classes} for f in 'sk'}
            t = {f: (margin[f] if margin.get(f) is not None else x[self.var['T' + f]] - SHIFT) for f in 'sk'}
            viol = [i for i in idx_all if i not in active and self.row_margin(i, u) < t[self.rows[i]['field']] - 1e-9]
            if not viol:
                return {'u': u, 't': t}
            active |= set(viol)

    def row_margin(self, i, u):
        r = self.rows[i]; v = sum(w * u[r['field']][c] for c, w in r['coefs'].items())
        return min(v - r['lo'] if r['lo'] is not None else 9.0, r['hi'] - v if r['hi'] is not None else 9.0)

    def realised(self, i, u):
        r = self.rows[i]
        return sum(w * u[r['field']][c] for c, w in r['coefs'].items())


def build_rows(pad, tab, pad_mode, ground_mode):
    rows = []
    for k, e in pairs.items():
        if not e['cooccurrence']['cooccurs_in_scored_episodes']:
            continue
        if any(x in SURF and x not in (pad, tab, 'ground') for x in k):
            continue
        a, b_ = k
        for f, fld in (('s', 'mu_static'), ('k', 'mu_kinetic')):
            lo, hi = e[fld]['range']; lab = e['key']; seed = a == b_ or 'ground' in k or pad in k or tab in k
            if 'ground' in k and ground_mode == 'multiply':   # ground unary 1.0, multiply: realised = u_obj
                o = a if b_ == 'ground' else b_
                rows.append({'coefs': {o: 1.0}, 'lo': lo, 'hi': hi, 'field': f, 'label': f'{lab} (multiply, ground 1.0)', 'seed': seed})
            elif pad in k and pad_mode == 'max':                # realised = max(u_obj, u_pad): u_pad in band, u_obj <= top
                o = a if b_ == pad else b_
                rows.append({'coefs': {pad: 1.0}, 'lo': lo, 'hi': hi, 'field': f, 'label': f'{lab} (max: u_pad in band)', 'seed': seed})
                rows.append({'coefs': {o: 1.0}, 'lo': None, 'hi': hi, 'field': f, 'label': f'{lab} (max: u_obj <= top)', 'seed': seed})
            else:
                coefs = {a: 1.0} if a == b_ else {a: 0.5, b_: 0.5}
                rows.append({'coefs': coefs, 'lo': lo, 'hi': hi, 'field': f, 'label': lab, 'seed': seed})
    classes = sorted({c for r in rows for c in r['coefs']})
    return rows, classes


def run_cell(pad, tab, pad_mode, ground_mode, critical=True):
    rows, classes = build_rows(pad, tab, pad_mode, ground_mode)
    P = Problem(rows, classes)
    j = P.solve('joint', {'s': None, 'k': None})
    if j is None:
        return {'feasible': False, 'note': 'LP infeasible even with negative margins (should not happen)'}
    tj = j['t']['s']
    s = P.solve('s', {'s': None, 'k': tj - 1e-7})
    ts = s['t']['s']
    kk = P.solve('k', {'s': ts - 1e-7, 'k': None})
    tk = kk['t']['k']; u = kk['u']
    out = {'feasible': tj >= -1e-9, 'joint_min_margin_mu': round(tj, 4), 'static_margin_mu': round(ts, 4), 'kinetic_margin_mu': round(tk, 4),
           'n_band_rows': len(rows), 'n_classes': len(classes),
           'unary_static': {c: round(u['s'][c], 4) for c in classes}, 'unary_kinetic': {c: round(u['k'][c], 4) for c in classes},
           'kinetic_equals_static': sorted(c for c in classes if abs(u['k'][c] - u['s'][c]) < 1e-7),
           'kinetic_above_static': sorted(c for c in classes if u['k'][c] > u['s'][c] + 1e-9)}
    assert not out['kinetic_above_static']
    viol = [i for i in range(len(rows)) if P.row_margin(i, u) < -1e-7]
    out['violations'] = [{'row': rows[i]['label'], 'field': rows[i]['field'], 'band': [rows[i]['lo'], rows[i]['hi']], 'realised': round(P.realised(i, u), 4)} for i in viol]
    bind = {f: [i for i in range(len(rows)) if rows[i]['field'] == f and P.row_margin(i, u) <= (ts if f == 's' else tk) + 1e-7] for f in 'sk'}
    out['binding_rows'] = {('static' if f == 's' else 'kinetic'): [{'row': rows[i]['label'], 'band': [rows[i]['lo'], rows[i]['hi']], 'realised': round(P.realised(i, u), 4)} for i in bind[f]] for f in 'sk'}
    if critical:
        crit = []
        for i in bind['s']:
            idx = [x for x in range(len(rows)) if x != i]
            r2 = P.solve('s', {'s': None, 'k': tj - 1e-7}, idx_all=idx)
            if r2 and r2['t']['s'] > ts + 1e-6:
                crit.append({'row': rows[i]['label'], 'band': [rows[i]['lo'], rows[i]['hi']], 'static_margin_without_it': round(r2['t']['s'], 4)})
        out['static_critical_rows'] = crit
    return out


surf_combos = [('pad_hard', 'table_laminate', 'default'), ('pad_elastomer', 'table_laminate', 'sensitivity_pad'),
               ('pad_hard', 'table_wood', 'sensitivity_table'), ('pad_elastomer', 'table_wood', 'sensitivity_both')]
feas = {'method': ('Joint LP over per-class unary friction u_s[c], u_k[c] >= 0 with u_k[c] <= u_s[c] for every class (objects including mixed_unclear, pads, tables, ground). '
                   'Under PhysX average combine a pair realises (u_a + u_b)/2, which must lie in the pair band. Stage 1 maximises the common minimum margin t over static and kinetic rows; '
                   'stage 2 maximises the static margin with kinetic rows held at margin >= t; stage 3 maximises the kinetic margin with static held at its stage-2 optimum. '
                   'Margins are in effective-friction units (the realised pair coefficient), not pair sums. Rows: every pair that co-occurs in a scored episode. '
                   'Ground "multiply" is the shipped ground (unary 1.0, multiply: realised = u_obj); "average" sets the ground material to average with its own unary. '
                   'pad_elastomer is also run in max mode (realised = max(u_obj, u_pad)). Pure-python two-phase simplex with constraint generation; no scipy.'),
        'combine_rule': 'PhysX effective friction combine mode = max(mode0, mode1) with average < min < multiply < max (PxMaterial.h); PhysX also clamps static >= dynamic. '
                        'The Ground prim in every layout sets friction 1.0 and combine mode multiply, so all object-ground pairs are multiply as shipped.',
        'scenarios': {}}
for pad, tab, tag in surf_combos:
    res = {}
    for pad_mode in (['average'] if pad == 'pad_hard' else ['average', 'max']):
        for gm in ('multiply', 'average'):
            res[f'pad_combine={pad_mode}:ground={gm}'] = run_cell(pad, tab, pad_mode, gm)
    feas['scenarios'][f'{pad}+{tab}'] = {'membership': tag, **res}
default_cell = feas['scenarios']['pad_hard+table_laminate']['pad_combine=average:ground=average']

# per-asset override interval for steel bottle 00022 against its actual partners in put_bottles_into_dustbin
tb = 'put_bottles_into_dustbin'
n22pairs, partners = 0, set()
for t, seed, lid, scored, inst in layouts:
    if t != tb or not scored:
        continue
    mids = [mid for sec, cat, o, mid in inst if sec not in ('Garment', 'Fluid')]
    if 'Rigid/bottle/00022' in mids:
        n22pairs += mids.count('Rigid/bottle/00022') >= 2
        partners |= {cls_of[x] for x in mids if x != 'Rigid/bottle/00022'}
partners = sorted(partners) + ['pad_hard', 'table_laminate', 'ground'] + (['steel'] if n22pairs else [])
ov = {'scored_layouts_with_two_or_more_00022': n22pairs, 'partners_used': partners}
for f, fld in (('s', 'mu_static'), ('k', 'mu_kinetic')):
    sol = default_cell['unary_static' if f == 's' else 'unary_kinetic']
    lo_, hi_, why = 0.0, UMAX, []
    for p_ in partners:
        e = pairs.get(canon('steel', p_))
        if e is None:
            continue
        l, h = e[fld]['range']; up = sol[p_]
        lo_, hi_ = max(lo_, 2 * l - up), min(hi_, 2 * h - up)
        why.append(f'{p_}: u in [{2 * l - up:.3f}, {2 * h - up:.3f}]')
    ov[fld] = {'allowed_unary_interval_for_00022': [round(lo_, 3), round(hi_, 3)], 'by_partner': why, 'class_steel_unary_in_solution': sol['steel']}
ov['note'] = 'The interval is for the stage-3 default solution (pad_hard, table_laminate, ground average); u_k <= u_s must also hold for the override.'
feas['per_asset_override_bottle_00022'] = ov

# PR #84's nominal-fit unary map: can it be extended to all classes? (fix its classes, maximise the joint margin for the rest)
ext84 = None
if slice84:
    um = slice84['physx_combine_and_unary_realisation']['unary_materials_average_combine']['classes']
    fixed = {}
    for c, v in um.items():
        if c in ('pad_elastomer', 'table_wood'):
            continue
        fixed[('s', c)] = v['static_friction']; fixed[('k', c)] = v['dynamic_friction']
    rows, classes = build_rows('pad_hard', 'table_laminate', 'average', 'average')
    P = Problem(rows, classes, fixed=fixed)
    j = P.solve('joint', {'s': None, 'k': None})
    viol84 = [{'row': rows[i]['label'], 'field': rows[i]['field'], 'band': [rows[i]['lo'], rows[i]['hi']], 'realised': round(P.realised(i, j['u']), 4)}
              for i in range(len(rows)) if P.row_margin(i, j['u']) < -1e-7] if j else None
    ext84 = {'fixed_from_84': {f'{c}': [fixed[('s', c)], fixed[('k', c)]] for (f, c) in fixed if f == 's'},
             'joint_min_margin_mu': round(j['t']['s'], 4) if j else None, 'extendable': bool(j) and j['t']['s'] >= -1e-9,
             'rows_below_band_at_best_extension': viol84,
             'unary_static_rest': {c: round(j['u']['s'][c], 4) for c in classes if ('s', c) not in fixed} if j else None,
             'unary_kinetic_rest': {c: round(j['u']['k'][c], 4) for c in classes if ('k', c) not in fixed} if j else None}
    feas['pr84_unary_map_extension'] = ext84

# ================================================================ 5. mass checks on every layout instance (all 3 seeds, scored and spare)
dens = {c: tuple(class_vals[c]['density_kg_m3']['range']) for c in class_vals if 'density_kg_m3' in class_vals[c]}
dstat = {c: class_vals[c]['density_kg_m3']['range_status'] for c in dens}
HOLLOW_FLAG = 'hollow_contents_dependent'


def check_instance(mid, mass, scale, ftype):
    mdl = by.get(mid)
    r = {'model': mid, 'class': mdl['material_class'] if mdl else None}
    if mdl is None:
        r['verdict'] = 'unresolved_model'; return r
    if ftype == 'geometry' or not mass:
        r['verdict'] = 'static_or_no_mass' if not mass else 'static_geometry'; return r
    cls = mdl['material_class']; e = mdl.get('bbox_extents_m')
    r['declared_layout_mass_kg'] = mass; r['metadata_mass_kg'] = mdl.get('declared_mass_kg')
    vb = e[0] * e[1] * e[2] * scale[0] * scale[1] * scale[2] if e else None
    r['bbox_volume_m3'] = vb
    mm_ = mdl.get('declared_mass_kg')
    r['layout_vs_metadata_ratio'] = round(mass / mm_, 3) if mm_ else None
    if mm_ and (mass / mm_ > 2.0 or mass / mm_ < 0.5):
        r['slip'] = f'layout/metadata mass ratio {mass / mm_:.3g}'
    if cls not in dens or not vb:
        r['verdict'] = 'no_density_band'; return r
    lo, hi = dens[cls]
    r['density_band_kg_m3'] = [lo, hi]
    rho = mass / vb; r['bbox_implied_density_kg_m3'] = round(rho, 1)
    if rho > hi:
        r['verdict'] = 'too_heavy_for_class'
    elif HOLLOW_FLAG in mdl['flags']:
        r['verdict'] = 'hollow_needs_fill_dependent_band'
    elif rho < 0.5 * lo:
        r['verdict'] = 'light_for_solid_class'
    else:
        r['verdict'] = 'plausible'
    return r


agg = collections.defaultdict(lambda: {'hits_scored': 0, 'hits_spare': 0, 'seeds': set(), 'scored_layout_ids': set()})
verdict_counts = collections.Counter(); fric_slips = collections.Counter(); checked = 0; inst_scored = 0
w22 = {'scored': set(), 'spare': set()}; bottle_slots = 0
for t, seed, lid, scored, inst in layouts:
    for sec, cat, o, mid in inst:
        if sec in ('Garment', 'Fluid'):
            verdict_counts['skipped_particle'] += 1; continue
        ph = o.get('physics', {}) or {}
        sc = o.get('scale')
        r = check_instance(mid, ph.get('mass'), sc if isinstance(sc, list) and len(sc) == 3 else [1, 1, 1], ph.get('type'))
        verdict_counts[r['verdict']] += 1; checked += 1; inst_scored += scored
        for nm in ('static_friction', 'dynamic_friction', 'friction'):
            val = ph.get(nm)
            if isinstance(val, (int, float)) and val > 3:
                fric_slips[(t, mid, nm, val)] += 1
        if t == tb and cat == 'bottle':
            bottle_slots += 1
            if mid == 'Rigid/bottle/00022' and ph.get('mass') == 22:
                w22['scored' if scored else 'spare'].add((seed, lid))
        if r['verdict'] in ('too_heavy_for_class', 'hollow_needs_fill_dependent_band', 'light_for_solid_class') or 'slip' in r:
            a = agg[(t, mid, r['verdict'], ph.get('mass'), r.get('slip'))]
            a['hits_scored' if scored else 'hits_spare'] += 1; a['seeds'].add(seed)
            if scored:
                a['scored_layout_ids'].add(lid)
            a.setdefault('example', {kk: r[kk] for kk in ('class', 'declared_layout_mass_kg', 'metadata_mass_kg', 'bbox_volume_m3', 'bbox_implied_density_kg_m3', 'density_band_kg_m3', 'layout_vs_metadata_ratio') if kk in r} | {'seed': seed, 'layout_id': lid})
            a['category'] = cat; a['inst_type'] = o.get('type', 'listed')


def group_list(pred):
    rs = []
    for (t, mid, verdict, mass, slip), v in agg.items():
        if pred(verdict, slip):
            rs.append({'task': t, 'model': mid, 'verdict': verdict, 'declared_layout_mass_kg': mass, 'slip': slip, 'category': v['category'], 'inst_type': v['inst_type'],
                       'hits_scored': v['hits_scored'], 'hits_spare': v['hits_spare'], 'seeds': sorted(v['seeds']),
                       'scored_layout_ids': sorted(v['scored_layout_ids']), 'example': v['example']})
    rs.sort(key=lambda r: -(r['hits_scored'] + r['hits_spare']))
    return {'groups': len(rs), 'distinct_models': len({r['model'] for r in rs}), 'distinct_models_with_scored_hits': len({r['model'] for r in rs if r['hits_scored']}),
            'hits_scored': sum(r['hits_scored'] for r in rs), 'hits_spare': sum(r['hits_spare'] for r in rs), 'rows': rs}


mass_lists = {'layout_level_slips_layout_mass_differs_from_metadata': group_list(lambda v, s: bool(s)),
              'hollow_objects_need_fill_dependent_band': group_list(lambda v, s: v == 'hollow_needs_fill_dependent_band' and not s),
              'too_heavy_for_class_band': group_list(lambda v, s: v == 'too_heavy_for_class' and not s),
              'light_for_solid_class_band': group_list(lambda v, s: v == 'light_for_solid_class' and not s)}

# ================================================================ 6. PR #84 alignment (machine-readable residual diff after v3)
align = None
if slice84:
    def fv(f):
        return [f.get('value'), f.get('range'), f.get('status')] if f else None
    diffs = []
    for aid, a in slice84['assets'].items():
        m = by.get(aid)
        if not m:
            continue
        ours = [m['material_class'], m['confidence'], sorted(m['alternative_material_classes'])]
        theirs = [a.get('material_class'), a.get('confidence'), sorted(a.get('alternative_material_classes') or [])]
        if ours != theirs:
            diffs.append({'scope': 'asset', 'key': aid, 'field': 'class/confidence/alternatives', 'ours_v3': ours, 'pr84': theirs})
    for c, v in slice84['classes'].items():
        for fld in ('density_kg_m3', 'restitution'):
            if fld in v and c in class_vals and fld in class_vals[c]:
                o_, t_ = class_vals[c][fld], v[fld]
                if [o_['value'], o_['range']] != [t_['value'], t_['range']]:
                    diffs.append({'scope': 'class', 'key': c, 'field': fld, 'ours_v3': fv(o_), 'pr84': fv(t_)})
    for k_, p in slice84['pairs'].items():
        e = pairs.get(canon(p['a'], p['b']))
        for fld in ('mu_static', 'mu_kinetic'):
            if e is None:
                diffs.append({'scope': 'pair', 'key': k_, 'field': fld, 'ours_v3': None, 'pr84': fv(p[fld])})
            elif [e[fld]['value'], e[fld]['range'], e[fld]['status']] != [p[fld]['value'], p[fld]['range'], p[fld]['status']]:
                diffs.append({'scope': 'pair', 'key': k_, 'field': fld, 'ours_v3': fv(e[fld]), 'pr84': fv(p[fld])})
    align = {'pr84_commit': '753d7d7 (v4)', 'remaining_differences': diffs, 'n_remaining': len(diffs)}

# v3.1 (#88 item 7): Geometry/camera_stand/00000 in put_bottles_into_dustbin: which class pairs can generate contacts
STAND, STAND_TASK = 'Geometry/camera_stand/00000', 'put_bottles_into_dustbin'
stand_partners, stand_alt_partners = collections.Counter(), collections.Counter()
stand_layouts = 0
for t, seed, lid, scored, inst in layouts:
    if t == STAND_TASK and scored and any(mid == STAND for _, _, _, mid in inst):
        stand_layouts += 1
        for sec, cat, o, mid in inst:
            if sec not in ('Garment', 'Fluid', 'Geometry') and mid in cls_of and cls_of[mid] != 'liquid':
                stand_partners[cls_of[mid]] += 1
                for a_ in by[mid]['alternative_material_classes']:
                    stand_alt_partners[a_] += 1
stand_cls = cls_of[STAND]
alt_only = set(stand_alt_partners) - set(stand_partners) - {'liquid', 'mixed_unclear'}
stand_pairs = sorted(canon(stand_cls, x) for x in set(stand_partners) | {'pad_hard', 'pad_elastomer'})
camera_stand = {
    'model': STAND, 'class': stand_cls, 'task': STAND_TASK, 'scored_layouts_with_it': stand_layouts,
    'rule': 'keep it in the co-occurrence counts and bind its class material in #62\'s static path; the stand is a static Geometry collider',
    'non_static_pairs': [f'{a}__{b}' for a, b in stand_pairs],
    'non_static_pairs_if_an_alternative_class_is_right': [f'{a}__{b}' for a, b in sorted(canon(stand_cls, x) for x in alt_only)],
    'non_static_pairs_by_partner': {'dynamic_object_classes': sorted(stand_partners), 'finger_pads': ['pad_hard', 'pad_elastomer']},
    'never_in_contact': [f'{a}__{b}' for a, b in (canon(stand_cls, x) for x in ('table_laminate', 'table_wood', 'ground'))],
    'never_in_contact_why': 'static Geometry against static Surface: PhysX generates no contact between two static colliders. In this task the stand is the only source of its class '
                            'in the static set, so these three pairs never act here; the same class pairs can act in other tasks through dynamic objects of that class.'}

# ================================================================ 7. write
WRITE = lambda name, obj: json.dump(obj, open(os.path.join(OUT, name), 'w'), indent=1, default=lambda x: sorted(x) if isinstance(x, set) else str(x))
conf_counts = lambda ms: dict(sorted(collections.Counter(m['confidence'] for m in ms).items(), key=lambda kv: -RANK[kv[0]]))
classes_out = {
    'schema': 'v3', 'status_vocabulary': ['sourced', 'estimated', 'unsourced'],
    'class_evidence_vocabulary': {
        'caption_material_specific': 'caption names a specific material of the class (e.g. PET, stainless steel, glazed ceramic)',
        'caption_material_generic': 'caption names only the material family (plastic, metal, wood)',
        'caption_material_conflict': 'caption names a material of another class; class kept on other evidence (see evidence)',
        'caption_identity_only': 'caption or description identifies the object but names no material',
        'caption_mismatch': 'caption describes a different object than the category; not used as evidence',
        'no_caption': 'no description.json at HF (404); class from category, bbox and mass only',
        'caption_names_product': 'caption names a branded product whose packaging material is known',
        'real_robot_frames': 'class from real-robot frames of the task', 'physics_type_fluid': 'metadata physics.type is fluid'},
    'object_classes': {c: {'description': cls_desc[c], 'n_models': sum(1 for m in models if m['material_class'] == c),
                           'confidence_counts': conf_counts([m for m in models if m['material_class'] == c])} for c in OBJ},
    'surface_classes': {c: dict(SURF[c]) for c in SURF},
    'scenarios': {'default': {'finger_pad': 'pad_hard', 'table': 'table_laminate'}, 'sensitivity': {'finger_pad': ['pad_elastomer'], 'table': ['table_wood']},
                  'note': 'scenario membership is its own field and never mixed into evidence status'},
    'models': models,
}
WRITE('classes.json', classes_out)
values = {'schema': 'v3', 'status_vocabulary': ['sourced', 'estimated', 'unsourced'],
          'field_status': 'value_status: evidence for the point value; range_status: evidence for the band; status: the weaker of the two (what a consumer of the band may rely on).',
          'scenarios': classes_out['scenarios'],
          'classes': {**{c: class_vals.get(c, {}) for c in OBJ}, **{c: dict(class_vals.get(c, {}), scenario_membership=SURF[c]['scenario_membership']) for c in SURF}},
          'restitution_schema': {
              'rule': 'unary restitution per class (the value and range of each class\'s `restitution` field), with optional per-asset and per-fill overrides: `fill_overrides.full` on a class '
                      'for a liquid-filled container (empty = the class value; #85). No runtime pair table: PhysX has no pair restitution, it combines the two unary values with the '
                      'collider\'s restitution_combine_mode (average here).',
              'pair_bands': 'pair bands (PR #84) are load-time check assertions only where the impact speed exceeds the 0.5 m/s bounce threshold (env/environment/base_env.py:27); '
                            'unary overrides reproduce all 20 of #84\'s pair bands (Wenhao\'s side review of #73).'},
          'camera_stand': camera_stand,
          'unary_values_default_scenario': {'note': 'Feasibility certificate, not a recommendation: the stage-3 max-margin unary map for the default scenario (pad_hard, table_laminate, '
                                                    'average combine everywhere including a ground set to average). u_k <= u_s holds for every class. See feasibility.json for all cells.',
                                            'static': default_cell['unary_static'], 'kinetic': default_cell['unary_kinetic'],
                                            'static_margin_mu': default_cell['static_margin_mu'], 'kinetic_margin_mu': default_cell['kinetic_margin_mu']},
          'cooccurrence_definition': {
              'episodes_scored_3seeds': f'exact count over all {integrity["scored_layouts"]} scored layouts (54 configs x seeds 0, 1, 2; population.json scored_layout_ids) at HF {HF_REV[:8]}: '
                                        'a layout counts once for a class pair if two distinct non-particle instances carry the two classes; every present class meets every surface class. '
                                        'Clutter vs listed model resolved by inst.type == "cluttered" as env/scene_manager/layout_manager.py:164-167 does.',
              'episodes_spare_3seeds': 'the same count over the spare (unscored) layouts',
              'excluded': 'liquid (particles), garments and fluids (particle contacts), pad-table, pad-ground, finger-finger and robot-link contacts'},
          'pairs': [pairs[k] for k in sorted(pairs)]}
WRITE('class_values.json', values)
WRITE('feasibility.json', feas)
lm = {'schema': 'v3',
      'method': 'Every instance of every layout file (54 configs x 3 seeds, scored and spare). Model id resolved by inst.type == "cluttered" (layout_manager.py:164-167). '
                'Declared layout mass vs class density band (class_values.json range) x bbox volume (metadata aligned bbox x layout scale). too_heavy = mass/bbox volume above the band top '
                '(a solid bbox cannot hold it). hollow = model flagged hollow_contents_dependent. light_for_solid = below half the band bottom. slip = layout mass differs from '
                'metadata.json mass by >2x or <0.5x. Groups are keyed by (task, model, verdict, layout mass, slip); hits are instance counts, split scored/spare.',
      'coverage': {'layout_files': integrity['files'], 'scored_layouts': integrity['scored_layouts'], 'instances_checked': checked, 'instances_in_scored_layouts': inst_scored,
                   'verdict_counts': dict(verdict_counts), 'integrity': integrity},
      'bottle_22kg': {'layouts_with_mass_22_scored': len(w22['scored']), 'layouts_with_mass_22_spare': len(w22['spare']), 'bottle_slots_all_layouts': bottle_slots},
      'friction_values_over_3_in_layouts': [{'task': k[0], 'model': k[1], 'field': k[2], 'value': k[3], 'instances': v} for k, v in sorted(fric_slips.items(), key=lambda kv: -kv[1])[:40]],
      **mass_lists}
WRITE('layout_mass_checks.json', lm)
if align:
    WRITE('alignment_84.json', align)

# ---- markdown: classes.md and citation_check.md, generated so their counts match the data
fields = []   # (scope, key, field, dict)
for c, x in values['classes'].items():
    for k, f in x.items():
        if isinstance(f, dict) and 'value_status' in f:
            fields.append(('class', c, k, f))
for p in values['pairs']:
    for fld in ('mu_static', 'mu_kinetic'):
        fields.append(('pair', p['key'], fld, p[fld]))
cnt = lambda key: collections.Counter(f[key] for *_, f in fields)
L = []
L.append('# Citation status of the #63 class table (v3)\n')
L.append(f'Generated by `audit/tools/params_classes_build.py` from `class_values.json`. {len(fields)} value fields: {sum(1 for s, *_ in fields if s == "class")} class fields '
         f'(density, restitution) and {sum(1 for s, *_ in fields if s == "pair")} pair fields ({len(values["pairs"])} pairs x static, kinetic). '
         'Each field has `value_status` (evidence for the point) and `range_status` (evidence for the band); `status` is the weaker of the two.\n')
L.append('| status | value_status | range_status | status |\n|---|---|---|---|')
for s in ('sourced', 'estimated', 'unsourced'):
    L.append(f'| {s} | {cnt("value_status")[s]} | {cnt("range_status")[s]} | {cnt("status")[s]} |')
L.append('\n## Fields whose point value is sourced\n')
L.append('| scope | key | field | value | range | range_status | sources (url: quote) |\n|---|---|---|---|---|---|---|')
for s, k, fld, f in fields:
    if f['value_status'] == 'sourced':
        src = '; '.join(f"{x['url'].split('//')[-1].split('/')[0]}: \"{x['quote'][:70]}\"" for x in f.get('sources', []) if (x.get('check') or {}).get('verdict', 'confirmed').startswith('confirmed'))
        L.append(f"| {s} | `{k}` | {fld} | {f['value']} | {f['range']} | {f['range_status']} | {src} |")
L.append('\n## Downgraded in v3 (sourced to estimated, review item 2)\n')
by_note = collections.defaultdict(list)
for d in downgraded:
    k, fld = d.split(':')
    by_note[pairs[tuple(k.split('__'))][fld]['v3_change'][0]].append(f'`{k}` {fld}')
for note, keys in by_note.items():
    L.append(f'- {", ".join(keys)}: {note}')
L.append('\n## Method and history\n')
L.append('- Step 3 (commit 0daca3c) checked 224 values: every URL fetched, quotes and numbers matched, condition mismatches flagged; its narrative is `git show 0daca3c:audit/out/63/citation_check.md`. '
         'v2 (52e73b1) kept those verdicts and added 146 pairs by rule (estimated or unsourced).')
L.append('- v3 splits each sourced field: `value_status` stays sourced when a confirmed source number supports the point; `range_status` is sourced only when the band lies inside '
         'the confirmed span of fetched numbers for the class members (1% tolerance), else estimated. Pairs whose sources measure other surface conditions (bare wood for '
         'finished wood, bare Al for coated Al, polystyrene for ABS) are estimated for both.')
L.append(f'- Re-fetched for v3 with curl on 2026-11-11 (raw HTML table cells): Engineering ToolBox friction, Roymech friction, Engineering ToolBox density of solids and of metals. '
         'New quotes carry "fetched 2026-11-11" in their conditions.')
open(os.path.join(OUT, 'citation_check.md'), 'w').write('\n'.join(L) + '\n')

C = []
cc_all = conf_counts(models)
C.append('# RoboDojo material classes (v3)\n')
C.append(f'{len(models)} distinct models of the frozen population (54 configs, HF {HF_REV[:8]}). Class and confidence rest on the HF captions (`description.json`), category, '
         'bounding box and declared mass; an earlier agent\'s `reason` is never counted as evidence. Values per class are in `class_values.json`; per-model detail in `classes.json`. '
         f'Generated by `audit/tools/params_classes_build.py`.\n')
C.append(f'Confidence overall: ' + ', '.join(f'{k} {v}' for k, v in cc_all.items()) + '.\n')
C.append('## Classes\n')
C.append('| class | models | high | medium | low | description |\n|---|---|---|---|---|---|')
for c in sorted(OBJ, key=lambda c: -classes_out['object_classes'][c]['n_models']):
    oc = classes_out['object_classes'][c]; cc = oc['confidence_counts']
    C.append(f"| `{c}` | {oc['n_models']} | {cc.get('high', 0)} | {cc.get('medium', 0)} | {cc.get('low', 0)} | {cls_desc[c]} |")
C.append('\nSurface classes (not models): ' + '; '.join(f'`{c}` ({SURF[c]["scenario_membership"]}): {SURF[c]["description"]}' for c in SURF) + '\n')
C.append('## Evidence behind each label\n')
ev_counts = collections.Counter((m['class_evidence'], m['confidence']) for m in models)
C.append('| class_evidence | models | high | medium | low |\n|---|---|---|---|---|')
for ev in classes_out['class_evidence_vocabulary']:
    n = sum(v for (e, _), v in ev_counts.items() if e == ev)
    if n:
        C.append(f"| `{ev}` | {n} | {ev_counts[(ev, 'high')]} | {ev_counts[(ev, 'medium')]} | {ev_counts[(ev, 'low')]} |")
nohigh = sum(1 for m in models if m['class_evidence'] in NO_MATERIAL_EVIDENCE and m['category'] in MULTI and m['confidence'] == 'high')
C.append(f'\nRule (v3): a model without caption material evidence (no caption, caption mismatch, identity only) in one of {len(MULTI)} multi-material categories is at most medium, with the '
         f'category\'s other common classes as alternatives, or low where no class dominates (chessman, dog_toy, mousemat, number_card, photo). High models left in that set: {nohigh}. '
         'In the same categories, a caption listing other contact materials (ear pads, soles, grips) caps the model at medium with those classes as alternatives. '
         'Anywhere, a caption whose first material names another class (`caption_material_conflict`) caps the model at medium with the caption class as alternative: '
         'step 2 kept those classes on bbox, mass or category evidence.\n')
C.append('## Low-confidence models\n')
low = collections.defaultdict(collections.Counter)
for m in models:
    if m['confidence'] == 'low':
        low[m['material_class']][f"{m['type']}/{m['category']}"] += 1
C.append(f"{sum(1 for m in models if m['confidence'] == 'low')} models, grouped by class and type/category (count).\n")
for c in sorted(low, key=lambda c: -sum(low[c].values())):
    C.append(f'- {c}: ' + '; '.join(f'{k} ({v})' for k, v in sorted(low[c].items())))
C.append('\n## Flags\n')
fl = collections.Counter(f for m in models for f in m['flags'])
for f, n in fl.most_common():
    grp = collections.Counter(f"{m['type']}/{m['category']}" for m in models if f in m['flags'])
    C.append(f'- **{f}** ({n}): ' + ', '.join(f'{k} ({v})' for k, v in grp.most_common(8)) + (' and more' if len(grp) > 8 else ''))
C.append('\n## Notes and cautions\n')
C.append('- `Rigid/cup/00001` (cover_blocks only) is `polyolefin_plastic` from the real-robot frames; its synthetic caption says glazed ceramic (flag `real_vs_sim_material_conflict`).')
C.append('- `polyolefin_plastic` is one class; `polymer_subtype` (' + ', '.join(f'{k} {v}' for k, v in sorted(sub_counts.items())) + ') records PP/PE where a caption, the category or the real frames state it.')
C.append('- `abs_toy_plastic` and `abs_device_housing` are hard-thermoplastic families (ABS/PS/PVC/PC), not one polymer; only ' +
         f"{sum(v for (e, _), v in fam_evidence.items() if e == 'caption_material_specific')} of their models have a caption naming a family polymer.")
C.append('- Contact material is the grasped surface: handles of tools (pliers, scissors, whisk, peeler) and pads of headphones carry the alternatives listed per model.')
C.append('- Garment/Top_Long is cloth particles; Fluid/wuliangye is the liquid, Rigid/wuliangye the bottle; Dynamic/conveyor is a 4 m belt driven kinematically.')
C.append('- Geometry fixtures are classed by their real analogue; their declared mass is often 0 and unused (static colliders).')
open(os.path.join(OUT, 'classes.md'), 'w').write('\n'.join(C) + '\n')

summary = {
    'models': len(models), 'confidence_step2': conf_counts(old['models']), 'confidence_v3': cc_all,
    'models_changed_v2_rules': len({r['id'] for r in log if r['version'] == 'v2'}), 'models_changed_v3_rules': len({r['id'] for r in log if r['version'] == 'v3'}),
    'v3_by_rule': dict(collections.Counter(r['rule'] for r in log if r['version'] == 'v3')),
    'v3_confidence_lowered': len({r['id'] for r in log if r['version'] == 'v3' and RANK[r['confidence_to']] < RANK[r['confidence_from']]}),
    'v3_class_changed': [r['id'] for r in log if r.get('class_from')],
    'class_counts': {c: classes_out['object_classes'][c]['n_models'] for c in OBJ},
    'class_evidence': {f'{e}:{c}': n for (e, c), n in sorted(ev_counts.items())},
    'abs_family_evidence': {f'{e}:{c}': n for (e, c), n in sorted(fam_evidence.items())},
    'reasons_rewritten_stale': stale, 'polymer_subtype': dict(sub_counts),
    'pairs_input': n_input_pairs, 'pairs_after_dedupe': len(pairs) - len(new_pairs), 'dups': [f'{a}__{b_}' for a, b_ in dups], 'new_pairs_by_rule': len(new_pairs),
    'pairs_total': len(pairs), 'pairs_cooccurring_scored': sum(1 for k in pairs if ep_scored.get(k)), 'pairs_spare_only': [pairs[k]['key'] for k in pairs if not ep_scored.get(k) and ep_spare.get(k)],
    'pairs_never': [pairs[k]['key'] for k in pairs if not ep_scored.get(k) and not ep_spare.get(k)],
    'pairs_by_kind': dict(collections.Counter(e['kind'] for e in pairs.values())),
    'fields': len(fields), 'value_status': dict(cnt('value_status')), 'range_status': dict(cnt('range_status')), 'status': dict(cnt('status')),
    'downgraded_v3': downgraded,
    'feasibility': {f'{s}|{cell}': [r['feasible'], r['static_margin_mu'], r['kinetic_margin_mu'], r['joint_min_margin_mu']] for s, x in feas['scenarios'].items() for cell, r in x.items() if cell != 'membership'},
    'override_00022': {f: ov[f]['allowed_unary_interval_for_00022'] for f in ('mu_static', 'mu_kinetic')}, 'pr84_unary_extension': ext84 and {k: ext84[k] for k in ('joint_min_margin_mu', 'extendable')},
    'layout_integrity': {k: integrity[k] for k in ('files', 'scored_layouts', 'layouts_per_seed', 'blob_count_mismatch_vs_population', 'scored_instances_not_in_population_scored_assets', 'unknown_models')},
    'mass': {k: {kk: v[kk] for kk in ('groups', 'distinct_models', 'distinct_models_with_scored_hits', 'hits_scored', 'hits_spare')} for k, v in mass_lists.items()},
    'mass_instances_checked': checked, 'bottle_22kg': lm['bottle_22kg'],
    'alignment_84_remaining': align and align['n_remaining'],
}
WRITE('build_summary.json', {'summary': summary, 'model_log': log})
print(json.dumps(summary, indent=1, default=str))
