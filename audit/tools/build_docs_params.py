#!/usr/bin/env python3
"""Deterministically generate docs/params/{classes,pairs,assets,robots}.json for the tasks in scope (#62 PR 2, #167, #59).

Pure Python standard library (python3 -I compatible across Python 3.9+ and 3.14+).
Contains zero hand-edited physics numbers: every value is read or derived by exact arithmetic from:
  1. audit/out/31/population.json (#31/#72 frozen task population: robot arms, scene fixtures, and task objects)
  2. audit/out/63/classes.json (#73/#91 model material classes and metadata bboxes)
  3. audit/out/63/class_values.json (#73/#91 class densities, restitutions, pair bands, and co-occurrence counts)
  4. audit/out/63/feasibility.json (#73/#91 00022 co-occurrence check)
  5. the pinned task slices in PINNED_TASK_SLICES, each SHA-256 verified:
     git:753d7d7:audit/out/63/bottles/put_bottles_into_dustbin.json (#84 v4 bottle slice),
     audit/out/63/cover_blocks/cover_blocks.json and audit/out/63/swap_T/swap_T.json (#63 lane 2, #167 decisions)
  6. audit/out/63/usd_geometry_facts.json (USD outer-hull solid/shell moments, camera_stand colliders, X5 links)
     and audit/out/63/usd_geometry_facts_cb_swapT.json (solid and box colliders of cubes, cup and Ts; cross-checked)
  7. audit/tools/reviewed_opt_in_profiles.json (committed reviewed per-key unsourced opt-in lists)
  8. audit/out/d3/gripper_sources.json (D3 gripper constants: arx5-sdk gains, transmission, effort decision)
  9. audit/out/d3/arm_limits_sources.json (#11 ARX X5 arm ranges, velocity and torque limits from arx5-sdk)
 10. the pinned contact censuses in PINNED_CONTACT_CENSUSES, each SHA-256 verified:
     audit/out/g2/contact_census_{put_bottles_into_dustbin,cover_blocks,swap_T}.json (#59 option 3, #167 decision 3)
One global unary friction map is fitted jointly by presence weighting over object-loaded pairs (#59 option 3, #167
decision 3): weight 1 iff a non-robot-self pair carries normal load in at least one scripted solution's corrected
contact census, band-only otherwise.
"""
from __future__ import annotations

import argparse
from decimal import Decimal, ROUND_HALF_EVEN
from fractions import Fraction
import gzip
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
from typing import Any

HF_REVISION = "1127bda94f172d757386945af3eb3a6754d1a4e6"
PINNED_BOTTLE_SLICE_SPEC = "git:753d7d7:audit/out/63/bottles/put_bottles_into_dustbin.json"
PINNED_BOTTLE_SLICE_SHA256 = "a214ec317b108c4659f0148b79e654206522d35d5004dc995450f49f7a8c0a5b"

# Tasks in scope and their pinned slices: (task_name, spec, SHA-256). A git spec reads the blob at a merged commit;
# a path spec reads the working tree. Both are verified against the SHA-256 here.
PINNED_TASK_SLICES: tuple[tuple[str, str, str], ...] = (
    ("put_bottles_into_dustbin", PINNED_BOTTLE_SLICE_SPEC, PINNED_BOTTLE_SLICE_SHA256),
    ("cover_blocks", "audit/out/63/cover_blocks/cover_blocks.json", "3a76480eb5169a4193c7f8103b7050ccbfe607202f72cad131dd811a2d72cc6e"),
    ("swap_T", "audit/out/63/swap_T/swap_T.json", "cc20fa009bf6189b02045a7c1240bb20c4292b77f7ab3c686d3dbd5d0dd5410c"),
)
# Committed scripted-solution contact censuses (corrected arm) used for presence weighting (#59 option 3, #167 decision 3).
PINNED_CONTACT_CENSUSES: tuple[tuple[str, str, str], ...] = (
    (
        "put_bottles_into_dustbin",
        "audit/out/g2/contact_census_put_bottles_into_dustbin.json",
        "0f6a783b9802e20ff0dd41a9b7f24033cac4ca6c2922dbc0d5686d64285476c8",
    ),
    (
        "cover_blocks",
        "audit/out/g2/contact_census_cover_blocks.json",
        "3fb4e2c4ddd3e14fc1a3550c7feda49208b45d06db9f79a7a876aa918ffb7ca0",
    ),
    (
        "swap_T",
        "audit/out/g2/contact_census_swap_T.json",
        "2e96fce924177cc88a41b7f366023e0adb2860d21782d25b47cb9212458250c2",
    ),
)
USD_FACTS_CB_SWAPT = ("audit/out/63/usd_geometry_facts_cb_swapT.json", "5d39fbe223764d275950df833eba8eb2fc81b25f580ea946137fc012c1b5ce68")
USD_FACTS_POP_T1 = ("audit/out/63/usd_geometry_facts_population_t1.json", "e7fa9ac74a30e5dbe827efd4d66f8ca2e69f905b33bed8620d66df93b44c84ae")
T1_POPULATION_ROWS = ("audit/out/63/population/t1_rows.json", "890716747d95925a441bb80e5e2e1dc1a33d9d898f5d6b4e736d58cd73cf75ec")
BASIS_DRAFTS_MD = ("audit/out/63/population/basis_drafts.md", "872e503fd2fe95b08f9506bdc10ff8d807d645a071992830bf906ee816be1846")
USD_FACTS_POP_T2 = ("audit/out/63/usd_geometry_facts_population_t2.json.gz", "adeaba26f7917ea2021ffc1c3398e64686a3a1af99551670432b5d8532e07fba")
T2_POPULATION_ROWS = ("audit/out/63/population/t2_rows.json.gz", "044ac940c186f06e692660f9b0eeb1a010d3fe0a3fa98ba52af6f2352ba40f23")
BASIS_DRAFTS_T2_MD = ("audit/out/63/population/basis_drafts_t2.md", "144b70b41690b7ec2462819b5f02623ad2615f9b11dbed492b5e51d3c676b34a")
# Classes assigned to X5 non-pad links in robots.json (camera shell and structural links).
ROBOT_LINK_CLASSES: tuple[str, ...] = ("abs_device_housing", "aluminium")
# Known-cost threshold: a counted pair whose realised value differs from nominal by at least this much is listed.
KNOWN_COST_ABS_DELTA = Decimal("0.05")
# Slice-only asset keys the loader rejects; they stay in the slices.
SLICE_ONLY_ASSET_KEYS: frozenset[str] = frozenset(
    {"class_conflict", "class_decision", "label_in_task", "proposed_alternative_material_classes", "named_sensitivity_classes",
     "declared_metadata", "scenario_role", "mass", "asset_path"}
)
ALLOWED_BAND_KEYS: frozenset[str] = frozenset(
    {
        "value",
        "range",
        "unit",
        "status",
        "value_status",
        "range_status",
        "confirmed_source_span",
        "sources",
        "conditions",
        "checked_by",
        "basis",
        "derivation",
        "reason_v2",
        "reasoning",
        "v3_change",
        "why",
        "fill_overrides",
        "value_is_for_fill",
        "impact_speed_above_0.5_m_s",
        "notes",
    }
)


def _repo_root(here: str) -> str:
    cand = os.path.abspath(os.path.join(here, "../.."))
    if os.path.isfile(os.path.join(cand, "AGENTS.md")):
        return cand
    return subprocess.run(
        ["git", "-C", here, "rev-parse", "--show-toplevel"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()


def _load_json_with_sha(
    spec: str,
    repo_dir: str | None,
    *,
    expected_sha256: str | None = None,
) -> tuple[Any, str]:
    """Load JSON from a file path or ``git:<rev>:<path>``, preferring pinned git blobs over working-tree files."""
    if spec.startswith("git:"):
        rev, rel = spec[4:].split(":", 1)
        pin_sha = expected_sha256 or (PINNED_BOTTLE_SLICE_SHA256 if spec == PINNED_BOTTLE_SLICE_SPEC else None)
        has_git_repo = (
            shutil.which("git") is not None
            and repo_dir is not None
            and os.path.exists(os.path.join(repo_dir, ".git"))
        )
        if has_git_repo:
            assert repo_dir is not None
            raw_bytes = subprocess.run(
                ["git", "-C", repo_dir, "show", f"{rev}:{rel}"],
                capture_output=True,
                check=True,
            ).stdout
            sha = hashlib.sha256(raw_bytes).hexdigest()
            if pin_sha is not None and sha != pin_sha:
                raise RuntimeError(f"Pinned blob {spec!r} sha256={sha} != expected {pin_sha}")
            return json.loads(raw_bytes.decode("utf-8")), sha

        # Fallback only when git is unavailable (e.g. inside container), strictly verified by SHA-256.
        if repo_dir is not None and pin_sha is not None:
            wt_path = os.path.join(repo_dir, rel)
            if os.path.isfile(wt_path):
                with open(wt_path, "rb") as bf:
                    raw_bytes = bf.read()
                sha = hashlib.sha256(raw_bytes).hexdigest()
                if sha != pin_sha:
                    raise RuntimeError(
                        f"Working-tree fallback {wt_path!r} sha256={sha} mismatches pinned {spec!r} sha256={pin_sha}"
                    )
                return json.loads(raw_bytes.decode("utf-8")), sha
        raise RuntimeError(
            f"Cannot resolve {spec!r}: git repository/executable unavailable and no SHA-256-verified fallback found"
        )

    with open(spec, "rb") as bf:
        raw_bytes = bf.read()
    sha = hashlib.sha256(raw_bytes).hexdigest()
    if expected_sha256 is not None and sha != expected_sha256:
        raise RuntimeError(f"Input file {spec!r} sha256={sha} != expected {expected_sha256}")
    payload_bytes = gzip.decompress(raw_bytes) if spec.endswith(".gz") else raw_bytes
    return json.loads(payload_bytes.decode("utf-8")), sha


def _load_json(spec: str, repo_dir: str | None) -> Any:
    return _load_json_with_sha(spec, repo_dir)[0]


def _load_text_with_sha(path: str, *, expected_sha256: str | None = None) -> tuple[str, str]:
    with open(path, "rb") as bf:
        raw_bytes = bf.read()
    sha = hashlib.sha256(raw_bytes).hexdigest()
    if expected_sha256 is not None and sha != expected_sha256:
        raise RuntimeError(f"Input file {path!r} sha256={sha} != expected {expected_sha256}")
    return raw_bytes.decode("utf-8"), sha


def _parse_basis_drafts_md(md_text: str, *, expected_count: int = 85) -> dict[str, str]:
    """Parse #173 physical-reasoning basis sentences from basis_drafts*.md."""
    out: dict[str, str] = {}
    hdr_re = re.compile(
        r"\*\*`((?:classes\.[a-z_]+\.(?:restitution|density_kg_m3))|(?:pairs\.[a-z_]+__[a-z_]+\.mu_(?:static|kinetic)))`"
    )
    bas_re = re.compile(r"^\s*-\s*\*\*Basis\s*\([^)]*\):\*\*\s*(.+?)\s*$")
    cur_key: str | None = None
    for line in md_text.splitlines():
        mh = hdr_re.search(line)
        if mh is not None:
            cur_key = mh.group(1)
            continue
        if cur_key is not None:
            mb = bas_re.match(line)
            if mb is not None:
                if cur_key in out:
                    raise RuntimeError(f"Duplicate basis draft key {cur_key!r}")
                out[cur_key] = mb.group(1)
                cur_key = None
    if len(out) != expected_count:
        raise RuntimeError(f"Expected {expected_count} basis draft entries, parsed {len(out)}")
    return out


def _entry_has_basis_text(entry: dict[str, Any]) -> bool:
    for f in ("basis", "derivation", "reasoning", "reason_v2", "why", "notes", "conditions", "v3_change"):
        if isinstance(entry.get(f), str) and entry[f].strip():
            return True
    for s in entry.get("sources") or ():
        if isinstance(s, dict) and (
            (isinstance(s.get("conditions"), str) and s["conditions"].strip())
            or (isinstance(s.get("quote"), str) and s["quote"].strip())
        ):
            return True
    return False


def verify_all_estimated_leaves_have_basis(docs_by_name: dict[str, dict[str, Any]]) -> None:
    """Fail generation if any leaf marked 'estimated' in classes/pairs/assets/robots lacks a per-value basis or source."""
    missing: list[str] = []

    def _walk(obj: Any, path: str, fname: str) -> None:
        if isinstance(obj, dict):
            st_vals = {obj.get(k) for k in ("status", "value_status", "range_status") if k in obj}
            if "estimated" in st_vals and not _entry_has_basis_text(obj):
                missing.append(f"{fname}:{path}")
            for k, v in obj.items():
                _walk(v, f"{path}.{k}" if path else str(k), fname)
        elif isinstance(obj, list):
            for idx, v in enumerate(obj):
                _walk(v, f"{path}[{idx}]", fname)

    for fname, doc in sorted(docs_by_name.items()):
        _walk(doc, "", fname)
    if missing:
        raise RuntimeError(
            f"{len(missing)} estimated parameter leaf/leaves lack a per-value basis or source quote: {missing[:10]}"
        )


def _d(x: Any) -> Decimal:
    return Decimal(str(x))


def _f(x: Any) -> Fraction:
    return Fraction(Decimal(str(x)))


def _q(val: Decimal, places: int) -> float:
    fmt = Decimal("1e-" + str(places))
    return float(val.quantize(fmt, rounding=ROUND_HALF_EVEN))


def _q_frac(val: Fraction, places: int = 6) -> float:
    scale = 10**places
    scaled = val * scale
    num, den = scaled.numerator, scaled.denominator
    quot, rem = divmod(abs(num), den)
    if 2 * rem > den or (2 * rem == den and quot % 2 == 1):
        quot += 1
    if num < 0:
        quot = -quot
    return float(Decimal(quot) * Decimal(f"1e-{places}"))


def _solve_exact_kkt_qp(
    h_mat: list[list[Fraction]],
    g_vec: list[Fraction],
    a_mat: list[list[Fraction]],
    b_vec: list[Fraction],
) -> tuple[list[Fraction], list[int]]:
    """Solve strictly convex QP ``min 0.5 x^T H x + g^T x s.t. A x <= b`` with exact KKT verification in ``Fraction``."""
    dim = len(g_vec)
    m = len(b_vec)
    h_flt = [[float(v) for v in row] for row in h_mat]
    g_flt = [float(v) for v in g_vec]
    a_flt = [[float(v) for v in row] for row in a_mat]
    b_flt = [float(v) for v in b_vec]

    def _float_solve(mat: list[list[float]], rhs: list[float]) -> list[float]:
        sz = len(rhs)
        aug = [row[:] + [rhs[i]] for i, row in enumerate(mat)]
        for col in range(sz):
            piv = max(range(col, sz), key=lambda r: abs(aug[r][col]))
            aug[col], aug[piv] = aug[piv], aug[col]
            div = aug[col][col]
            if abs(div) < 1e-15:
                raise RuntimeError("Singular Hessian in QP warm-start")
            for j in range(col, sz + 1):
                aug[col][j] /= div
            for r in range(sz):
                if r != col and aug[r][col] != 0.0:
                    fac = aug[r][col]
                    for j in range(col, sz + 1):
                        aug[r][j] -= fac * aug[col][j]
        return [aug[i][sz] for i in range(sz)]

    def _frac_solve(mat: list[list[Fraction]], rhs: list[Fraction]) -> list[Fraction]:
        sz = len(rhs)
        aug = [row[:] + [rhs[i]] for i, row in enumerate(mat)]
        for col in range(sz):
            piv = next((r for r in range(col, sz) if aug[r][col] != 0), None)
            if piv is None:
                raise RuntimeError("Singular KKT linear system in exact active-set QP")
            aug[col], aug[piv] = aug[piv], aug[col]
            div = aug[col][col]
            for j in range(col, sz + 1):
                aug[col][j] /= div
            for r in range(sz):
                if r != col and aug[r][col] != 0:
                    fac = aug[r][col]
                    for j in range(col, sz + 1):
                        aug[r][j] -= fac * aug[col][j]
        return [aug[i][sz] for i in range(sz)]

    # Deterministic Hildreth dual coordinate descent warm-start to locate active constraint indices.
    hinv_g = _float_solve(h_flt, g_flt)
    hinv_at = [_float_solve(h_flt, a_flt[row]) for row in range(m)]
    p_dual = [[sum(a_flt[i][k] * hinv_at[j][k] for k in range(dim)) for j in range(m)] for i in range(m)]
    q_dual = [b_flt[i] + sum(a_flt[i][k] * hinv_g[k] for k in range(dim)) for i in range(m)]
    lam = [0.0] * m
    for _ in range(30000):
        max_chg = 0.0
        for i in range(m):
            pii = p_dual[i][i]
            if pii <= 1e-15:
                continue
            grad_i = q_dual[i] + sum(p_dual[i][j] * lam[j] for j in range(m)) - pii * lam[i]
            new_l = max(0.0, -grad_i / pii)
            chg = abs(new_l - lam[i])
            if chg > max_chg:
                max_chg = chg
            lam[i] = new_l
        if max_chg < 1e-14:
            break
    active = [i for i in range(m) if lam[i] > 1e-9]

    # Exact primal-dual active-set polish + KKT certificate verification in Fraction arithmetic.
    for _ in range(100):
        k = len(active)
        kkt = [[Fraction(0)] * (dim + k) for _ in range(dim + k)]
        rhs = [Fraction(0)] * (dim + k)
        for r in range(dim):
            rhs[r] = -g_vec[r]
            for c in range(dim):
                kkt[r][c] = h_mat[r][c]
            for j, ai in enumerate(active):
                kkt[r][dim + j] = a_mat[ai][r]
                kkt[dim + j][r] = a_mat[ai][r]
        for j, ai in enumerate(active):
            rhs[dim + j] = b_vec[ai]
        sol = _frac_solve(kkt, rhs)
        x_ex = sol[:dim]
        lam_act = sol[dim:]

        if any(lv < 0 for lv in lam_act):
            worst_j = min(range(k), key=lambda j: lam_act[j])
            active.pop(worst_j)
            continue

        viols = [
            (sum(a_mat[i][c] * x_ex[c] for c in range(dim)) - b_vec[i], i)
            for i in range(m)
            if i not in active
        ]
        max_v, max_i = max(viols, default=(Fraction(0), -1))
        if max_v > 0:
            active.append(max_i)
            active.sort()
            continue

        lam_full = [Fraction(0)] * m
        for j, ai in enumerate(active):
            lam_full[ai] = lam_act[j]
        for r in range(dim):
            stat = (
                sum(h_mat[r][c] * x_ex[c] for c in range(dim))
                + g_vec[r]
                + sum(a_mat[i][r] * lam_full[i] for i in range(m))
            )
            if stat != 0:
                raise RuntimeError(f"KKT stationarity residual {stat} != 0 at variable {r}")
        for i in range(m):
            slack = sum(a_mat[i][c] * x_ex[c] for c in range(dim)) - b_vec[i]
            if slack > 0:
                raise RuntimeError(f"KKT primal feasibility violated ({slack} > 0) at constraint {i}")
            if lam_full[i] < 0:
                raise RuntimeError(f"KKT dual feasibility violated ({lam_full[i]} < 0) at constraint {i}")
            if lam_full[i] * slack != 0:
                raise RuntimeError(f"KKT complementary slackness violated at constraint {i}")
        return x_ex, active

    raise RuntimeError("Exact active-set QP solver exceeded iteration cap")


PAIR_FORMABILITY_RULE = (
    "A material pair (m_a, m_b) can form in a task under scenario (pad_class, table_class) "
    "iff an episode of that task contains two distinct engine collider bodies b_1 != b_2 carrying m_a and m_b "
    "(using asset_id when the asset has material_override, else its assigned material_class) such that "
    "at least one of b_1, b_2 is dynamic (Rigid body or robot Articulation link; static Geometry and Scene "
    "colliders have RigidBodyAPI disabled/absent and never form static-static contacts in PhysX). "
    "It can form a load-bearing contact iff such a body pair exists that is not finger on finger (both bodies "
    "gripper finger pads: an empty close, or the grippers of two arms meeting in a handover). Finger-on-finger "
    "contact never carries a sliding load these tasks depend on (#167, Bo's side's decision (b)). "
    "Any banded pair that cannot form a load-bearing contact in the task (an uncarried class unary, a "
    "no_self_contact self-pair, a static-static collider pair, or a finger-on-finger-only pair such as "
    "pad_hard__pad_hard) is enforced as a band constraint only and never enters the L2 objective. "
    "Across tasks (#59 option 3, #167 decisions 2, 3 and (b)): objective weights come from the committed "
    "scripted-solution contact censuses (see PAIR_WEIGHT_RULE): weight 1 iff the pair carries normal load on a "
    "non-robot-self body pair in at least one scripted solution, and band-only (weight 0) otherwise."
)

PAIR_WEIGHT_RULE = (
    "Presence weighting from the committed scripted-solution contact censuses (#59 option 3, #167 decisions 3 and (b)): "
    "a primary-scenario material pair receives weight 1 in the global Stage 1 L2 objective (counted_in_fit_objective = True) "
    "iff it can form in at least one task in scope and carries positive normal load (total_normal_impulse_Ns > 0 and "
    "actions_force_total > 0 on body pairs that are not robot-link x robot-link) in the committed corrected contact "
    "census of at least one task's scripted solution; robot-link x robot-link contacts (finger on finger such as "
    "link7|link8 and intra-arm/inter-arm link grazes such as r1.link1|r1.link3) are excluded per #167 decision (b). "
    "In Stage 2, each sensitivity-scenario pair (pad_elastomer, table_wood) inherits the presence weight (1 or 0) of its "
    "default-scenario counterpart (pad_hard, table_laminate) with Stage 1 classes held fixed. "
    "Every weight-0 pair (formable pairs with zero object-contact load in all censused scripts, robot-link x robot-link-only "
    "pairs, pairs from tasks without a scripted-solution census yet, and non-formable pairs) is enforced as an exact band "
    "constraint only (with 1-hop Fourier-Motzkin cross-band projection in Stages 1-2 and an exact feasible Stage 3 solve "
    "for zero-weight classes)."
)


def _is_census_robot_link(body_name: str) -> bool:
    """Return True iff a census body_pairs side names a robot articulation link (r<i>.<link> or robot<i>.<link>)."""
    prefix, sep, _ = body_name.partition(".")
    if not sep:
        return False
    return (prefix.startswith("robot") and prefix[5:].isdigit()) or (prefix.startswith("r") and prefix[1:].isdigit())


def _sens_counterpart_key(pk: str) -> str:
    """Map a sensitivity pair (pad_elastomer, table_wood) to its default-scenario counterpart key."""
    rep = {"pad_elastomer": "pad_hard", "table_wood": "table_laminate"}
    if pk.startswith("Rigid/"):
        aid, partner = pk.split("__", 1)
        return f"{aid}__{rep.get(partner, partner)}"
    a, b = pk.split("__", 1)
    ca, cb = rep.get(a, a), rep.get(b, b)
    return "__".join(sorted((ca, cb)))


def derive_census_presence_weights(
    contact_censuses: dict[str, dict[str, Any]],
    *,
    task_order: tuple[str, ...],
    arm: str = "corrected",
) -> dict[str, Any]:
    """Derive exact rational presence weights (w in {0, 1}) and per-pair census evidence from committed censuses."""
    pair_weights: dict[str, Fraction] = {}
    per_pair_tasks: dict[str, dict[str, Any]] = {}
    excluded_robot_self_only: dict[str, dict[str, Any]] = {}
    task_summaries: dict[str, dict[str, Any]] = {}
    n_censused = max(len(task_order), 1)
    mean_imp_share_acc: dict[str, Fraction] = {}

    for tname in task_order:
        cdoc = contact_censuses[tname]
        if cdoc.get("task") != tname:
            raise RuntimeError(f"Contact census task={cdoc.get('task')!r} != expected {tname!r}")
        adata = (cdoc.get("arms") or {}).get(arm)
        if not isinstance(adata, dict):
            raise RuntimeError(f"Contact census for {tname!r} missing arm {arm!r}")
        n_ep = int(adata["episodes_count"])
        cps = adata.get("class_pairs") or {}
        bps = adata.get("body_pairs") or {}
        tot_imp = sum((Decimal(str(cd["total_normal_impulse_Ns"])) for cd in cps.values()), Decimal(0))
        tot_act = sum((int(cd["actions_force_total"]) for cd in cps.values()), 0)

        robot_self_imp: dict[str, Decimal] = {}
        robot_self_act: dict[str, int] = {}
        robot_self_bps: dict[str, list[str]] = {}
        for bp_key, bd in sorted(bps.items()):
            b1, sep, b2 = bp_key.partition("|")
            if not sep or not (_is_census_robot_link(b1) and _is_census_robot_link(b2)):
                continue
            act_bp = int(bd.get("actions_force_total", 0))
            imp_bp = Decimal(str(bd.get("total_normal_impulse_Ns", 0)))
            if act_bp <= 0 and imp_bp <= 0:
                continue
            cp_bp = str(bd["class_pair"])
            robot_self_imp[cp_bp] = robot_self_imp.get(cp_bp, Decimal(0)) + imp_bp
            robot_self_act[cp_bp] = robot_self_act.get(cp_bp, 0) + act_bp
            robot_self_bps.setdefault(cp_bp, []).append(bp_key)

        loaded_obj_in_task: list[str] = []
        robot_self_only_in_task: list[str] = []
        for pk, cd in sorted(cps.items()):
            act_f = int(cd.get("actions_force_total", 0))
            imp_d = Decimal(str(cd.get("total_normal_impulse_Ns", 0)))
            if act_f <= 0 or imp_d <= 0:
                continue
            w_imp_t = (Fraction(imp_d) / Fraction(tot_imp)) if tot_imp > 0 else Fraction(0)
            w_act_t = Fraction(act_f, tot_act) if tot_act > 0 else Fraction(0)
            rs_imp = robot_self_imp.get(pk, Decimal(0))
            rs_bps = robot_self_bps.get(pk, [])
            is_finger_self = pk in ("pad_hard__pad_hard", "pad_elastomer__pad_elastomer")
            # Robot links have fixed material classes across all layouts, so robot-self impulse is exact to the
            # census's 1e-5 N*s rounding per body pair.
            tol = Decimal("0.0001") * max(len(rs_bps), 1)
            obj_imp = Decimal(0) if (is_finger_self or (rs_bps and imp_d - rs_imp <= tol)) else (imp_d - rs_imp)
            is_obj_loaded = obj_imp > 0

            mean_imp_share_acc[pk] = mean_imp_share_acc.get(pk, Fraction(0)) + (
                (Fraction(obj_imp) / Fraction(tot_imp)) / Fraction(n_censused) if (is_obj_loaded and tot_imp > 0) else Fraction(0)
            )
            ev_row: dict[str, Any] = {
                "episodes_with_force": int(cd["episodes_with_force"]),
                "episodes_total": n_ep,
                "actions_force_total": act_f,
                "normal_impulse_Ns": float(imp_d),
                "task_impulse_share": _q_frac(w_imp_t, 6),
                "task_force_action_share": _q_frac(w_act_t, 6),
                "object_contact_loaded": is_obj_loaded,
            }
            if rs_bps:
                ev_row["robot_link_self_body_pairs"] = rs_bps
                ev_row["robot_link_self_impulse_Ns"] = float(rs_imp)
                ev_row["non_robot_self_impulse_Ns"] = float(obj_imp)
            per_pair_tasks.setdefault(pk, {})[tname] = ev_row

            if is_obj_loaded:
                pair_weights[pk] = Fraction(1)
                loaded_obj_in_task.append(pk)
            else:
                robot_self_only_in_task.append(pk)
                excluded_robot_self_only.setdefault(pk, {})[tname] = ev_row

        task_summaries[tname] = {
            "policy": cdoc.get("policy"),
            "development_set": cdoc.get("development_set"),
            "arm": arm,
            "episodes_count": n_ep,
            "success_count": int(adata["success_count"]),
            "total_actions": int(adata["total_actions"]),
            "total_normal_impulse_Ns": float(tot_imp),
            "object_loaded_pairs": loaded_obj_in_task,
            "excluded_robot_self_only_pairs": robot_self_only_in_task,
        }

    # Filter out any pair that later turned out loaded in another task from excluded_robot_self_only
    for pk_loaded in list(pair_weights):
        excluded_robot_self_only.pop(pk_loaded, None)

    return {
        "rule": PAIR_WEIGHT_RULE,
        "arm": arm,
        "censused_tasks": list(task_order),
        "pair_weights": pair_weights,
        "per_pair_tasks": per_pair_tasks,
        "mean_3task_object_impulse_share": {k: _q_frac(v, 6) for k, v in sorted(mean_imp_share_acc.items()) if v > 0},
        "excluded_robot_self_only_pairs": excluded_robot_self_only,
        "task_summaries": task_summaries,
    }


def derive_task_formable_pairs(
    population_src: dict[str, Any],
    *,
    task_name: str,
    out_assets: dict[str, dict[str, Any]],
    out_pairs: dict[str, dict[str, Any]],
    x5_links: list[dict[str, Any]],
    non_pad_map: dict[str, dict[str, Any]],
    default_pad_cls: str = "pad_hard",
    default_table_cls: str = "table_laminate",
    sens_pad_cls: str = "pad_elastomer",
    sens_table_cls: str = "table_wood",
    slot_models: list[str] | None = None,
) -> dict[str, Any]:
    """Derive formable vs band-only pairs from population.json task objects, robot links, and scene fixtures.

    ``slot_models`` lists the model of every Rigid slot of an episode when each slot holds one fixed model (from the
    slice's layout census, e.g. cover_blocks: 3 cubes and 3 cups). Without it, all non-override Rigid models must share
    one class and fill the task's Rigid slots (the bottle task).
    """
    tcfg = next((c for c in population_src["configs"] if c["task_name"] == task_name), None)
    if tcfg is None:
        raise RuntimeError(f"Task {task_name!r} not found in population.json")

    def _is_finger_pad(body_name: str) -> bool:
        # Robot link bodies are named robot<i>.<link>; links without a non-pad class carry the pad class.
        return body_name.startswith("robot") and body_name.split(".", 1)[1] not in non_pad_map

    def _carrier(aid: str, *, table_override_cls: str | None = None) -> tuple[str, str]:
        adict = out_assets[aid]
        if "material_override" in adict:
            return ("override", aid)
        if table_override_cls is not None:
            return ("class", table_override_cls)
        return ("class", str(adict["material_class"]))

    obj_groups = tcfg["assets"]["objects"]
    inst_counts_l0 = tcfg["assets"]["layout0_seed0_instance_counts"]
    geom_groups = [og for og in obj_groups if og["kind"] == "Geometry"]
    rigid_groups = [og for og in obj_groups if og["kind"] == "Rigid"]
    other_groups = [og for og in obj_groups if og["kind"] not in ("Geometry", "Rigid")]
    if other_groups:
        raise RuntimeError(f"Unsupported object group kind(s) in {task_name}: {[og['kind'] for og in other_groups]}")

    geom_models = [f"{og['kind']}/{og['category']}/{idx:05d}" for og in geom_groups for idx in og["indices"]]
    if int(inst_counts_l0.get("Geometry", 0)) != len(geom_models):
        raise RuntimeError(
            f"Geometry instance count {inst_counts_l0.get('Geometry')} != distinct Geometry models {geom_models}"
        )
    for g_aid in geom_models:
        g_cls = str(out_assets[g_aid]["material_class"])
        self_pk = f"{g_cls}__{g_cls}"
        if self_pk in out_pairs:
            co_ep = ((out_pairs[self_pk].get("cooccurrence") or {}).get("episodes_scored_by_config") or {}).get(
                task_name, 0
            )
            # Single-instance check when no other object/link carries g_cls is reflected in co_ep == 0.
            _ = co_ep

    rigid_models = [f"{og['kind']}/{og['category']}/{idx:05d}" for og in rigid_groups for idx in og["indices"]]
    n_rigid_slots = int(inst_counts_l0.get("Rigid", 0))
    if n_rigid_slots <= 0 or (slot_models is None and len(rigid_models) < n_rigid_slots):
        raise RuntimeError(f"Invalid Rigid slot count {n_rigid_slots} for models {rigid_models}")

    override_rigid = [aid for aid in rigid_models if "material_override" in out_assets[aid]]
    class_rigid = [aid for aid in rigid_models if "material_override" not in out_assets[aid]]
    rigid_realizations: list[list[tuple[str, tuple[str, str], bool]]]
    if slot_models is not None:
        if len(slot_models) != n_rigid_slots or set(slot_models) != set(rigid_models):
            raise RuntimeError(
                f"{task_name}: slot models {slot_models} do not match population Rigid models {rigid_models} "
                f"({n_rigid_slots} slots)"
            )
        if override_rigid:
            raise RuntimeError(f"{task_name}: fixed-slot path does not handle material_override assets {override_rigid}")
        rigid_realizations = [
            [
                (f"Rigid/slot_{s}({aid})", ("class", str(out_assets[aid]["material_class"])), True)
                for s, aid in enumerate(slot_models)
            ]
        ]
    else:
        rigid_realizations = []
    rigid_classes = sorted({str(out_assets[aid]["material_class"]) for aid in class_rigid})
    if slot_models is None and len(rigid_classes) != 1:
        raise RuntimeError(f"Expected uniform non-override Rigid class in {task_name}, got {rigid_classes}")
    for rigid_cls in rigid_classes:
        rigid_self_pk = f"{rigid_cls}__{rigid_cls}"
        n_cls_slots = n_rigid_slots if slot_models is None else sum(
            1 for aid in slot_models if str(out_assets[aid]["material_class"]) == rigid_cls
        )
        if rigid_self_pk in out_pairs:
            co_rig = ((out_pairs[rigid_self_pk].get("cooccurrence") or {}).get("episodes_scored_by_config") or {}).get(
                task_name, 0
            )
            if co_rig <= 0 and n_cls_slots >= 2 and slot_models is None:
                raise RuntimeError(f"Unexpected zero cooccurrence on {rigid_self_pk} for {task_name}")

    # Episode Rigid-slot realizations (uniform-class path): without override asset (all n_rigid_slots carry
    # rigid_cls), and with each override asset occupying 1 slot (or 2 slots iff not flagged no_self_contact).
    if slot_models is None:
        rigid_cls = rigid_classes[0]
        rigid_realizations = [
            [(f"Rigid/slot_{s}({rigid_cls})", ("class", rigid_cls), True) for s in range(n_rigid_slots)]
        ]
    for ov_aid in override_rigid:
        no_self = "no_self_contact" in (out_assets[ov_aid].get("flags") or ())
        max_ov_slots = 1 if no_self else min(2, n_rigid_slots)
        entries: list[tuple[str, tuple[str, str], bool]] = [
            (f"{ov_aid}#slot_{s}", ("override", ov_aid), True) for s in range(max_ov_slots)
        ]
        for s in range(max_ov_slots, n_rigid_slots):
            entries.append((f"Rigid/slot_{s}({rigid_cls})", ("class", rigid_cls), True))
        rigid_realizations.append(entries)

    def _build_episode_bodies(
        pad_cls: str,
        table_cls: str,
        rig_slots: list[tuple[str, tuple[str, str], bool]],
    ) -> list[tuple[str, tuple[str, str], bool]]:
        bodies: list[tuple[str, tuple[str, str], bool]] = []
        for arm_idx, arm in enumerate(tcfg["robot"]["arms"]):
            rname = str(arm["robot_name"])
            if rname != "x5":
                raise RuntimeError(f"Unsupported robot {rname!r} in {task_name}")
            for lrec in x5_links:
                lname = str(lrec["link"])
                lcls = str(non_pad_map[lname]["value"]) if lname in non_pad_map else pad_cls
                bodies.append((f"robot{arm_idx}.{lname}", ("class", lcls), True))
        for fx in tcfg["scene"]["fixtures"]:
            fx_aid = f"Scene/{fx}"
            if fx_aid not in out_assets:
                continue
            is_dyn = out_assets[fx_aid]["type"] == "Rigid"
            bodies.append((fx_aid, _carrier(fx_aid, table_override_cls=(table_cls if fx == "Table" else None)), is_dyn))
        for g_aid in geom_models:
            is_dyn = out_assets[g_aid]["type"] == "Rigid"
            bodies.append((g_aid, _carrier(g_aid), is_dyn))
        bodies.extend(rig_slots)
        return bodies

    formable_witness: dict[tuple[tuple[str, str], tuple[str, str]], str] = {}
    load_bearing_witness: dict[tuple[tuple[str, str], tuple[str, str]], str] = {}
    distinct_any_witness: dict[tuple[tuple[str, str], tuple[str, str]], str] = {}
    carried_primary_classes: set[str] = set()
    carried_all_classes: set[str] = set()
    present_carriers: set[tuple[str, str]] = set()

    scenario_cells = [
        ("default", default_pad_cls, default_table_cls),
        ("sensitivity_pad", sens_pad_cls, default_table_cls),
        ("sensitivity_table", default_pad_cls, sens_table_cls),
        ("sensitivity_both", sens_pad_cls, sens_table_cls),
    ]
    for scen_tag, pad_c, tab_c in scenario_cells:
        for rig_slots in rigid_realizations:
            ep_bodies = _build_episode_bodies(pad_c, tab_c, rig_slots)
            for _, m_car, _ in ep_bodies:
                present_carriers.add(m_car)
                if m_car[0] == "class":
                    carried_all_classes.add(m_car[1])
                    if scen_tag == "default":
                        carried_primary_classes.add(m_car[1])
            nb = len(ep_bodies)
            for i in range(nb):
                n1, m1, d1 = ep_bodies[i]
                for j in range(i + 1, nb):
                    n2, m2, d2 = ep_bodies[j]
                    pkey = (m1, m2) if m1 <= m2 else (m2, m1)
                    w_lbl = (
                        f"{n1} ({'dynamic' if d1 else 'static'}) x {n2} ({'dynamic' if d2 else 'static'})"
                    )
                    distinct_any_witness.setdefault(pkey, w_lbl)
                    if d1 or d2:
                        # Prefer inter-actor witnesses over intra-arm link pairs for readability.
                        prev = formable_witness.get(pkey)
                        is_intra_arm = (
                            n1.startswith("robot")
                            and n2.startswith("robot")
                            and n1.split(".", 1)[0] == n2.split(".", 1)[0]
                        )
                        if prev is None or (
                            not is_intra_arm
                            and prev.startswith("robot0.")
                            and " x robot0." in prev
                        ):
                            formable_witness[pkey] = w_lbl
                        # Finger on finger (both bodies finger pads) is formable but never load-bearing (#167 (b)).
                        if not (_is_finger_pad(n1) and _is_finger_pad(n2)):
                            prev_lb = load_bearing_witness.get(pkey)
                            if prev_lb is None or (
                                not is_intra_arm
                                and prev_lb.startswith("robot0.")
                                and " x robot0." in prev_lb
                            ):
                                load_bearing_witness[pkey] = w_lbl

    carried_sens_only = sorted(carried_all_classes - carried_primary_classes)

    def can_form(m_a: tuple[str, str], m_b: tuple[str, str]) -> bool:
        return ((m_a, m_b) if m_a <= m_b else (m_b, m_a)) in formable_witness

    def classify_pair(m_a: tuple[str, str], m_b: tuple[str, str]) -> tuple[bool, str]:
        """(counted in the objective, reason): True iff the pair can form a load-bearing contact in the task."""
        pkey = (m_a, m_b) if m_a <= m_b else (m_b, m_a)
        if pkey in load_bearing_witness:
            return True, f"load_bearing ({load_bearing_witness[pkey]})"
        if pkey in formable_witness:
            return False, (
                f"band_only: not_load_bearing (forms only finger on finger, e.g. {formable_witness[pkey]}; an empty close "
                f"or a handover never carries a sliding load the task depends on)"
            )
        absent_ov = [m[1] for m in (m_a, m_b) if m[0] == "override" and m not in present_carriers]
        if absent_ov:
            return False, f"band_only: absent_in_task ({','.join(sorted(set(absent_ov)))} not in {task_name})"
        uncarried = [m[1] for m in (m_a, m_b) if m[0] == "class" and m[1] not in carried_all_classes]
        if uncarried:
            ov_classes = {str(out_assets[aid]["material_class"]) for aid in override_rigid}
            if set(uncarried) <= ov_classes:
                return False, f"band_only: uncarried_class ({','.join(sorted(set(uncarried)))}; replaced by asset material_override)"
            return False, f"band_only: absent_in_task ({','.join(sorted(set(uncarried)))} carried by no engine object in {task_name})"
        if pkey not in distinct_any_witness:
            # Self-pair with at most 1 body carrying this material in any episode.
            if m_a[0] == "override":
                return False, "band_only: no_self_contact (at most 1 instance per episode)"
            return False, "band_only: no_self_contact_and_static_static (at most 1 static collider per episode)"
        return False, f"band_only: static_static ({distinct_any_witness[pkey]}; neither collider has RigidBodyAPI)"

    return {
        "task_name": task_name,
        "rule": PAIR_FORMABILITY_RULE,
        "carried_primary_classes": sorted(carried_primary_classes),
        "carried_sensitivity_classes": carried_sens_only,
        "carried_all_classes": sorted(carried_all_classes),
        "classify_pair": classify_pair,
        "can_form": can_form,
    }


def derive_t1_task_formable_pairs(
    population_src: dict[str, Any],
    *,
    task_name: str,
    task_census: dict[str, Any],
    out_assets: dict[str, dict[str, Any]],
    x5_links: list[dict[str, Any]],
    non_pad_map: dict[str, dict[str, Any]],
    default_pad_cls: str = "pad_hard",
    default_table_cls: str = "table_laminate",
    sens_pad_cls: str = "pad_elastomer",
    sens_table_cls: str = "table_wood",
) -> dict[str, Any]:
    """Derive per-task formable vs band-only pairs for a T1 config from its 3-seed episode carrier realizations."""
    tcfg = next((c for c in population_src["configs"] if c["task_name"] == task_name), None)
    if tcfg is None:
        raise RuntimeError(f"Task {task_name!r} not found in population.json")

    def _is_finger_pad(body_name: str) -> bool:
        return body_name.startswith("robot") and body_name.split(".", 1)[1] not in non_pad_map

    realizations = task_census.get("episode_carrier_realizations") or []
    if not realizations:
        raise RuntimeError(f"{task_name}: empty episode_carrier_realizations in t1_rows.json")

    formable_witness: dict[tuple[tuple[str, str], tuple[str, str]], str] = {}
    load_bearing_witness: dict[tuple[tuple[str, str], tuple[str, str]], str] = {}
    distinct_any_witness: dict[tuple[tuple[str, str], tuple[str, str]], str] = {}
    carried_primary_classes: set[str] = set()
    carried_all_classes: set[str] = set()
    present_carriers: set[tuple[str, str]] = set()

    scenario_cells = [
        ("default", default_pad_cls, default_table_cls),
        ("sensitivity_pad", sens_pad_cls, default_table_cls),
        ("sensitivity_table", default_pad_cls, sens_table_cls),
        ("sensitivity_both", sens_pad_cls, sens_table_cls),
    ]
    for scen_tag, pad_c, tab_c in scenario_cells:
        for real_rec in realizations:
            bodies: list[tuple[str, tuple[str, str], bool]] = []
            for arm_idx, arm in enumerate(tcfg["robot"]["arms"]):
                rname = str(arm["robot_name"])
                if rname != "x5":
                    raise RuntimeError(f"Unsupported robot {rname!r} in {task_name}")
                for lrec in x5_links:
                    lname = str(lrec["link"])
                    lcls = str(non_pad_map[lname]["value"]) if lname in non_pad_map else pad_c
                    bodies.append((f"robot{arm_idx}.{lname}", ("class", lcls), True))
            for fx in tcfg["scene"]["fixtures"]:
                fx_aid = f"Scene/{fx}"
                if fx_aid not in out_assets:
                    continue
                is_dyn = out_assets[fx_aid]["type"] == "Rigid"
                fcls = tab_c if fx == "Table" else str(out_assets[fx_aid]["material_class"])
                bodies.append((fx_aid, ("class", fcls), is_dyn))
            for idx_w, w_item in enumerate(real_rec["witness_objects"]):
                sec_w, aid_w, _ = str(w_item[0]), str(w_item[1]), str(w_item[2])
                ad_w = out_assets[aid_w]
                if "visual_marker_no_collider" in (ad_w.get("flags") or ()):
                    continue
                car_w = ("override", aid_w) if "material_override" in ad_w else ("class", str(ad_w["material_class"]))
                bodies.append((f"{aid_w}#{idx_w}", car_w, sec_w == "Rigid"))

            for _, m_car, _ in bodies:
                present_carriers.add(m_car)
                if m_car[0] == "class":
                    carried_all_classes.add(m_car[1])
                    if scen_tag == "default":
                        carried_primary_classes.add(m_car[1])
            nb = len(bodies)
            for i in range(nb):
                n1, m1, d1 = bodies[i]
                for j in range(i + 1, nb):
                    n2, m2, d2 = bodies[j]
                    pkey = (m1, m2) if m1 <= m2 else (m2, m1)
                    w_lbl = f"{n1} ({'dynamic' if d1 else 'static'}) x {n2} ({'dynamic' if d2 else 'static'})"
                    distinct_any_witness.setdefault(pkey, w_lbl)
                    if d1 or d2:
                        prev = formable_witness.get(pkey)
                        is_intra_arm = (
                            n1.startswith("robot")
                            and n2.startswith("robot")
                            and n1.split(".", 1)[0] == n2.split(".", 1)[0]
                        )
                        if prev is None or (not is_intra_arm and prev.startswith("robot0.") and " x robot0." in prev):
                            formable_witness[pkey] = w_lbl
                        if not (_is_finger_pad(n1) and _is_finger_pad(n2)):
                            prev_lb = load_bearing_witness.get(pkey)
                            if prev_lb is None or (
                                not is_intra_arm and prev_lb.startswith("robot0.") and " x robot0." in prev_lb
                            ):
                                load_bearing_witness[pkey] = w_lbl

    carried_sens_only = sorted(carried_all_classes - carried_primary_classes)

    def can_form(m_a: tuple[str, str], m_b: tuple[str, str]) -> bool:
        return ((m_a, m_b) if m_a <= m_b else (m_b, m_a)) in formable_witness

    def classify_pair(m_a: tuple[str, str], m_b: tuple[str, str]) -> tuple[bool, str]:
        pkey = (m_a, m_b) if m_a <= m_b else (m_b, m_a)
        if pkey in load_bearing_witness:
            return True, f"load_bearing ({load_bearing_witness[pkey]})"
        if pkey in formable_witness:
            return False, (
                f"band_only: not_load_bearing (forms only finger on finger, e.g. {formable_witness[pkey]}; an empty close "
                f"or a handover never carries a sliding load the task depends on)"
            )
        absent_ov = [m[1] for m in (m_a, m_b) if m[0] == "override" and m not in present_carriers]
        if absent_ov:
            return False, f"band_only: absent_in_task ({','.join(sorted(set(absent_ov)))} not in {task_name})"
        uncarried = [m[1] for m in (m_a, m_b) if m[0] == "class" and m[1] not in carried_all_classes]
        if uncarried:
            return False, f"band_only: absent_in_task ({','.join(sorted(set(uncarried)))} carried by no engine object in {task_name})"
        if pkey not in distinct_any_witness:
            return False, "band_only: no_self_contact_and_static_static (at most 1 static collider per episode)"
        return False, f"band_only: static_static ({distinct_any_witness[pkey]}; neither collider has RigidBodyAPI)"

    return {
        "task_name": task_name,
        "rule": PAIR_FORMABILITY_RULE,
        "carried_primary_classes": sorted(carried_primary_classes),
        "carried_sensitivity_classes": carried_sens_only,
        "carried_all_classes": sorted(carried_all_classes),
        "classify_pair": classify_pair,
        "can_form": can_form,
    }


def combine_task_formability(per_task: list[dict[str, Any]]) -> dict[str, Any]:
    """Union of per-task formability (#167 decision 2): a pair counts iff it can form a load-bearing contact in at
    least one task in scope (#167 (b))."""
    task_names = [f["task_name"] for f in per_task]
    carried_prim: set[str] = set()
    carried_all: set[str] = set()
    for f in per_task:
        carried_prim |= set(f["carried_primary_classes"])
        carried_all |= set(f["carried_all_classes"])

    def formable_tasks(m_a: tuple[str, str], m_b: tuple[str, str]) -> list[str]:
        """Tasks where the pair can form at all (load-bearing or not)."""
        return [f["task_name"] for f in per_task if f["can_form"](m_a, m_b)]

    def classify_pair(m_a: tuple[str, str], m_b: tuple[str, str]) -> tuple[bool, str]:
        per = [(f["task_name"],) + tuple(f["classify_pair"](m_a, m_b)) for f in per_task]
        yes = [(t, r) for t, ok, r in per if ok]
        if yes:
            return True, f"load-bearing in {', '.join(t for t, _ in yes)}; {yes[0][1]}"
        informative = [(t, r) for t, ok, r in per if not str(r).startswith("band_only: absent_in_task")]
        shown = informative or [(t, r) for t, ok, r in per]
        return False, "; ".join(f"{t}: {r}" for t, r in shown)

    return {
        "task_name": " + ".join(task_names),
        "task_names": task_names,
        "rule": PAIR_FORMABILITY_RULE,
        "carried_primary_classes": sorted(carried_prim),
        "carried_sensitivity_classes": sorted(carried_all - carried_prim),
        "carried_all_classes": sorted(carried_all),
        "per_task_carried_classes": {f["task_name"]: f["carried_all_classes"] for f in per_task},
        "classify_pair": classify_pair,
        "formable_tasks": formable_tasks,
    }


def solve_nominal_tracking_unaries(
    out_pairs: dict[str, dict[str, Any]],
    task_classes: tuple[str, ...],
    combine_modes: dict[str, str],
    out_assets: dict[str, dict[str, Any]],
    formability: dict[str, Any],
    census_weights: dict[str, Any] | None = None,
    t1_uncensused_classes: set[str] | None = None,
) -> tuple[dict[str, float], dict[str, float], dict[str, Any], dict[str, Any]]:
    """Compute the Variant A nominal-tracking L2 unary friction map (#62 B1/B5; presence-weighted across tasks, #59/#167)."""
    non_avg = {c: m for c, m in combine_modes.items() if m != "average"}
    if non_avg:
        raise RuntimeError(f"Linear average-combine QP expects 'average' combine mode on all classes, got {non_avg}")

    classify_pair = formability["classify_pair"]
    formable_tasks = formability.get("formable_tasks") or (
        lambda m_a, m_b: [formability["task_name"]] if classify_pair(m_a, m_b)[0] else []
    )
    carried_prim_classes: list[str] = list(formability["carried_primary_classes"])
    carried_sens_classes: list[str] = list(formability["carried_sensitivity_classes"])
    carried_all_set = set(carried_prim_classes) | set(carried_sens_classes)
    uncarried_classes = sorted(set(task_classes) - carried_all_set)

    pairs_list = [out_pairs[k] for k in sorted(out_pairs.keys())]
    pair_by_canon = {(min(p["a"], p["b"]), max(p["a"], p["b"])): p for p in pairs_list}

    prim_pairs = [
        p
        for p in pairs_list
        if p["scenario_membership"] != "sensitivity"
        and p["a"] in carried_prim_classes
        and p["b"] in carried_prim_classes
    ]

    counted_stage1_keys: list[str] = []
    counted_stage2_keys: list[str] = []
    excluded_non_formable: dict[str, str] = {}

    pw_map: dict[str, Fraction] | None = census_weights["pair_weights"] if census_weights is not None else None

    def _w_prim(pk: str, can_form_lb: bool) -> Fraction:
        if not can_form_lb:
            return Fraction(0)
        if pw_map is None:
            return Fraction(1)
        return pw_map.get(pk, Fraction(0))

    # Determine Stage 1 classes (carried primary classes with positive objective weight) and excluded non-formable pairs.
    pos_prim: set[str] = set()
    formable_lb_prim_keys: list[str] = []
    for p in prim_pairs:
        pk = p["key"]
        can_form_lb, reason = classify_pair(("class", p["a"]), ("class", p["b"]))
        if can_form_lb:
            formable_lb_prim_keys.append(pk)
            w = _w_prim(pk, True)
            if w > 0:
                pos_prim.add(p["a"])
                pos_prim.add(p["b"])
                counted_stage1_keys.append(pk)
        else:
            excluded_non_formable[pk] = reason

    for aid, adict in sorted(out_assets.items()):
        if "material_override" not in adict:
            continue
        mo = adict["material_override"]
        if mo.get("friction_combine_mode", "average") != "average":
            raise RuntimeError(f"Unsupported override friction_combine_mode on {aid}")
        acls = str(adict["material_class"])
        no_self = "no_self_contact" in (adict.get("flags") or ())
        if no_self:
            _, self_reason = classify_pair(("override", aid), ("override", aid))
            excluded_non_formable[f"{aid}__{aid}"] = self_reason
        for partner in carried_prim_classes:
            if partner == acls and no_self:
                continue
            pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
            if pb is None:
                continue
            opk = f"{aid}__{partner}"
            can_form_lb, reason = classify_pair(("override", aid), ("class", partner))
            if can_form_lb:
                formable_lb_prim_keys.append(opk)
                w = _w_prim(opk, True)
                if w > 0:
                    pos_prim.add(partner)
                    counted_stage1_keys.append(opk)
            else:
                excluded_non_formable[opk] = reason

    stage1_classes = [c for c in carried_prim_classes if c in pos_prim]
    zero_weight_prim = [c for c in carried_prim_classes if c not in pos_prim]
    np1 = len(stage1_classes)
    pidx = {c: i for i, c in enumerate(stage1_classes)}
    h1 = [[Fraction(0)] * (2 * np1) for _ in range(2 * np1)]
    g1 = [Fraction(0)] * (2 * np1)
    a1: list[list[Fraction]] = []
    b1: list[Fraction] = []
    lbl1: list[str] = []

    # Stage 1: Exact rational joint L2 fit over weighted primary-scenario pairs on stage1_classes,
    # subject to all direct Stage 1 bands, exact 1-hop Fourier-Motzkin cross-band feasibility for all non-Stage-1
    # classes, override bands, and 0 <= u_k <= u_s.
    for p in prim_pairs:
        if p["a"] not in pidx or p["b"] not in pidx:
            continue
        pk = p["key"]
        ia, ib = pidx[p["a"]], pidx[p["b"]]
        vs, vk = _f(p["mu_static"]["value"]), _f(p["mu_kinetic"]["value"])
        slo, shi = _f(p["mu_static"]["range"][0]), _f(p["mu_static"]["range"][1])
        klo, khi = _f(p["mu_kinetic"]["range"][0]), _f(p["mu_kinetic"]["range"][1])
        can_form_lb, _ = classify_pair(("class", p["a"]), ("class", p["b"]))
        w = _w_prim(pk, can_form_lb)
        if w > 0:
            for u in (ia, ib):
                g1[u] -= w * vs
                for v in (ia, ib):
                    h1[u][v] += w * Fraction(1, 2)
            for u in (np1 + ia, np1 + ib):
                g1[u] -= w * vk
                for v in (np1 + ia, np1 + ib):
                    h1[u][v] += w * Fraction(1, 2)

        r = [Fraction(0)] * (2 * np1)
        r[ia] += Fraction(1, 2)
        r[ib] += Fraction(1, 2)
        a1.append(r)
        b1.append(shi)
        lbl1.append(f"{pk}.mu_static<=hi")

        r = [Fraction(0)] * (2 * np1)
        r[ia] -= Fraction(1, 2)
        r[ib] -= Fraction(1, 2)
        a1.append(r)
        b1.append(-slo)
        lbl1.append(f"{pk}.mu_static>=lo")

        r = [Fraction(0)] * (2 * np1)
        r[np1 + ia] += Fraction(1, 2)
        r[np1 + ib] += Fraction(1, 2)
        a1.append(r)
        b1.append(khi)
        lbl1.append(f"{pk}.mu_kinetic<=hi")

        r = [Fraction(0)] * (2 * np1)
        r[np1 + ia] -= Fraction(1, 2)
        r[np1 + ib] -= Fraction(1, 2)
        a1.append(r)
        b1.append(-klo)
        lbl1.append(f"{pk}.mu_kinetic>=lo")

    for aid, adict in sorted(out_assets.items()):
        if "material_override" not in adict:
            continue
        mo = adict["material_override"]
        acls = str(adict["material_class"])
        no_self = "no_self_contact" in (adict.get("flags") or ())
        os_val, ok_val = _f(mo["static_friction"]), _f(mo["dynamic_friction"])
        for partner in stage1_classes:
            if partner == acls and no_self:
                continue
            pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
            if pb is None:
                continue
            opk = f"{aid}__{partner}"
            ip = pidx[partner]
            vs, vk = _f(pb["mu_static"]["value"]), _f(pb["mu_kinetic"]["value"])
            slo, shi = _f(pb["mu_static"]["range"][0]), _f(pb["mu_static"]["range"][1])
            klo, khi = _f(pb["mu_kinetic"]["range"][0]), _f(pb["mu_kinetic"]["range"][1])
            can_form_lb, _ = classify_pair(("override", aid), ("class", partner))
            w = _w_prim(opk, can_form_lb)
            if w > 0:
                h1[ip][ip] += w * Fraction(1, 2)
                g1[ip] += w * (Fraction(1, 2) * os_val - vs)
                h1[np1 + ip][np1 + ip] += w * Fraction(1, 2)
                g1[np1 + ip] += w * (Fraction(1, 2) * ok_val - vk)

            r = [Fraction(0)] * (2 * np1)
            r[ip] = Fraction(1, 2)
            a1.append(r)
            b1.append(shi - Fraction(1, 2) * os_val)
            lbl1.append(f"{opk}.mu_static<=hi")

            r = [Fraction(0)] * (2 * np1)
            r[ip] = Fraction(-1, 2)
            a1.append(r)
            b1.append(-(slo - Fraction(1, 2) * os_val))
            lbl1.append(f"{opk}.mu_static>=lo")

            r = [Fraction(0)] * (2 * np1)
            r[np1 + ip] = Fraction(1, 2)
            a1.append(r)
            b1.append(khi - Fraction(1, 2) * ok_val)
            lbl1.append(f"{opk}.mu_kinetic<=hi")

            r = [Fraction(0)] * (2 * np1)
            r[np1 + ip] = Fraction(-1, 2)
            a1.append(r)
            b1.append(-(klo - Fraction(1, 2) * ok_val))
            lbl1.append(f"{opk}.mu_kinetic>=lo")

    # Exact 1-hop Fourier-Motzkin cross-band feasibility projection for every non-Stage-1 class onto Stage 1 variables.
    non_s1 = sorted(set(task_classes) - set(stage1_classes))
    for z in non_s1:
        ls_terms: list[tuple[Fraction, int | None, str]] = [(Fraction(0), None, f"{z}.s>=0")]
        us_terms: list[tuple[Fraction, int | None, str]] = []
        lk_terms: list[tuple[Fraction, int | None, str]] = [(Fraction(0), None, f"{z}.k>=0")]
        uk_terms: list[tuple[Fraction, int | None, str]] = []
        self_p = pair_by_canon.get((z, z))
        if self_p is not None:
            ls_terms.append((_f(self_p["mu_static"]["range"][0]), None, f"{z}__{z}.slo"))
            us_terms.append((_f(self_p["mu_static"]["range"][1]), None, f"{z}__{z}.shi"))
            lk_terms.append((_f(self_p["mu_kinetic"]["range"][0]), None, f"{z}__{z}.klo"))
            uk_terms.append((_f(self_p["mu_kinetic"]["range"][1]), None, f"{z}__{z}.khi"))
        for aid, adict in sorted(out_assets.items()):
            if "material_override" not in adict:
                continue
            acls = str(adict["material_class"])
            no_self = "no_self_contact" in (adict.get("flags") or ())
            if z == acls and no_self:
                continue
            pb = pair_by_canon.get((min(acls, z), max(acls, z)))
            if pb is None:
                continue
            mo = adict["material_override"]
            os_v, ok_v = _f(mo["static_friction"]), _f(mo["dynamic_friction"])
            ls_terms.append((2 * _f(pb["mu_static"]["range"][0]) - os_v, None, f"{aid}__{z}.slo"))
            us_terms.append((2 * _f(pb["mu_static"]["range"][1]) - os_v, None, f"{aid}__{z}.shi"))
            lk_terms.append((2 * _f(pb["mu_kinetic"]["range"][0]) - ok_v, None, f"{aid}__{z}.klo"))
            uk_terms.append((2 * _f(pb["mu_kinetic"]["range"][1]) - ok_v, None, f"{aid}__{z}.khi"))
        for x in stage1_classes:
            pb = pair_by_canon.get((min(x, z), max(x, z)))
            if pb is None:
                continue
            ix = pidx[x]
            ls_terms.append((2 * _f(pb["mu_static"]["range"][0]), ix, f"{pb['key']}.slo"))
            us_terms.append((2 * _f(pb["mu_static"]["range"][1]), ix, f"{pb['key']}.shi"))
            lk_terms.append((2 * _f(pb["mu_kinetic"]["range"][0]), np1 + ix, f"{pb['key']}.klo"))
            uk_terms.append((2 * _f(pb["mu_kinetic"]["range"][1]), np1 + ix, f"{pb['key']}.khi"))

        def _add_fm_pairs(
            l_list: list[tuple[Fraction, int | None, str]],
            u_list: list[tuple[Fraction, int | None, str]],
            tag_mid: str,
        ) -> None:
            for c_l, v_l, n_l in l_list:
                for c_u, v_u, n_u in u_list:
                    if v_l is None and v_u is None:
                        if c_l > c_u:
                            raise RuntimeError(f"Infeasible fixed FM bound on {z}: {n_l}={c_l} > {n_u}={c_u}")
                        continue
                    if v_l == v_u:
                        if c_l > c_u:
                            raise RuntimeError(f"Infeasible self FM bound on {z}: {n_l} > {n_u}")
                        continue
                    r = [Fraction(0)] * (2 * np1)
                    if v_u is not None:
                        r[v_u] += Fraction(1)
                    if v_l is not None:
                        r[v_l] -= Fraction(1)
                    a1.append(r)
                    b1.append(c_u - c_l)
                    lbl1.append(f"fm[{z}:{tag_mid}]:{n_l}<={n_u}")

        _add_fm_pairs(ls_terms, us_terms, "s")
        _add_fm_pairs(lk_terms, uk_terms, "k")
        _add_fm_pairs(lk_terms, us_terms, "k<=s")

    for i, cname in enumerate(stage1_classes):
        r = [Fraction(0)] * (2 * np1)
        r[np1 + i] = Fraction(1)
        r[i] = Fraction(-1)
        a1.append(r)
        b1.append(Fraction(0))
        lbl1.append(f"{cname}.u_k<=u_s")

        r = [Fraction(0)] * (2 * np1)
        r[np1 + i] = Fraction(-1)
        a1.append(r)
        b1.append(Fraction(0))
        lbl1.append(f"{cname}.u_k>=0")

    if pw_map is not None:
        dedup_1: dict[tuple[Fraction, ...], tuple[Fraction, str]] = {}
        for r, bv, lb in zip(a1, b1, lbl1):
            rk = tuple(r)
            cur = dedup_1.get(rk)
            if cur is None or bv < cur[0]:
                dedup_1[rk] = (bv, lb)
        a1 = [list(rk) for rk in dedup_1]
        b1 = [v[0] for v in dedup_1.values()]
        lbl1 = [v[1] for v in dedup_1.values()]

    x1, act1 = _solve_exact_kkt_qp(h1, g1, a1, b1)
    us_exact: dict[str, Fraction] = {stage1_classes[i]: x1[i] for i in range(np1)}
    uk_exact: dict[str, Fraction] = {stage1_classes[i]: x1[np1 + i] for i in range(np1)}

    # Record non-formable sensitivity pairs for provenance completeness.
    for p in pairs_list:
        if p["scenario_membership"] == "sensitivity" and p["a"] in carried_all_set and p["b"] in carried_all_set:
            can_form_s, reason_s = classify_pair(("class", p["a"]), ("class", p["b"]))
            if not can_form_s:
                excluded_non_formable[p["key"]] = reason_s
    for aid, adict in sorted(out_assets.items()):
        if "material_override" not in adict:
            continue
        acls = str(adict["material_class"])
        for partner in carried_sens_classes:
            pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
            if pb is None:
                continue
            can_form_so, reason_so = classify_pair(("override", aid), ("class", partner))
            if not can_form_so:
                excluded_non_formable[f"{aid}__{partner}"] = reason_so

    # Stage 2: Sensitivity classes (pad_elastomer, table_wood) with counterpart presence weights, holding Stage 1 fixed.
    act2: list[int] = []
    lbl2: list[str] = []
    s1_or_s2 = set(stage1_classes) | set(carried_sens_classes)
    sens_pairs = [
        p
        for p in pairs_list
        if p["scenario_membership"] == "sensitivity"
        and p["a"] in s1_or_s2
        and p["b"] in s1_or_s2
    ]

    def _w_sens(pk: str, can_form_lb: bool) -> Fraction:
        if not can_form_lb:
            return Fraction(0)
        if pw_map is None:
            return Fraction(1)
        return pw_map.get(_sens_counterpart_key(pk), Fraction(0))

    stage2_pos: list[str] = []
    stage2_zero: list[str] = []
    if carried_sens_classes:
        s_pos_set: set[str] = set()
        for p in sens_pairs:
            can_form_lb, _ = classify_pair(("class", p["a"]), ("class", p["b"]))
            if _w_sens(p["key"], can_form_lb) > 0:
                for c in (p["a"], p["b"]):
                    if c in carried_sens_classes:
                        s_pos_set.add(c)
        for aid, adict in sorted(out_assets.items()):
            if "material_override" not in adict:
                continue
            acls = str(adict["material_class"])
            for partner in carried_sens_classes:
                pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
                if pb is None:
                    continue
                opk = f"{aid}__{partner}"
                can_form_lb, _ = classify_pair(("override", aid), ("class", partner))
                if _w_sens(opk, can_form_lb) > 0:
                    s_pos_set.add(partner)
        stage2_pos = [c for c in carried_sens_classes if c in s_pos_set]
        stage2_zero = [c for c in carried_sens_classes if c not in s_pos_set]

    if stage2_pos:
        ns2 = len(stage2_pos)
        sidx = {c: i for i, c in enumerate(stage2_pos)}
        s1_or_s2pos = set(stage1_classes) | set(stage2_pos)
        h2 = [[Fraction(0)] * (2 * ns2) for _ in range(2 * ns2)]
        g2 = [Fraction(0)] * (2 * ns2)
        a2: list[list[Fraction]] = []
        b2: list[Fraction] = []
        for p in sens_pairs:
            ca, cb = p["a"], p["b"]
            if ca not in s1_or_s2pos or cb not in s1_or_s2pos:
                continue
            pk = p["key"]
            vs, vk = _f(p["mu_static"]["value"]), _f(p["mu_kinetic"]["value"])
            slo, shi = _f(p["mu_static"]["range"][0]), _f(p["mu_static"]["range"][1])
            klo, khi = _f(p["mu_kinetic"]["range"][0]), _f(p["mu_kinetic"]["range"][1])
            s_vars: list[int] = []
            s_const = Fraction(0)
            k_const = Fraction(0)
            for end_c in (ca, cb):
                if end_c in sidx:
                    s_vars.append(sidx[end_c])
                else:
                    s_const += Fraction(1, 2) * us_exact[end_c]
                    k_const += Fraction(1, 2) * uk_exact[end_c]
            can_form_lb, _ = classify_pair(("class", ca), ("class", cb))
            w = _w_sens(pk, can_form_lb)
            if w > 0:
                counted_stage2_keys.append(pk)
                res_s = s_const - vs
                res_k = k_const - vk
                for u in s_vars:
                    g2[u] += w * res_s
                    g2[ns2 + u] += w * res_k
                    for v in s_vars:
                        h2[u][v] += w * Fraction(1, 2)
                        h2[ns2 + u][ns2 + v] += w * Fraction(1, 2)

            r = [Fraction(0)] * (2 * ns2)
            for u in s_vars:
                r[u] += Fraction(1, 2)
            a2.append(r)
            b2.append(shi - s_const)
            lbl2.append(f"{pk}.mu_static<=hi")

            r = [Fraction(0)] * (2 * ns2)
            for u in s_vars:
                r[u] -= Fraction(1, 2)
            a2.append(r)
            b2.append(-(slo - s_const))
            lbl2.append(f"{pk}.mu_static>=lo")

            r = [Fraction(0)] * (2 * ns2)
            for u in s_vars:
                r[ns2 + u] += Fraction(1, 2)
            a2.append(r)
            b2.append(khi - k_const)
            lbl2.append(f"{pk}.mu_kinetic<=hi")

            r = [Fraction(0)] * (2 * ns2)
            for u in s_vars:
                r[ns2 + u] -= Fraction(1, 2)
            a2.append(r)
            b2.append(-(klo - k_const))
            lbl2.append(f"{pk}.mu_kinetic>=lo")

        for aid, adict in sorted(out_assets.items()):
            if "material_override" not in adict:
                continue
            mo = adict["material_override"]
            acls = str(adict["material_class"])
            os_val, ok_val = _f(mo["static_friction"]), _f(mo["dynamic_friction"])
            for partner in stage2_pos:
                pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
                if pb is None:
                    continue
                opk = f"{aid}__{partner}"
                ip = sidx[partner]
                vs, vk = _f(pb["mu_static"]["value"]), _f(pb["mu_kinetic"]["value"])
                slo, shi = _f(pb["mu_static"]["range"][0]), _f(pb["mu_static"]["range"][1])
                klo, khi = _f(pb["mu_kinetic"]["range"][0]), _f(pb["mu_kinetic"]["range"][1])
                can_form_lb, _ = classify_pair(("override", aid), ("class", partner))
                w = _w_sens(opk, can_form_lb)
                if w > 0:
                    counted_stage2_keys.append(opk)
                    h2[ip][ip] += w * Fraction(1, 2)
                    g2[ip] += w * (Fraction(1, 2) * os_val - vs)
                    h2[ns2 + ip][ns2 + ip] += w * Fraction(1, 2)
                    g2[ns2 + ip] += w * (Fraction(1, 2) * ok_val - vk)

                r = [Fraction(0)] * (2 * ns2)
                r[ip] = Fraction(1, 2)
                a2.append(r)
                b2.append(shi - Fraction(1, 2) * os_val)
                lbl2.append(f"{opk}.mu_static<=hi")

                r = [Fraction(0)] * (2 * ns2)
                r[ip] = Fraction(-1, 2)
                a2.append(r)
                b2.append(-(slo - Fraction(1, 2) * os_val))
                lbl2.append(f"{opk}.mu_static>=lo")

                r = [Fraction(0)] * (2 * ns2)
                r[ns2 + ip] = Fraction(1, 2)
                a2.append(r)
                b2.append(khi - Fraction(1, 2) * ok_val)
                lbl2.append(f"{opk}.mu_kinetic<=hi")

                r = [Fraction(0)] * (2 * ns2)
                r[ns2 + ip] = Fraction(-1, 2)
                a2.append(r)
                b2.append(-(klo - Fraction(1, 2) * ok_val))
                lbl2.append(f"{opk}.mu_kinetic>=lo")

        for i, cname in enumerate(stage2_pos):
            r = [Fraction(0)] * (2 * ns2)
            r[ns2 + i] = Fraction(1)
            r[i] = Fraction(-1)
            a2.append(r)
            b2.append(Fraction(0))
            lbl2.append(f"{cname}.u_k<=u_s")

            r = [Fraction(0)] * (2 * ns2)
            r[ns2 + i] = Fraction(-1)
            a2.append(r)
            b2.append(Fraction(0))
            lbl2.append(f"{cname}.u_k>=0")

        x2, act2 = _solve_exact_kkt_qp(h2, g2, a2, b2)
        for i, cname in enumerate(stage2_pos):
            us_exact[cname] = x2[i]
            uk_exact[cname] = x2[ns2 + i]

    # Stage 3: Zero-weight classes never steer Stage 1 or Stage 2. Under presence weighting (#59 option 3), Stage 3
    # splits lexicographically into Stage 3a (zero-weight classes of the censused tasks, with exact 1-hop Fourier-Motzkin
    # cross-band feasibility projection from uncensused-only classes) and Stage 3b (classes appearing only in uncensused
    # tasks, solved holding Stages 1, 2 and 3a fixed), so uncensused configs enter purely as band constraints and cannot
    # pull any censused-task class unary unless a cross-band binds.
    stage3_classes = sorted(set(zero_weight_prim) | set(stage2_zero) | set(uncarried_classes))
    if census_weights is not None and (formability.get("per_task_carried_classes") or {}):
        censused_t = list(census_weights.get("censused_tasks") or ())
        ptc = formability["per_task_carried_classes"]
        censused_cls_set = (
            {c for t in censused_t for c in ptc.get(t, ())}
            | set(ROBOT_LINK_CLASSES)
            | {str(ad["material_class"]) for ad in out_assets.values() if "material_override" in ad}
        )
        stage3a_classes = [c for c in stage3_classes if c in censused_cls_set]
        rem_uncensused = [c for c in stage3_classes if c not in censused_cls_set]
        if t1_uncensused_classes is not None:
            stage3b_classes = [c for c in rem_uncensused if c in t1_uncensused_classes]
            stage3c_classes = [c for c in rem_uncensused if c not in t1_uncensused_classes]
        else:
            stage3b_classes = rem_uncensused
            stage3c_classes = []
    else:
        stage3a_classes = list(stage3_classes)
        stage3b_classes = []
        stage3c_classes = []

    def _solve_zero_weight_substage(
        sub_classes: list[str],
        later_classes: list[str],
    ) -> list[str]:
        if not sub_classes:
            return []
        nu = len(sub_classes)
        uidx_sub = {c: i for i, c in enumerate(sub_classes)}
        allowed_end = set(us_exact.keys()) | set(sub_classes)
        hs = [[Fraction(0)] * (2 * nu) for _ in range(2 * nu)]
        gs = [Fraction(0)] * (2 * nu)
        as_mat: list[list[Fraction]] = []
        bs_vec: list[Fraction] = []
        lbls: list[str] = []
        for p in pairs_list:
            pk = p["key"]
            ca, cb = p["a"], p["b"]
            if ca not in uidx_sub and cb not in uidx_sub:
                continue
            if ca not in allowed_end or cb not in allowed_end:
                continue
            if ca in uncarried_classes or cb in uncarried_classes:
                can_form_u, reason_u = classify_pair(("class", ca), ("class", cb))
                if can_form_u:
                    raise RuntimeError(f"Pair {pk} involving uncarried class unexpectedly classified formable")
                excluded_non_formable[pk] = reason_u
            vs, vk = _f(p["mu_static"]["value"]), _f(p["mu_kinetic"]["value"])
            slo, shi = _f(p["mu_static"]["range"][0]), _f(p["mu_static"]["range"][1])
            klo, khi = _f(p["mu_kinetic"]["range"][0]), _f(p["mu_kinetic"]["range"][1])
            u_vars: list[int] = []
            s_const = Fraction(0)
            k_const = Fraction(0)
            for end_c in (ca, cb):
                if end_c in uidx_sub:
                    u_vars.append(uidx_sub[end_c])
                else:
                    s_const += Fraction(1, 2) * us_exact[end_c]
                    k_const += Fraction(1, 2) * uk_exact[end_c]
            res_s = s_const - vs
            res_k = k_const - vk
            for u in u_vars:
                gs[u] += res_s
                gs[nu + u] += res_k
                for v in u_vars:
                    hs[u][v] += Fraction(1, 2)
                    hs[nu + u][nu + v] += Fraction(1, 2)
            for sign, bnd, off, tag in (
                (Fraction(1, 2), shi - s_const, 0, "mu_static<=hi"),
                (Fraction(-1, 2), -(slo - s_const), 0, "mu_static>=lo"),
                (Fraction(1, 2), khi - k_const, nu, "mu_kinetic<=hi"),
                (Fraction(-1, 2), -(klo - k_const), nu, "mu_kinetic>=lo"),
            ):
                r = [Fraction(0)] * (2 * nu)
                for u in u_vars:
                    r[off + u] += sign
                as_mat.append(r)
                bs_vec.append(bnd)
                lbls.append(f"{pk}.{tag}")

        for aid, adict in sorted(out_assets.items()):
            if "material_override" not in adict:
                continue
            mo = adict["material_override"]
            acls = str(adict["material_class"])
            no_self = "no_self_contact" in (adict.get("flags") or ())
            os_val, ok_val = _f(mo["static_friction"]), _f(mo["dynamic_friction"])
            for partner in sub_classes:
                if partner == acls and no_self:
                    continue
                pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
                if pb is None:
                    continue
                opk = f"{aid}__{partner}"
                ip = uidx_sub[partner]
                slo, shi = _f(pb["mu_static"]["range"][0]), _f(pb["mu_static"]["range"][1])
                klo, khi = _f(pb["mu_kinetic"]["range"][0]), _f(pb["mu_kinetic"]["range"][1])
                for sign, bnd, off, tag in (
                    (Fraction(1, 2), shi - Fraction(1, 2) * os_val, 0, "mu_static<=hi"),
                    (Fraction(-1, 2), -(slo - Fraction(1, 2) * os_val), 0, "mu_static>=lo"),
                    (Fraction(1, 2), khi - Fraction(1, 2) * ok_val, nu, "mu_kinetic<=hi"),
                    (Fraction(-1, 2), -(klo - Fraction(1, 2) * ok_val), nu, "mu_kinetic>=lo"),
                ):
                    r = [Fraction(0)] * (2 * nu)
                    r[off + ip] = sign
                    as_mat.append(r)
                    bs_vec.append(bnd)
                    lbls.append(f"{opk}.{tag}")

        # 1-hop Fourier-Motzkin cross-band projection from later_classes onto sub_classes (holding solved classes fixed).
        for z in later_classes:
            ls_t: list[tuple[Fraction, int | None, str]] = [(Fraction(0), None, f"{z}.s>=0")]
            us_t: list[tuple[Fraction, int | None, str]] = []
            lk_t: list[tuple[Fraction, int | None, str]] = [(Fraction(0), None, f"{z}.k>=0")]
            uk_t: list[tuple[Fraction, int | None, str]] = []
            self_pz = pair_by_canon.get((z, z))
            if self_pz is not None:
                ls_t.append((_f(self_pz["mu_static"]["range"][0]), None, f"{z}__{z}.slo"))
                us_t.append((_f(self_pz["mu_static"]["range"][1]), None, f"{z}__{z}.shi"))
                lk_t.append((_f(self_pz["mu_kinetic"]["range"][0]), None, f"{z}__{z}.klo"))
                uk_t.append((_f(self_pz["mu_kinetic"]["range"][1]), None, f"{z}__{z}.khi"))
            for aid, adict in sorted(out_assets.items()):
                if "material_override" not in adict:
                    continue
                acls = str(adict["material_class"])
                if z == acls and "no_self_contact" in (adict.get("flags") or ()):
                    continue
                pb = pair_by_canon.get((min(acls, z), max(acls, z)))
                if pb is None:
                    continue
                mo = adict["material_override"]
                os_v, ok_v = _f(mo["static_friction"]), _f(mo["dynamic_friction"])
                ls_t.append((2 * _f(pb["mu_static"]["range"][0]) - os_v, None, f"{aid}__{z}.slo"))
                us_t.append((2 * _f(pb["mu_static"]["range"][1]) - os_v, None, f"{aid}__{z}.shi"))
                lk_t.append((2 * _f(pb["mu_kinetic"]["range"][0]) - ok_v, None, f"{aid}__{z}.klo"))
                uk_t.append((2 * _f(pb["mu_kinetic"]["range"][1]) - ok_v, None, f"{aid}__{z}.khi"))
            for x_fix in sorted(us_exact.keys()):
                pb = pair_by_canon.get((min(x_fix, z), max(x_fix, z)))
                if pb is None:
                    continue
                ls_t.append((2 * _f(pb["mu_static"]["range"][0]) - us_exact[x_fix], None, f"{pb['key']}.slo"))
                us_t.append((2 * _f(pb["mu_static"]["range"][1]) - us_exact[x_fix], None, f"{pb['key']}.shi"))
                lk_t.append((2 * _f(pb["mu_kinetic"]["range"][0]) - uk_exact[x_fix], None, f"{pb['key']}.klo"))
                uk_t.append((2 * _f(pb["mu_kinetic"]["range"][1]) - uk_exact[x_fix], None, f"{pb['key']}.khi"))
            for x_sub in sub_classes:
                pb = pair_by_canon.get((min(x_sub, z), max(x_sub, z)))
                if pb is None:
                    continue
                ix = uidx_sub[x_sub]
                ls_t.append((2 * _f(pb["mu_static"]["range"][0]), ix, f"{pb['key']}.slo"))
                us_t.append((2 * _f(pb["mu_static"]["range"][1]), ix, f"{pb['key']}.shi"))
                lk_t.append((2 * _f(pb["mu_kinetic"]["range"][0]), nu + ix, f"{pb['key']}.klo"))
                uk_t.append((2 * _f(pb["mu_kinetic"]["range"][1]), nu + ix, f"{pb['key']}.khi"))

            def _add_fm_sub(
                l_list: list[tuple[Fraction, int | None, str]],
                u_list: list[tuple[Fraction, int | None, str]],
                tag_mid: str,
            ) -> None:
                for c_l, v_l, n_l in l_list:
                    for c_u, v_u, n_u in u_list:
                        if v_l is None and v_u is None:
                            if c_l > c_u:
                                raise RuntimeError(f"Infeasible fixed Stage 3 FM bound on {z}: {n_l}={c_l} > {n_u}={c_u}")
                            continue
                        if v_l == v_u:
                            if c_l > c_u:
                                raise RuntimeError(f"Infeasible self Stage 3 FM bound on {z}: {n_l} > {n_u}")
                            continue
                        r = [Fraction(0)] * (2 * nu)
                        if v_u is not None:
                            r[v_u] += Fraction(1)
                        if v_l is not None:
                            r[v_l] -= Fraction(1)
                        as_mat.append(r)
                        bs_vec.append(c_u - c_l)
                        lbls.append(f"fm3[{z}:{tag_mid}]:{n_l}<={n_u}")

            _add_fm_sub(ls_t, us_t, "s")
            _add_fm_sub(lk_t, uk_t, "k")
            _add_fm_sub(lk_t, us_t, "k<=s")

        for i, cname in enumerate(sub_classes):
            r = [Fraction(0)] * (2 * nu)
            r[nu + i] = Fraction(1)
            r[i] = Fraction(-1)
            as_mat.append(r)
            bs_vec.append(Fraction(0))
            lbls.append(f"{cname}.u_k<=u_s")

            r = [Fraction(0)] * (2 * nu)
            r[nu + i] = Fraction(-1)
            as_mat.append(r)
            bs_vec.append(Fraction(0))
            lbls.append(f"{cname}.u_k>=0")

        if later_classes:
            dedup_s: dict[tuple[Fraction, ...], tuple[Fraction, str]] = {}
            for r, bv, lb in zip(as_mat, bs_vec, lbls):
                rk = tuple(r)
                cur = dedup_s.get(rk)
                if cur is None or bv < cur[0]:
                    dedup_s[rk] = (bv, lb)
            as_mat = [list(rk) for rk in dedup_s]
            bs_vec = [v[0] for v in dedup_s.values()]
            lbls = [v[1] for v in dedup_s.values()]

        xs, acts = _solve_exact_kkt_qp(hs, gs, as_mat, bs_vec)
        for i, cname in enumerate(sub_classes):
            us_exact[cname] = xs[i]
            uk_exact[cname] = xs[nu + i]
        return [lbls[i] for i in acts]

    act3a_lbls = _solve_zero_weight_substage(stage3a_classes, stage3b_classes + stage3c_classes)
    act3b_lbls = _solve_zero_weight_substage(stage3b_classes, stage3c_classes)
    act3c_lbls = _solve_zero_weight_substage(stage3c_classes, [])
    act3_lbls = act3a_lbls + act3b_lbls + act3c_lbls

    # Exact rational verification of EVERY band and 0 <= u_k <= u_s before 1e-6 quantization.
    for c in task_classes:
        if not (Fraction(0) <= uk_exact[c] <= us_exact[c]):
            raise RuntimeError(f"Exact unary ordering violation on {c}: u_k={uk_exact[c]} > u_s={us_exact[c]}")
    for p in pairs_list:
        ca, cb = p["a"], p["b"]
        es = Fraction(1, 2) * (us_exact[ca] + us_exact[cb])
        ek = Fraction(1, 2) * (uk_exact[ca] + uk_exact[cb])
        slo, shi = _f(p["mu_static"]["range"][0]), _f(p["mu_static"]["range"][1])
        klo, khi = _f(p["mu_kinetic"]["range"][0]), _f(p["mu_kinetic"]["range"][1])
        if not (slo <= es <= shi and klo <= ek <= khi):
            raise RuntimeError(f"Exact pair {p['key']} outside band [{slo},{shi}]/[{klo},{khi}]")

    u_static = {c: _q_frac(us_exact[c], 6) for c in task_classes}
    u_kinetic = {c: _q_frac(uk_exact[c], 6) for c in task_classes}

    # Verify quantized 6-decimal float values satisfy all u_k <= u_s, all class-pair bands, and all override bands.
    for c in task_classes:
        if not (0.0 <= u_kinetic[c] <= u_static[c]):
            raise RuntimeError(f"Quantized unary inversion on {c}: u_k={u_kinetic[c]} > u_s={u_static[c]}")

    counted_stage1_sorted = sorted(counted_stage1_keys)
    counted_stage2_sorted = sorted(counted_stage2_keys)
    counted_all_sorted = sorted(counted_stage1_sorted + counted_stage2_sorted)
    counted_all_set = set(counted_all_sorted)

    per_pair_tasks_ev = (census_weights or {}).get("per_pair_tasks") or {}
    mean_imp_ev = (census_weights or {}).get("mean_3task_object_impulse_share") or {}

    def _row_census_evidence(
        pk: str,
        scen_mem: str,
        can_form_any: bool,
        can_form_lb: bool,
        is_counted: bool,
    ) -> dict[str, Any]:
        if census_weights is None:
            return {"weighted": False}
        is_sens = scen_mem == "sensitivity"
        lookup_pk = _sens_counterpart_key(pk) if is_sens else pk
        by_task = per_pair_tasks_ev.get(lookup_pk) or {}
        loaded_tasks = [t for t, ev in by_task.items() if ev.get("object_contact_loaded")]
        mean_share = float(mean_imp_ev.get(lookup_pk, 0.0))
        if not can_form_any:
            status = "not_formable"
        elif not can_form_lb:
            status = "excluded_finger_on_finger"
        elif is_counted:
            status = "sensitivity_counterpart_loaded" if is_sens else "loaded_object_contact"
        elif by_task and not loaded_tasks:
            status = "sensitivity_counterpart_robot_self_only" if is_sens else "excluded_robot_link_self_contact"
        else:
            status = "sensitivity_counterpart_unloaded" if is_sens else "unloaded_in_censuses"
        out_ev: dict[str, Any] = {
            "status": status,
            "loaded_in_tasks": loaded_tasks,
            "mean_3task_impulse_share": mean_share,
            "by_task": by_task,
        }
        if is_sens:
            out_ev["counterpart_primary_pair"] = lookup_pk
        return out_ev

    class_pairs_table: dict[str, dict[str, Any]] = {}
    for p in pairs_list:
        pk = p["key"]
        ca, cb = p["a"], p["b"]
        can_form_lb, reason = classify_pair(("class", ca), ("class", cb))
        ftasks = formable_tasks(("class", ca), ("class", cb))
        can_form_any = bool(ftasks)
        is_counted = pk in counted_all_set
        s_nom = float(p["mu_static"]["value"])
        k_nom = float(p["mu_kinetic"]["value"])
        s_lo, s_hi = float(p["mu_static"]["range"][0]), float(p["mu_static"]["range"][1])
        k_lo, k_hi = float(p["mu_kinetic"]["range"][0]), float(p["mu_kinetic"]["range"][1])
        eff_s = _q((_d(u_static[ca]) + _d(u_static[cb])) * Decimal("0.5"), 6)
        eff_k = _q((_d(u_kinetic[ca]) + _d(u_kinetic[cb])) * Decimal("0.5"), 6)
        if not (s_lo <= eff_s <= s_hi) or not (k_lo <= eff_k <= k_hi):
            raise RuntimeError(f"Quantized unary pair {pk} ({eff_s}/{eff_k}) outside band [{s_lo},{s_hi}]/[{k_lo},{k_hi}]")
        ds = _q(_d(eff_s) - _d(s_nom), 6)
        dk = _q(_d(eff_k) - _d(k_nom), 6)
        row_cp: dict[str, Any] = {
            "a": ca,
            "b": cb,
            "scenario_membership": p["scenario_membership"],
            "can_form_in_task": can_form_any,
            "formable_in_tasks": ftasks,
            "counted_in_fit_objective": is_counted,
            "weight_in_fit_objective": 1 if is_counted else 0,
            "formability": reason,
            "census_evidence": _row_census_evidence(pk, p["scenario_membership"], can_form_any, can_form_lb, is_counted),
            "friction_combine_mode": "average",
            "mu_static": {
                "nominal": s_nom,
                "realised": eff_s,
                "delta": ds,
                "band": [s_lo, s_hi],
                "in_band": True,
            },
            "mu_kinetic": {
                "nominal": k_nom,
                "realised": eff_k,
                "delta": dk,
                "band": [k_lo, k_hi],
                "in_band": True,
            },
        }
        if is_counted and (abs(_d(ds)) >= KNOWN_COST_ABS_DELTA or abs(_d(dk)) >= KNOWN_COST_ABS_DELTA):
            row_cp["known_cost_note"] = _known_cost_note(pk, s_nom, k_nom, eff_s, eff_k)
        class_pairs_table[pk] = row_cp

    override_pairs_table: dict[str, dict[str, Any]] = {}
    excluded_self_overrides: dict[str, str] = {}
    for aid, adict in sorted(out_assets.items()):
        if "material_override" not in adict:
            continue
        mo = adict["material_override"]
        acls = str(adict["material_class"])
        no_self = "no_self_contact" in (adict.get("flags") or ())
        os_f, ok_f = float(mo["static_friction"]), float(mo["dynamic_friction"])
        for partner in task_classes:
            if partner == acls and no_self:
                excluded_self_overrides[f"{aid}__{aid}"] = (
                    "no_self_contact: scored_layouts_with_two_or_more_00022 == 0"
                )
                continue
            pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
            if pb is None:
                continue
            opk = f"{aid}__{partner}"
            can_form_lb, reason = classify_pair(("override", aid), ("class", partner))
            ftasks = formable_tasks(("override", aid), ("class", partner))
            can_form_any = bool(ftasks)
            is_counted = opk in counted_all_set
            s_nom = float(pb["mu_static"]["value"])
            k_nom = float(pb["mu_kinetic"]["value"])
            s_lo, s_hi = float(pb["mu_static"]["range"][0]), float(pb["mu_static"]["range"][1])
            k_lo, k_hi = float(pb["mu_kinetic"]["range"][0]), float(pb["mu_kinetic"]["range"][1])
            eff_s = _q((_d(os_f) + _d(u_static[partner])) * Decimal("0.5"), 6)
            eff_k = _q((_d(ok_f) + _d(u_kinetic[partner])) * Decimal("0.5"), 6)
            if not (s_lo <= eff_s <= s_hi) or not (k_lo <= eff_k <= k_hi):
                raise RuntimeError(
                    f"Quantized override pair {aid}__{partner} ({eff_s}/{eff_k}) outside band [{s_lo},{s_hi}]/[{k_lo},{k_hi}]"
                )
            override_pairs_table[opk] = {
                "asset_id": aid,
                "asset_class": acls,
                "partner_class": partner,
                "band_pair_key": pb["key"],
                "scenario_membership": pb["scenario_membership"],
                "can_form_in_task": can_form_any,
                "formable_in_tasks": ftasks,
                "counted_in_fit_objective": is_counted,
                "weight_in_fit_objective": 1 if is_counted else 0,
                "formability": reason,
                "census_evidence": _row_census_evidence(
                    opk, pb["scenario_membership"], can_form_any, can_form_lb, is_counted
                ),
                "friction_combine_mode": "average",
                "mu_static": {
                    "class_pair_nominal": s_nom,
                    "realised": eff_s,
                    "delta_vs_class_nominal": _q(_d(eff_s) - _d(s_nom), 6),
                    "band": [s_lo, s_hi],
                    "in_band": True,
                },
                "mu_kinetic": {
                    "class_pair_nominal": k_nom,
                    "realised": eff_k,
                    "delta_vs_class_nominal": _q(_d(eff_k) - _d(k_nom), 6),
                    "band": [k_lo, k_hi],
                    "in_band": True,
                },
            }

    def _pair_deltas(pk: str) -> tuple[float, float]:
        if pk in class_pairs_table:
            r = class_pairs_table[pk]
            return float(r["mu_static"]["delta"]), float(r["mu_kinetic"]["delta"])
        r = override_pairs_table[pk]
        return float(r["mu_static"]["delta_vs_class_nominal"]), float(r["mu_kinetic"]["delta_vs_class_nominal"])

    def _summarize_subset(subset_keys: list[str]) -> dict[str, Any]:
        pairs_d = [_pair_deltas(k) for k in subset_keys]
        ds_list = [d[0] for d in pairs_d]
        dk_list = [d[1] for d in pairs_d]
        cnt = max(len(subset_keys), 1)
        sse_s = sum(v * v for v in ds_list)
        sse_k = sum(v * v for v in dk_list)
        return {
            "pair_count": len(subset_keys),
            "static_l1_sum": round(sum(abs(v) for v in ds_list), 6),
            "kinetic_l1_sum": round(sum(abs(v) for v in dk_list), 6),
            "static_l2_sse": round(sse_s, 6),
            "kinetic_l2_sse": round(sse_k, 6),
            "joint_l2_sse": round(sse_s + sse_k, 6),
            "static_rmse": round(math.sqrt(sse_s / cnt), 6),
            "kinetic_rmse": round(math.sqrt(sse_k / cnt), 6),
            "static_min_delta": round(min(ds_list), 6) if ds_list else 0.0,
            "static_max_delta": round(max(ds_list), 6) if ds_list else 0.0,
            "kinetic_min_delta": round(min(dk_list), 6) if dk_list else 0.0,
            "kinetic_max_delta": round(max(dk_list), 6) if dk_list else 0.0,
        }

    formable_lb_prim_sorted = sorted(formable_lb_prim_keys)
    unloaded_prim_sorted = sorted(set(formable_lb_prim_sorted) - set(counted_stage1_sorted))
    prim_class_keys = [p["key"] for p in pairs_list if p["scenario_membership"] != "sensitivity"]
    all_class_keys = [p["key"] for p in pairs_list]

    summary_stats = {
        "counted_primary_formable_pairs": _summarize_subset(counted_stage1_sorted),
        "unloaded_primary_formable_pairs": _summarize_subset(unloaded_prim_sorted),
        "all_load_bearing_formable_primary_pairs": _summarize_subset(formable_lb_prim_sorted),
        "counted_all_formable_pairs": _summarize_subset(counted_all_sorted),
        "primary_scenario_pairs": _summarize_subset(prim_class_keys),
        "all_task_pairs": _summarize_subset(all_class_keys),
    }

    later_gate_txt = (
        "Lane 3 / future G2 censuses (re-fit and re-check done tasks if a later scripted-solution census adds a loaded "
        "pair on these classes; #59)"
    )
    known_costs: dict[str, Any] = {}
    for pk in sorted(class_pairs_table):
        row_kc = class_pairs_table[pk]
        if "known_cost_note" not in row_kc:
            continue
        known_costs[pk] = {
            "nominal_mu_static": row_kc["mu_static"]["nominal"],
            "realised_mu_static": row_kc["realised_mu_static"] if "realised_mu_static" in row_kc else row_kc["mu_static"]["realised"],
            "delta_mu_static": row_kc["mu_static"]["delta"],
            "nominal_mu_kinetic": row_kc["mu_kinetic"]["nominal"],
            "realised_mu_kinetic": row_kc["mu_kinetic"]["realised"],
            "delta_mu_kinetic": row_kc["mu_kinetic"]["delta"],
            "formable_in_tasks": row_kc["formable_in_tasks"],
            "loaded_in_tasks": row_kc["census_evidence"].get("loaded_in_tasks", []),
            "note": row_kc["known_cost_note"],
            "later_fix_gate": later_gate_txt,
        }
    for opk in sorted(override_pairs_table):
        orow_kc = override_pairs_table[opk]
        if not orow_kc["counted_in_fit_objective"]:
            continue
        ods = _d(orow_kc["mu_static"]["delta_vs_class_nominal"])
        odk = _d(orow_kc["mu_kinetic"]["delta_vs_class_nominal"])
        if abs(ods) >= KNOWN_COST_ABS_DELTA or abs(odk) >= KNOWN_COST_ABS_DELTA:
            known_costs[opk] = {
                "nominal_mu_static": orow_kc["mu_static"]["class_pair_nominal"],
                "realised_mu_static": orow_kc["mu_static"]["realised"],
                "delta_mu_static": orow_kc["mu_static"]["delta_vs_class_nominal"],
                "nominal_mu_kinetic": orow_kc["mu_kinetic"]["class_pair_nominal"],
                "realised_mu_kinetic": orow_kc["mu_kinetic"]["realised"],
                "delta_mu_kinetic": orow_kc["mu_kinetic"]["delta_vs_class_nominal"],
                "formable_in_tasks": orow_kc["formable_in_tasks"],
                "loaded_in_tasks": orow_kc["census_evidence"].get("loaded_in_tasks", []),
                "note": _known_cost_note(
                    opk,
                    float(orow_kc["mu_static"]["class_pair_nominal"]),
                    float(orow_kc["mu_kinetic"]["class_pair_nominal"]),
                    float(orow_kc["mu_static"]["realised"]),
                    float(orow_kc["mu_kinetic"]["realised"]),
                ),
                "later_fix_gate": later_gate_txt,
            }

    # Pairs that can form only finger on finger (#167 (b)): band constraints only, recorded with their values.
    not_load_bearing: dict[str, Any] = {}
    excluded_by_census: dict[str, Any] = {}
    for pk_nl, row_nl in sorted(class_pairs_table.items()):
        if not row_nl["can_form_in_task"]:
            continue
        can_lb_nl, _ = classify_pair(("class", row_nl["a"]), ("class", row_nl["b"]))
        if not can_lb_nl:
            not_load_bearing[pk_nl] = {
                "reason": (
                    "Forms only finger on finger (an empty close, or the grippers of two arms meeting in a handover); "
                    "such a contact never carries a sliding load these tasks depend on, so the pair is a band "
                    "constraint only (#167, Bo's side's decision (b))."
                ),
                "formable_in_tasks": row_nl["formable_in_tasks"],
                "scenario_membership": row_nl["scenario_membership"],
                "nominal_mu_static": row_nl["mu_static"]["nominal"],
                "realised_mu_static": row_nl["mu_static"]["realised"],
                "nominal_mu_kinetic": row_nl["mu_kinetic"]["nominal"],
                "realised_mu_kinetic": row_nl["mu_kinetic"]["realised"],
            }
        elif not row_nl["counted_in_fit_objective"]:
            excluded_by_census[pk_nl] = {
                "status": row_nl["census_evidence"]["status"],
                "formable_in_tasks": row_nl["formable_in_tasks"],
                "scenario_membership": row_nl["scenario_membership"],
                "by_task": row_nl["census_evidence"].get("by_task", {}),
                "nominal_mu_static": row_nl["mu_static"]["nominal"],
                "realised_mu_static": row_nl["mu_static"]["realised"],
                "nominal_mu_kinetic": row_nl["mu_kinetic"]["nominal"],
                "realised_mu_kinetic": row_nl["mu_kinetic"]["realised"],
            }
    for opk_nl, orow_nl in sorted(override_pairs_table.items()):
        if orow_nl["can_form_in_task"] and not orow_nl["counted_in_fit_objective"]:
            excluded_by_census[opk_nl] = {
                "status": orow_nl["census_evidence"]["status"],
                "formable_in_tasks": orow_nl["formable_in_tasks"],
                "scenario_membership": orow_nl["scenario_membership"],
                "by_task": orow_nl["census_evidence"].get("by_task", {}),
                "nominal_mu_static": orow_nl["mu_static"]["class_pair_nominal"],
                "realised_mu_static": orow_nl["mu_static"]["realised"],
                "nominal_mu_kinetic": orow_nl["mu_kinetic"]["class_pair_nominal"],
                "realised_mu_kinetic": orow_nl["mu_kinetic"]["realised"],
            }

    n_cls1 = sum(1 for k in counted_stage1_sorted if k in class_pairs_table)
    n_cls2 = sum(1 for k in counted_stage2_sorted if k in class_pairs_table)
    n_ovr_bands = len(override_pairs_table)
    is_presence = pw_map is not None
    fit_meta: dict[str, Any] = {
        "variant": (
            "A_global_presence_weighted"
            if is_presence
            else ("A_global_formable_pairs" if len(formability.get("task_names") or []) > 1 else "A_task_formable_pairs")
        ),
        "map_version": (
            "v2_presence"
            if is_presence
            else ("v2_global" if len(formability.get("task_names") or []) > 1 else "v1_bottles")
        ),
        "tasks_in_scope": list(formability.get("task_names") or [formability["task_name"]]),
        "solver": "exact_rational_active_set_kkt_qp",
        "arithmetic": "stdlib fractions.Fraction exact KKT solve + Decimal 1e-6 ROUND_HALF_EVEN quantization",
        "pair_formability_rule": formability["rule"],
        "pair_weight_rule": PAIR_WEIGHT_RULE if is_presence else "equal weight 1.0 over all load-bearing formable pairs",
        "objective": (
            "sum_{p in counted_primary_pairs} [(0.5*(u_s[a]+u_s[b]) - nom_s[p])^2 + "
            "(0.5*(u_k[a]+u_k[b]) - nom_k[p])^2], where an asset with material_override substitutes its fixed override unary"
        ),
        "weights": (
            f"lexicographic Stage 3a exact feasible solve for censused-task zero-weight classes {stage3a_classes} "
            f"({len(act3a_lbls)} active constraints, with exact 1-hop FM cross-band projection from uncensused classes), "
            f"Stage 3b exact feasible solve for T1 uncensused classes {stage3b_classes} ({len(act3b_lbls)} active constraints, "
            f"with exact 1-hop FM projection from T2 classes) holding Stages 1-3a fixed, and Stage 3c exact feasible solve "
            f"for T2 uncensused classes {stage3c_classes} ({len(act3c_lbls)} active constraints) holding Stages 1-3b fixed"
            if (is_presence and stage3c_classes)
            else (
                f"presence weight 1.0 over all {len(counted_stage1_sorted)} object-loaded primary pairs across the committed "
                f"corrected contact censuses of {census_weights['censused_tasks']} ({n_cls1} class pairs + "
                f"{len(counted_stage1_sorted) - n_cls1} asset-override pairs; robot-link x robot-link contacts such as "
                f"{sorted(census_weights['excluded_robot_self_only_pairs'])} and finger-on-finger {sorted(not_load_bearing)} "
                f"excluded per #167 (b) / #59; {len(unloaded_prim_sorted)} unloaded/robot-self formable primary pairs are "
                f"band constraints only); lexicographic Stage 2 presence weight 1.0 over {len(counted_stage2_sorted)} "
                f"counterpart-loaded sensitivity pairs ({n_cls2} class + {len(counted_stage2_sorted) - n_cls2} override) "
                f"for {stage2_pos} with Stage 1 classes {stage1_classes} held fixed ({len(act2)} active constraints); "
                f"lexicographic Stage 3a exact feasible solve for censused-task zero-weight classes {stage3a_classes} "
                f"({len(act3a_lbls)} active constraints, with exact 1-hop FM cross-band projection from uncensused classes) and "
                f"Stage 3b exact feasible solve for uncensused-only classes {stage3b_classes} ({len(act3b_lbls)} active constraints) "
                f"holding Stages 1-3a fixed"
                if is_presence
                else (
                    f"equal weight 1.0 over all {len(counted_stage1_sorted)} load-bearing formable primary pairs"
                )
            )
        ),
        "combine_rule": "average (PhysX default; 0.5 * (u_a + u_b) on every class)",
        "constraints": (
            f"all {len(class_pairs_table)} class-pair static/kinetic bands (including exact 1-hop Fourier-Motzkin cross-band "
            f"projection of zero-weight classes onto Stage 1 and uncensused classes onto Stage 3a) + all {n_ovr_bands} "
            f"asset-override bands + 0 <= u_k <= u_s per class"
        ),
        "carried_primary_classes": carried_prim_classes,
        "carried_sensitivity_classes": carried_sens_classes,
        "uncarried_classes_band_only": uncarried_classes,
        "stage1_classes": stage1_classes,
        "stage2_classes": stage2_pos,
        "stage3_zero_weight_classes": stage3_classes,
        "stage3_censused_zero_weight_classes": stage3a_classes,
        "stage3_uncensused_classes": stage3b_classes + stage3c_classes,
        **({"stage3b_t1_uncensused_classes": stage3b_classes, "stage3c_t2_uncensused_classes": stage3c_classes} if stage3c_classes else {}),
        "counted_primary_pairs": counted_stage1_sorted,
        "counted_sensitivity_pairs": counted_stage2_sorted,
        "unloaded_primary_formable_pairs": unloaded_prim_sorted,
        "excluded_non_formable_pairs": {k: excluded_non_formable[k] for k in sorted(excluded_non_formable)},
        "dropped_not_load_bearing_pairs": not_load_bearing,
        "excluded_by_census_pairs": excluded_by_census,
        "census_summary": (census_weights or {}).get("task_summaries") or {},
        "kkt_verified_exact": True,
        "active_constraints_stage1": [lbl1[i] for i in act1],
        "active_constraints_stage2": [lbl2[i] for i in act2],
        "active_constraints_stage3": act3_lbls,
        "active_constraints_stage3a": act3a_lbls,
        "active_constraints_stage3b": act3b_lbls,
        **({"active_constraints_stage3c": act3c_lbls} if stage3c_classes else {}),
        "exact_rational_static": {c: f"{us_exact[c].numerator}/{us_exact[c].denominator}" for c in task_classes},
        "exact_rational_kinetic": {c: f"{uk_exact[c].numerator}/{uk_exact[c].denominator}" for c in task_classes},
        "known_costs": known_costs,
        "summary": summary_stats,
    }

    realised_vs_nominal: dict[str, Any] = {
        "fill": "empty",
        "combine_rule": "average",
        "fit_variant": fit_meta["variant"],
        "map_version": fit_meta["map_version"],
        "pair_formability": {
            "task_name": formability["task_name"],
            "tasks_in_scope": fit_meta["tasks_in_scope"],
            "censused_tasks": list((census_weights or {}).get("censused_tasks") or []),
            "rule": formability["rule"],
            "weight_rule": fit_meta["pair_weight_rule"],
            "formable_load_bearing_primary_pair_count": len(formable_lb_prim_sorted),
            "counted_primary_pair_count": len(counted_stage1_sorted),
            "counted_sensitivity_pair_count": len(counted_stage2_sorted),
            "counted_primary_pairs": counted_stage1_sorted,
            "counted_sensitivity_pairs": counted_stage2_sorted,
            "unloaded_primary_formable_pairs": unloaded_prim_sorted,
            "excluded_non_formable_pairs": {k: excluded_non_formable[k] for k in sorted(excluded_non_formable)},
            "dropped_not_load_bearing_pairs": not_load_bearing,
            "excluded_by_census_pairs": excluded_by_census,
        },
        "known_costs": known_costs,
        "summary": summary_stats,
        "class_pairs": class_pairs_table,
        "asset_override_pairs": override_pairs_table,
        "excluded_self_override_pairs": excluded_self_overrides,
    }
    return u_static, u_kinetic, fit_meta, realised_vs_nominal


def _known_cost_note(pk: str, s_nom: float, k_nom: float, eff_s: float, eff_k: float) -> str:
    return (
        f"Known cost of the global presence-weighted average-combine fit (#59, #167): {pk} realises "
        f"{eff_s:.6f}/{eff_k:.6f} against nominal {s_nom:.3f}/{k_nom:.3f} (static/kinetic), a deviation of at least "
        f"{KNOWN_COST_ABS_DELTA} in one of them while staying inside its sourced/estimated band."
    )


def _attach_restitution_to_realised_table(
    realised_vs_nominal: dict[str, Any],
    out_classes: dict[str, Any],
    out_pairs: dict[str, Any],
    out_assets: dict[str, Any],
) -> None:
    """Populate empty-fill (and full-fill reference) combined restitution entries in ``realised_vs_nominal``."""
    c_rest_emp = {c: float(out_classes[c]["restitution"]["value"]) for c in out_classes}
    c_rest_ful = {
        c: float(((out_classes[c]["restitution"].get("fill_overrides") or {}).get("full") or {}).get("value", c_rest_emp[c]))
        for c in out_classes
    }
    for pk, row in realised_vs_nominal["class_pairs"].items():
        pdict = out_pairs[pk]
        if "restitution" not in pdict:
            continue
        pr = pdict["restitution"]
        ca, cb = row["a"], row["b"]
        r_nom = float(pr["value"])
        r_lo, r_hi = float(pr["range"][0]), float(pr["range"][1])
        eff_r = _q((_d(c_rest_emp[ca]) + _d(c_rest_emp[cb])) * Decimal("0.5"), 6)
        r_entry: dict[str, Any] = {
            "combine_mode": "average",
            "nominal": r_nom,
            "realised": eff_r,
            "delta": _q(_d(eff_r) - _d(r_nom), 6),
            "band": [r_lo, r_hi],
            "check_active": bool(pr.get("impact_speed_above_0.5_m_s", False)),
            "in_band": bool(r_lo - 1e-6 <= eff_r <= r_hi + 1e-6),
        }
        fo_full = (pr.get("fill_overrides") or {}).get("full")
        if isinstance(fo_full, dict):
            # Note: Geometry/dustbin/00000 is open_receptacle so polyolefin_plastic stays at empty COR 0.25 unless sealed.
            ra_ful = c_rest_emp[ca] if ca == "polyolefin_plastic" else c_rest_ful[ca]
            rb_ful = c_rest_emp[cb] if cb == "polyolefin_plastic" else c_rest_ful[cb]
            eff_rf = _q((_d(ra_ful) + _d(rb_ful)) * Decimal("0.5"), 6)
            f_nom = float(fo_full["value"])
            f_lo, f_hi = float(fo_full["range"][0]), float(fo_full["range"][1])
            r_entry["full_fill"] = {
                "nominal": f_nom,
                "realised": eff_rf,
                "delta": _q(_d(eff_rf) - _d(f_nom), 6),
                "band": [f_lo, f_hi],
                "in_band": bool(f_lo - 1e-6 <= eff_rf <= f_hi + 1e-6),
            }
        row["restitution"] = r_entry

    for opk, orow in realised_vs_nominal["asset_override_pairs"].items():
        aid = orow["asset_id"]
        partner = orow["partner_class"]
        bpk = orow["band_pair_key"]
        pdict = out_pairs[bpk]
        if "restitution" not in pdict:
            continue
        pr = pdict["restitution"]
        mo = out_assets[aid]["material_override"]
        r_emp_a = float(mo["restitution_by_fill"]["empty"]["value"])
        eff_r = _q((_d(r_emp_a) + _d(c_rest_emp[partner])) * Decimal("0.5"), 6)
        r_nom = float(pr["value"])
        r_lo, r_hi = float(pr["range"][0]), float(pr["range"][1])
        orow["restitution"] = {
            "combine_mode": "average",
            "class_pair_nominal": r_nom,
            "realised": eff_r,
            "delta_vs_class_nominal": _q(_d(eff_r) - _d(r_nom), 6),
            "band": [r_lo, r_hi],
            "check_active": bool(pr.get("impact_speed_above_0.5_m_s", False)),
            "in_band": bool(r_lo - 1e-6 <= eff_r <= r_hi + 1e-6),
        }


def _clean_band(entry: dict[str, Any], *, downgrade_wood_1405: bool = False) -> dict[str, Any]:
    out: dict[str, Any] = {}
    extra_notes: list[str] = []
    for k, v in entry.items():
        if k == "fill_overrides" and isinstance(v, dict):
            out["fill_overrides"] = {
                fk: _clean_band(fv, downgrade_wood_1405=downgrade_wood_1405)
                for fk, fv in v.items()
                if isinstance(fv, dict)
            }
        elif k in ALLOWED_BAND_KEYS:
            out[k] = v
        else:
            extra_notes.append(f"{k}={json.dumps(v, sort_keys=True)}")

    # #88/#91 Note 3 + Rule 4: arXiv:1405.3049 tested wood particles dropped onto methacrylate (tagged
    # conditions_mismatch on engineered_wood); never admit it as 'estimated' for rigid body restitution.
    if downgrade_wood_1405:
        srcs_str = json.dumps(out.get("sources") or [])
        if "1405.3049" in srcs_str and out.get("status") == "estimated":
            for sf in ("status", "value_status", "range_status"):
                if sf in out:
                    out[sf] = "unsourced"
            extra_notes.append(
                "downgraded estimated->unsourced (#91 note 3: arXiv:1405.3049 wood particles on methacrylate is conditions_mismatch)"
            )

    if extra_notes:
        prev = str(out.get("notes") or "").strip()
        joined = "; ".join(extra_notes)
        out["notes"] = f"{prev}; {joined}" if prev else joined
    return out


def _composite_full_moments(
    m_empty: Decimal,
    m_full: Decimal,
    com_sh: list[float],
    iom_sh: list[float],
    com_so: list[float],
    iom_so: list[float],
) -> tuple[list[float], list[float]]:
    """Exact parallel-axis composite COM and diagonal I/m for a thin shell (m_empty) + solid interior liquid (m_full - m_empty)."""
    w_sh = m_empty / m_full
    w_so = Decimal(1) - w_sh
    csh = [_d(x) for x in com_sh]
    cso = [_d(x) for x in com_so]
    ish = [_d(x) for x in iom_sh]
    iso = [_d(x) for x in iom_so]

    c_full = [w_sh * csh[k] + w_so * cso[k] for k in range(3)]
    d_sh = [csh[k] - c_full[k] for k in range(3)]
    d_so = [cso[k] - c_full[k] for k in range(3)]
    norm2_sh = sum(x * x for x in d_sh)
    norm2_so = sum(x * x for x in d_so)

    iom_full = [
        w_sh * (ish[k] + norm2_sh - d_sh[k] * d_sh[k]) + w_so * (iso[k] + norm2_so - d_so[k] * d_so[k])
        for k in range(3)
    ]
    return [_q(c_full[k], 6) for k in range(3)], [_q(iom_full[k], 8) for k in range(3)]


def _subset_leaf(entry: dict[str, Any], keys: tuple[str, ...], *, notes: str | None = None) -> dict[str, Any]:
    out = {k: entry[k] for k in keys if k in entry}
    if notes:
        out["notes"] = notes
    return out


def _solid_rigid_asset(aid: str, sa: dict[str, Any], u_row: dict[str, Any]) -> dict[str, Any]:
    """Asset row for a solid or open Rigid object from a task slice (mass_kg; COM and I/m identical for both fills)."""
    if list(sa["size_m"]["value"]) != list(u_row["collider_union_extents_m"]):
        raise RuntimeError(f"{aid}: slice size {sa['size_m']['value']} != USD collider extents {u_row['collider_union_extents_m']}")
    geo_iom = [list(u_row["visual_surface"]["enclosed_solid_inertia_over_mass_m2"])]
    if u_row.get("box_primitive_colliders"):
        geo_iom.append(list(u_row["box_primitive_colliders"]["union_solid_inertia_over_mass_m2"]))
    if list(sa["solid_collider_inertia_over_mass_m2"]["value"]) not in geo_iom:
        raise RuntimeError(f"{aid}: slice solid-collider I/m does not match the pinned USD facts")
    flags = list(sa["flags"])
    if "open_receptacle" in flags and "hollow_contents_dependent" not in flags:
        flags.append("hollow_contents_dependent")
    leaf_keys = ("value", "unit", "status", "basis", "sources")
    fill_note = "Solid or open object: fill does not apply, so the empty and full entries are identical."
    mk = sa["mass_kg"]
    sens = mk.get("sensitivity_by_class") or {}
    mass_notes = (
        "Class sensitivities (G4 cells outside docs/params): "
        + "; ".join(f"{c} {json.dumps(v, sort_keys=True)}" for c, v in sorted(sens.items()))
        if sens
        else None
    )
    row: dict[str, Any] = {
        "id": aid,
        "type": sa["type"],
        "category": sa["category"],
        "material_class": sa["material_class"],
        "alternative_material_classes": list(sa["alternative_material_classes"]),
        "confidence": sa["confidence"],
        "identity": sa["identity"],
        "flags": flags,
        "size_m": _subset_leaf(sa["size_m"], ("value", "unit", "status", "sources")),
        "mass_kg": _subset_leaf(mk, ("value", "range", "unit", "status", "basis", "sources"), notes=mass_notes),
        "inertia_over_mass_by_fill": {
            f: _subset_leaf(sa["inertia_over_mass_m2"], leaf_keys, notes=fill_note) for f in ("empty", "full")
        },
        "com_offset_by_fill": {f: _subset_leaf(sa["com_offset_m"], leaf_keys, notes=fill_note) for f in ("empty", "full")},
        "solid_collider_inertia_over_mass_m2": _subset_leaf(sa["solid_collider_inertia_over_mass_m2"], leaf_keys),
    }
    dec = sa.get("class_decision")
    if dec:
        row["notes"] = (
            f"{dec['decision']} Named sensitivity classes: {sa.get('named_sensitivity_classes', [])}. "
            f"{dec['why_wood_over_abs']} {dec['class_for_painted_wood']}"
        )
    return row


def _t1_asset_row(aid: str, tr: dict[str, Any], u_t1: dict[str, Any]) -> dict[str, Any]:
    """Convert a verified row from audit/out/63/population/t1_rows.json into D2 assets.json schema."""
    is_geom = tr["type"] == "Geometry"
    u_row = u_t1["geometry_objects"][aid] if is_geom else u_t1["rigid_objects"][aid]
    exp_ext = u_row["collider_union_extents_m"] if u_row["collider_union_extents_m"] is not None else u_row["metadata_aligned_bbox_extents_m"]
    if list(tr["size_m"]["value"]) != list(exp_ext):
        raise RuntimeError(f"{aid}: t1_rows size_m {tr['size_m']['value']} != USD extents {exp_ext}")

    leaf_keys = ("value", "unit", "status", "basis", "form_limit", "sources")
    if is_geom:
        zero_col = bool((tr.get("mass") or {}).get("visual_only_no_collider"))
        if zero_col != (int(u_row["enabled_gprim_collider_count"]) == 0):
            raise RuntimeError(f"{aid}: visual_only_no_collider mismatch vs USD facts")
        row_g: dict[str, Any] = {
            "id": aid,
            "type": "Geometry",
            "category": tr["category"],
            "material_class": tr["material_class"],
            "alternative_material_classes": list(tr["alternative_material_classes"]),
            "confidence": tr["confidence"],
            "identity": tr["identity"],
            "flags": list(tr["flags"]),
            "size_decision": tr.get("size_decision", "asset"),
            "size_m": _subset_leaf(tr["size_m"], ("value", "unit", "status", "basis", "sources")),
        }
        if zero_col:
            row_g["collision"] = {
                "enabled": False,
                "approximation": "asset",
                "status": "sourced",
                "notes": "Zero-collider visual marker in pinned USD (0 enabled Gprim colliders; excluded from contact pairs).",
                "sources": list(tr["size_m"]["sources"][:1]),
            }
        return row_g

    fill_note = "Solid, hollow-housing, or open-mesh T1 object (#175): fill does not apply, so empty and full entries are identical."
    notes_parts: list[str] = []
    if tr.get("class_override"):
        co = tr["class_override"]
        notes_parts.append(
            f"Class override ({co['decision']}): {co['from_classes_json']} -> {co['to_material_class']} ({co['reason']})"
        )
    if tr.get("review_flag"):
        rf = tr["review_flag"]
        notes_parts.append(f"Flag [{rf['category']}]: {rf['reason']}")
    row_r: dict[str, Any] = {
        "id": aid,
        "type": "Rigid",
        "category": tr["category"],
        "material_class": tr["material_class"],
        "alternative_material_classes": list(tr["alternative_material_classes"]),
        "confidence": tr["confidence"],
        "identity": tr["identity"],
        "flags": list(tr["flags"]),
        "size_decision": tr.get("size_decision", "asset"),
        "size_m": _subset_leaf(tr["size_m"], ("value", "unit", "status", "basis", "sources")),
        "mass_kg": _subset_leaf(tr["mass_kg"], ("value", "range", "unit", "status", "basis", "sources")),
        "inertia_over_mass_by_fill": {
            f: _subset_leaf(tr["inertia_over_mass_m2"], leaf_keys, notes=fill_note) for f in ("empty", "full")
        },
        "com_offset_by_fill": {
            f: _subset_leaf(tr["com_offset_m"], leaf_keys, notes=fill_note) for f in ("empty", "full")
        },
        "solid_collider_inertia_over_mass_m2": _subset_leaf(tr["solid_collider_inertia_over_mass_m2"], leaf_keys),
    }
    if notes_parts:
        row_r["notes"] = " ".join(notes_parts)
    return row_r


def _t2_asset_row(aid: str, tr: dict[str, Any], u_t2: dict[str, Any], q_tags: dict[str, str]) -> dict[str, Any]:
    """Convert a verified row from audit/out/63/population/t2_rows.json.gz into D2 assets.json schema."""
    is_geom = tr["type"] == "Geometry"
    u_row = u_t2["geometry_objects"][aid] if is_geom else u_t2["rigid_objects"][aid]
    exp_ext = (
        u_row["collider_union_extents_m"]
        if u_row["collider_union_extents_m"] is not None
        else u_row["metadata_aligned_bbox_extents_m"]
    )
    if list(tr["size_m"]["value"]) != list(exp_ext):
        raise RuntimeError(f"{aid}: t2_rows size_m {tr['size_m']['value']} != USD extents {exp_ext}")

    leaf_keys = ("value", "unit", "status", "basis", "form_limit", "sources")
    if is_geom:
        zero_col = bool((tr.get("mass") or {}).get("visual_only_no_collider"))
        if zero_col != (int(u_row["enabled_gprim_collider_count"]) == 0):
            raise RuntimeError(f"{aid}: visual_only_no_collider mismatch vs USD facts")
        row_g: dict[str, Any] = {
            "id": aid,
            "type": "Geometry",
            "category": tr["category"],
            "material_class": tr["material_class"],
            "alternative_material_classes": list(tr["alternative_material_classes"]),
            "confidence": tr["confidence"],
            "identity": tr["identity"],
            "flags": list(tr["flags"]),
            "size_decision": tr.get("size_decision", "asset"),
            "size_m": _subset_leaf(tr["size_m"], ("value", "unit", "status", "basis", "sources")),
        }
        if zero_col:
            row_g["collision"] = {
                "enabled": False,
                "approximation": "asset",
                "status": "sourced",
                "notes": "Zero-collider visual marker in pinned USD (0 enabled Gprim colliders; excluded from contact pairs).",
                "sources": list(tr["size_m"]["sources"][:1]),
            }
        if tr.get("class_override"):
            co_g = tr["class_override"]
            row_g["notes"] = (
                f"Class override ({co_g['decision']}): {co_g['from_classes_json']} -> {co_g['to_material_class']} ({co_g['reason']})"
            )
        return row_g

    notes_parts: list[str] = []
    if tr.get("class_override"):
        co = tr["class_override"]
        notes_parts.append(
            f"Class override ({co['decision']}): {co['from_classes_json']} -> {co['to_material_class']} ({co['reason']})"
        )
    if tr.get("review_flag"):
        rf = tr["review_flag"]
        notes_parts.append(f"Flag [{rf['category']}]: {rf['reason']}")

    row_r: dict[str, Any] = {
        "id": aid,
        "type": "Rigid",
        "category": tr["category"],
        "material_class": tr["material_class"],
        "alternative_material_classes": list(tr["alternative_material_classes"]),
        "confidence": tr["confidence"],
        "identity": tr["identity"],
        "flags": list(tr["flags"]),
        "size_decision": tr.get("size_decision", "asset"),
        "size_m": _subset_leaf(tr["size_m"], ("value", "unit", "status", "basis", "sources")),
    }
    mbf = tr.get("mass_by_fill_draft")
    if isinstance(mbf, dict) and "empty" in mbf and "full" in mbf:
        emp_band = _subset_leaf(mbf["empty"], ("value", "range", "unit", "status", "basis", "sources"))
        ful_band = _subset_leaf(mbf["full"], ("value", "range", "unit", "status", "basis", "sources"))
        com_sh = list(tr["com_offset_m"]["value"])
        iom_sh = list(tr["inertia_over_mass_m2"]["value"])
        com_so = list(tr["solid_collider_com_offset_m"]["value"])
        iom_so = list(tr["solid_collider_inertia_over_mass_m2"]["value"])
        com_ful, iom_ful = _composite_full_moments(
            _d(emp_band["value"]), _d(ful_band["value"]), com_sh, iom_sh, com_so, iom_so
        )
        q2_sfx = f" [{q_tags['Q2']}]" if q_tags.get("Q2") else ""
        usd_src_only = list(tr["com_offset_m"].get("sources") or [])
        row_r["mass_by_fill"] = {"empty": emp_band, "full": ful_band}
        row_r["inertia_over_mass_by_fill"] = {
            "empty": _subset_leaf(tr["inertia_over_mass_m2"], leaf_keys),
            "full": {
                "value": iom_ful,
                "unit": "m^2",
                "status": "estimated",
                "basis": f"Parallel-axis composite of empty container shell plus uniform-density interior contents.{q2_sfx}",
                "form_limit": "Rigid composite body; internal contents motion omitted in rigid-body PhysX (#85).",
                "sources": usd_src_only,
            },
        }
        row_r["com_offset_by_fill"] = {
            "empty": _subset_leaf(tr["com_offset_m"], leaf_keys),
            "full": {
                "value": com_ful,
                "unit": "m",
                "status": "estimated",
                "basis": f"Mass-weighted centroid of empty container shell plus interior contents.{q2_sfx}",
                "form_limit": "Assumes uniform interior fill without headspace shifting (#85).",
                "sources": usd_src_only,
            },
        }
    else:
        fill_note = "Solid, hollow-housing, or open-mesh T2 object (#59, #175): fill does not apply, so empty and full entries are identical."
        row_r["mass_kg"] = _subset_leaf(tr["mass_kg"], ("value", "range", "unit", "status", "basis", "sources"))
        row_r["inertia_over_mass_by_fill"] = {
            f: _subset_leaf(tr["inertia_over_mass_m2"], leaf_keys, notes=fill_note) for f in ("empty", "full")
        }
        row_r["com_offset_by_fill"] = {
            f: _subset_leaf(tr["com_offset_m"], leaf_keys, notes=fill_note) for f in ("empty", "full")
        }
    row_r["solid_collider_inertia_over_mass_m2"] = _subset_leaf(tr["solid_collider_inertia_over_mass_m2"], leaf_keys)
    if notes_parts:
        row_r["notes"] = " ".join(notes_parts)
    return row_r


def verify_all_t2_leaves_have_basis(
    classes_doc: dict[str, Any],
    pairs_doc: dict[str, Any],
    assets_doc: dict[str, Any],
    *,
    t2_classes: set[str],
    t2_pair_keys: set[str],
    t2_new_asset_ids: set[str],
) -> None:
    """Fail generation if any new T2 unsourced or estimated leaf lacks an explicit non-empty basis string."""
    missing: list[str] = []

    def _walk(obj: Any, path: str) -> None:
        if isinstance(obj, dict):
            st_vals = {obj.get(k) for k in ("status", "value_status", "range_status") if k in obj}
            if st_vals & {"unsourced", "estimated"}:
                has_explicit_basis = isinstance(obj.get("basis"), str) and bool(obj["basis"].strip())
                if not has_explicit_basis and not _entry_has_basis_text(obj):
                    missing.append(path)
                elif "unsourced" in st_vals and not has_explicit_basis:
                    missing.append(f"{path} (unsourced missing explicit .basis)")
            for k, v in obj.items():
                _walk(v, f"{path}.{k}" if path else str(k))
        elif isinstance(obj, list):
            for idx, v in enumerate(obj):
                _walk(v, f"{path}[{idx}]")

    for c in sorted(t2_classes):
        _walk(classes_doc["classes"][c], f"classes.{c}")
    for pk in sorted(t2_pair_keys):
        if pk in pairs_doc["pairs"]:
            _walk(pairs_doc["pairs"][pk], f"pairs.{pk}")
    for aid in sorted(t2_new_asset_ids):
        _walk(assets_doc["assets"][aid], f"assets.{aid}")
    if missing:
        raise RuntimeError(f"{len(missing)} T2 unsourced/estimated leaf/leaves lack explicit basis: {missing[:10]}")


def sanitize_wood_restitution_entry(entry: dict[str, Any]) -> dict[str, Any]:
    """Exported helper (tested by unit tests) enforcing #91 Note 3 on wood restitution entries."""
    return _clean_band(dict(entry), downgrade_wood_1405=True)


def build_gripper_block(src: dict[str, Any]) -> dict[str, Any]:
    """D3 (#90) ARX X5 gripper: per-finger drive from arx5-sdk's MIT gains through the phase1.md A.4 transmission.

    One motor drives both fingers through a symmetric rack: jaw width w = q7 + q8 moves J_w = gripper_width /
    gripper_open_readout per motor radian and F_finger = eta * tau / J_w. A symmetric finger displacement dq turns the
    motor by 2 dq / J_w, so the per-finger stiffness is 2 eta Kp / J_w^2 and the damping 2 eta Kd / J_w^2.
    """
    sdk, tr, eff, mim = src["arx5_sdk"], src["transmission"], src["effort"], src["mimic"]
    j_w = _f(sdk["gripper_width_m"]) / _f(sdk["gripper_open_readout_rad"])
    eta = {k: _f(tr[f"efficiency_{k}"]) for k in ("nominal", "low", "high")}
    tau_freeze = _f(eff["freeze_torque_nm_numerator"]) / _f(sdk["torque_constant_dm_j4310"])
    tau_literal = _f(eff["freeze_torque_nm_numerator"])
    tau_2nm = _f(eff["arx_2nm_torque_nm"])
    trip = src["overcurrent_trip"]
    tau_trip = _f(trip["trip_torque_numerator_nm"]) / _f(sdk["torque_constant_dm_j4310"])

    def per_finger_gain(e: Fraction, gain: Fraction) -> Fraction:
        return 2 * e * gain / (j_w * j_w)

    def finger_force(e: Fraction, tau: Fraction) -> Fraction:
        return e * tau / j_w

    sdk_src = {
        "url": sdk["config_url"],
        "quote": "; ".join(
            sdk[q] for q in ("default_gripper_kp_quote", "default_gripper_kd_quote", "gripper_width_quote", "gripper_open_readout_quote")
        ),
        "conditions": f"arx5-sdk robot X5, joint_controller_6 ({sdk['checked']}); control law: {sdk['control_law_quote']}",
    }
    tr_src = {
        "url": "audit/out/32/phase1.md",
        "quote": tr["source"],
        "conditions": f"efficiency {tr['efficiency_nominal']} ({tr['efficiency_low']}-{tr['efficiency_high']}), {tr['status']}",
    }
    eta_note = f"eta = {tr['efficiency_nominal']} nominal, range from eta {tr['efficiency_low']}-{tr['efficiency_high']}"

    def gain_entry(gain_key: str, unit: str, name: str, places: int) -> dict[str, Any]:
        gain = _f(sdk[gain_key])
        return {
            "value": _q_frac(per_finger_gain(eta["nominal"], gain), places),
            "range": [
                _q_frac(per_finger_gain(eta["low"], gain), places),
                _q_frac(per_finger_gain(eta["high"], gain), places),
            ],
            "unit": unit,
            "status": "estimated",
            "derivation": (
                f"2 * eta * {name} / J_w^2 with {name} = {sdk[gain_key]} (arx5-sdk MIT gain at the motor) and J_w = "
                f"{sdk['gripper_width_m']} m / {sdk['gripper_open_readout_rad']} rad; {eta_note}"
            ),
            "basis": (
                "arx5-sdk's position-loop gain (real-stanford/arx5-sdk at c160db7, a community SDK, not necessarily what "
                "RoboDojo's real setup runs) mapped to one finger through the inferred symmetric-rack transmission and "
                "efficiency (#90 item 2, revised November 11)."
            ),
            "sources": [sdk_src, tr_src],
        }

    def effort_entry(value: str, lo: Fraction, hi: Fraction, derivation: str, basis: str) -> dict[str, Any]:
        return {
            "value": float(_f(value)),
            "range": [_q_frac(lo, 2), _q_frac(hi, 2)],
            "unit": "N",
            "status": "estimated",
            "derivation": derivation,
            "basis": basis,
            "sources": [
                {"url": "https://github.com/builtin-pb/robodojo-physics/issues/90", "quote": eff["decision"], "conditions": "D3 default"},
                tr_src,
            ],
        }

    rot = src["rotor_inertia"]
    j_w_alt = _f(rot["alt_stroke_width_m"]) / _f(rot["alt_stroke_readout_rad"])

    def finger_armature(jw: Fraction) -> Fraction:
        # One finger moves dq = (J_w / 2) dtheta_out, so the rotor's reflected mass per finger is 2 G^2 I / J_w^2
        # (joint7, carrying both fingers, gets twice it); efficiency does not enter kinetic energy.
        return 2 * _f(rot["gear_ratio"]) ** 2 * _f(rot["rotor_inertia_kg_m2"]) / (jw * jw)

    software_gearing, software_offset = _f(mim["software_gearing"]), _f(mim["software_offset"])
    mimic_src = {
        "url": mim["robot_config_url"],
        "quote": mim["robot_config_quote"],
        "conditions": mim["physx_relation"],
    }
    return {
        "driven_joint": "joint7",
        "mimic_joint": "joint8",
        "finger_stiffness_n_per_m": gain_entry("default_gripper_kp_nm_per_rad", "N/m", "Kp", 1),
        "finger_damping_n_s_per_m": gain_entry("default_gripper_kd_nm_s_per_rad", "N*s/m", "Kd", 2),
        "finger_effort_limit_n": {
            "nominal": effort_entry(
                eff["nominal_n"], finger_force(eta["low"], tau_freeze), finger_force(eta["high"], tau_2nm),
                f"eta * tau / J_w; range from the arx5-sdk freeze torque {eff['freeze_torque_nm_numerator']} / "
                f"{sdk['torque_constant_dm_j4310']} N*m at eta {tr['efficiency_low']} to ARX's 2 N*m at eta {tr['efficiency_high']}",
                "Nominal where the freeze and 2 N*m readings overlap; inferred, not measured (no load-cell figure exists).",
            ),
            "low": effort_entry(
                eff["low_n"], finger_force(eta["low"], tau_literal), finger_force(eta["high"], tau_literal),
                f"eta * tau / J_w with tau = {eff['freeze_torque_nm_numerator']} N*m read literally; {eta_note}",
                "Sensitivity value: the least plausible torque reading (#90).",
            ),
            "high": effort_entry(
                eff["high_n"], finger_force(eta["low"], tau_2nm), finger_force(eta["high"], tau_2nm),
                f"URDF effort {eff['urdf_effort_n']} N; range is eta * 2 N*m / J_w, {eta_note}",
                f"Sensitivity value: the declared URDF placeholder ({eff['urdf_quote']}).",
            ),
        },
        "finger_trip_n": {
            "value": _q_frac(finger_force(eta["nominal"], tau_trip), 2),
            "range": [_q_frac(finger_force(eta["low"], tau_trip), 2), _q_frac(finger_force(eta["high"], tau_trip), 2)],
            "unit": "N",
            "status": "estimated",
            "derivation": (
                f"eta * tau_trip / J_w with tau_trip = {trip['trip_torque_numerator_nm']} / {sdk['torque_constant_dm_j4310']} N*m "
                f"(gripper_torque_max on the SDK's torque reading, mapped as D3's freeze threshold) and J_w = "
                f"{sdk['gripper_width_m']} m / {sdk['gripper_open_readout_rad']} rad; {eta_note}"
            ),
            "basis": (
                "The drive's engine effort limit under the corrected default (#160): " + trip["decision"] + " "
                + trip["reading_note"] + " " + trip["emergency_note"]
            ),
            "sources": [
                {"url": trip["trip_url"], "quote": trip["trip_quote"], "conditions": f"arx5-sdk {trip['commit'][:7]} ({trip['checked']})"},
                {"url": trip["torque_max_url"], "quote": trip["torque_max_quote"], "conditions": f"arx5-sdk {trip['commit'][:7]}"},
                {"url": trip["reading_url"], "quote": trip["reading_quote"], "conditions": f"arx5-sdk {trip['commit'][:7]}"},
                {"url": trip["emergency_url"], "quote": trip["emergency_quote"], "conditions": "not modelled (open caveat)"},
                tr_src,
            ],
        },
        "finger_motor_peak_n": {
            "value": _q_frac(finger_force(eta["nominal"], _f(src["motor_peak"]["peak_torque_nm"])), 2),
            "range": [
                _q_frac(finger_force(eta["low"], _f(src["motor_peak"]["peak_torque_nm"])), 2),
                _q_frac(finger_force(eta["high"], _f(src["motor_peak"]["peak_torque_nm"])), 2),
            ],
            "unit": "N",
            "status": "estimated",
            "derivation": (
                f"eta * tau_peak / J_w with tau_peak = {src['motor_peak']['peak_torque_nm']} N*m (DM-J4310-2EC V1.1) and J_w = "
                f"{sdk['gripper_width_m']} m / {sdk['gripper_open_readout_rad']} rad; {eta_note}"
            ),
            "basis": (
                "The drive's engine effort limit under scenario gripper_drive_limit = motor_peak (#160 sensitivity: "
                "arx5-sdk's law uncapped): the motor's peak torque through the rack. " + src["motor_peak"]["controller_note"]
            ),
            "sources": [
                {"url": src["motor_peak"]["url"], "quote": src["motor_peak"]["quote"], "conditions": src["motor_peak"]["conditions"]},
                {"url": trip["trip_url"].split("#")[0] + "#L562-L578", "quote": src["motor_peak"]["controller_quote"], "conditions": "arx5-sdk c160db7"},
                tr_src,
            ],
        },
        "finger_armature_kg": {
            "value": _q_frac(finger_armature(j_w), 3),
            "range": sorted([_q_frac(finger_armature(j_w), 3), _q_frac(finger_armature(j_w_alt), 3)]),
            "unit": "kg",
            "status": "estimated",
            "derivation": (
                f"2 * G^2 * I_rotor / J_w^2 with I_rotor = {rot['rotor_inertia_kg_m2']} kg*m^2, G = {rot['gear_ratio']}, "
                f"J_w = {sdk['gripper_width_m']} m / {sdk['gripper_open_readout_rad']} rad; range to J_w = "
                f"{rot['alt_stroke_width_m']} m / {rot['alt_stroke_readout_rad']} rad"
            ),
            "basis": (
                "The motor rotor's inertia reflected through the 10:1 gearbox and the rack to one finger (#90 round 2). "
                + rot["motor_side_check"] + ". " + rot["status"] + "."
            ),
            "sources": [
                {"url": rot["url"], "quote": rot["quote"], "conditions": f"page sha256 {rot['page_sha256']} ({rot['checked']})"},
                {"url": "https://www.worldrobotconference.com/profile/robot/download/2025/06/30/达妙科技DAMIAO - 2025年产品选型手册_20250630192546A441.pdf", "quote": rot["gear_ratio_quote"], "conditions": "official Damiao selection manual"},
                tr_src,
            ],
        },
        "slew_m_per_s": {
            "value": _q_frac(_f(sdk["gripper_vel_max_m_s"]) / 2, 6),
            "range": [_q_frac(_f(sdk["gripper_vel_max_m_s"]) / 2, 6), _q_frac(_f(sdk["gripper_vel_max_m_s"]) / 2, 6)],
            "unit": "m/s",
            "status": "estimated",
            "derivation": f"gripper_vel_max {sdk['gripper_vel_max_m_s']} m/s of jaw width / 2 fingers",
            "basis": "arx5-sdk slew-limits the previous command, not the measured opening, so it caps free speed and not steady force; whether RoboDojo's real rig ran arx5-sdk is unknown.",
            "sources": [{"url": sdk["controller_url"], "quote": sdk["slew_quote"], "conditions": sdk["gripper_vel_max_quote"]}],
        },
        "mimic": {
            "gearing": {
                "value": float(-software_gearing),
                "status": "estimated",
                "basis": f"q8 = {mim['software_gearing']} * q7 + {mim['software_offset']} (RoboDojo's software mimic) in PhysX form q8 + gearing * q7 + offset = 0",
                "sources": [mimic_src],
            },
            "offset": {
                "value": float(-software_offset),
                "status": "estimated",
                "basis": "Negated software mimic offset (same relation as gearing).",
                "sources": [mimic_src],
            },
            "axis": mim["axis_token"],
            "notes": mim["axis_note"],
        },
        "notes": (
            "Per-finger values. joint8's drive is off and the mimic constraint carries finger8's load to joint7, so "
            "joint7's engine stiffness, damping, effort limit and armature are twice these. Scenario key gripper_armature "
            "= none drops the armature (D3 as first implemented). Scenario key gripper_effort selects "
            "nominal, low or high (the freeze threshold only). Scenario key gripper_drive_limit selects joint7's engine "
            "limit (#160): trip (default, finger_trip_n) or motor_peak (finger_motor_peak_n). "
            "The gains, thresholds and limits follow arx5-sdk's control law (real-stanford/arx5-sdk at c160db7, a community "
            "SDK, not necessarily what RoboDojo's real setup runs), mapped through an inferred transmission and efficiency, so "
            "the forces carry about +-20-30% (efficiency 0.7-0.9 alone +-12.5%; the alternative stroke mapping up to +9%), "
            "and 2.4x less if the SDK's torque units are read literally (gripper_effort low). "
            f"Velocity target stays 0: {sdk['velocity_feedforward_note']}"
        ),
    }


def build_arm_limits_block(src: dict[str, Any]) -> dict[str, Any]:
    """#11 ARX X5 arm limits, copied from arx5-sdk's X5 configuration (the only published controller limits)."""
    sdk, arx = src["arx5_sdk"], src["arx_official"]

    def vec(key: str) -> list[float]:
        return [float(_f(x)) for x in sdk[key]]

    sdk_cond = f"real-stanford/arx5-sdk {sdk['commit'][:7]} ({src['checked']}); {sdk['frame_check']}"
    arx_src = {"url": arx["repo"], "quote": arx["urdf_quote"], "conditions": arx["manual_note"]}
    rd_src = {"url": "https://github.com/builtin-pb/robodojo-physics/issues/11", "quote": src["robodojo_unloaded_layer"]["quote"], "conditions": src["robodojo_unloaded_layer"]["use"]}

    def leaf(key: str, unit: str, status: str, quotes: tuple[str, ...], basis: str) -> dict[str, Any]:
        # quotes[0] is the config.h value; the rest show how controller_base.cpp uses it.
        return {
            "value": vec(key),
            "unit": unit,
            "status": status,
            "basis": basis,
            "sources": [
                {"url": sdk["config_url"], "quote": sdk[quotes[0]], "conditions": sdk_cond},
                {"url": sdk["controller_url"], "quote": " ".join(sdk[q] for q in quotes[1:]), "conditions": sdk_cond},
                arx_src,
            ]
            + ([rd_src] if key.startswith("joint_pos") else []),
        }

    return {
        "joints": list(src["joints"]),
        "lower_rad": leaf(
            "joint_pos_min", "rad", "sourced", ("range_quote", "range_use_quote"),
            "arx5-sdk clips every command to this range; ARX publishes no range (its URDF carries +-10 rad placeholders).",
        ),
        "upper_rad": leaf(
            "joint_pos_max", "rad", "sourced", ("range_quote", "range_use_quote"),
            "arx5-sdk clips every command to this range; ARX publishes no range (its URDF carries +-10 rad placeholders).",
        ),
        "velocity_rad_s": leaf(
            "joint_vel_max", "rad/s", "sourced", ("velocity_quote", "velocity_use_quote"),
            "arx5-sdk slew-limits the command at this rate. In the sim it caps the joint's speed (PhysX maxJointVelocity), as x5.py's 5.0 rad/s already does.",
        ),
        "effort_nm": leaf(
            "joint_torque_max", "N*m", "estimated", ("torque_quote", "torque_use_quote", "torque_units_note", "motor_quote"),
            "arx5-sdk's over-current threshold (emergency stop after 20 consecutive 2 ms cycles above it) and feed-forward clip, "
            "on its own torque reading; authored as the drive's effort limit. DM-J4310 peak torque "
            f"({src['dm_j4310_peak_torque_nm']['v1_1']} N*m V1.1, {src['dm_j4310_peak_torque_nm']['v1_2']} N*m V1.2) may bind first on joints 4-6. "
            "arx5-sdk's gains (c160db7, a community SDK, not necessarily what RoboDojo's real setup runs) are far softer "
            "than RoboDojo's drive (" + sdk["gains_quote"] + "; RoboDojo K 4400, D 40, "
            "velocity target 0), so these caps throttle the sim arm where its damping alone exceeds them.",
        ),
        "notes": (
            "Applied under the physics fix robot_limits: ranges authored on the USD joints before parse, velocity and effort "
            "through the arm actuator config. Scenario key arm_effort = shipped keeps RoboDojo's 100 N*m effort limit."
        ),
    }


def _collect_unsourced_keys_for_fill(
    classes_doc: dict[str, Any],
    pairs_doc: dict[str, Any],
    assets_doc: dict[str, Any],
    robots_doc: dict[str, Any],
    fill_mode: str,
) -> list[str]:
    """Compute the exact sorted list of status='unsourced' parameter keys checked by PhysicsParamsTable for fill_mode."""
    keys: set[str] = set()

    for cname, cdict in sorted(classes_doc["classes"].items()):
        for field_name in ("density_kg_m3", "shell_density_kg_m3"):
            if field_name in cdict and cdict[field_name].get("status") == "unsourced":
                keys.add(f"classes.{cname}.{field_name}")
        if "restitution" in cdict:
            r = cdict["restitution"]
            if r.get("status") == "unsourced":
                keys.add(f"classes.{cname}.restitution")
            if fill_mode == "full":
                fo_full = (r.get("fill_overrides") or {}).get("full")
                if isinstance(fo_full, dict) and fo_full.get("status", r.get("status")) == "unsourced":
                    keys.add(f"classes.{cname}.restitution.fill_overrides.full")

    pair_statuses_by_cls: dict[str, list[str]] = {c: [] for c in classes_doc["unary_values"]["static"]}
    for pk, pdict in sorted(pairs_doc["pairs"].items()):
        ca, cb = pk.split("__", 1)
        s_st = pdict["mu_static"].get("range_status") or pdict["mu_static"].get("status") or "unsourced"
        k_st = pdict["mu_kinetic"].get("range_status") or pdict["mu_kinetic"].get("status") or "unsourced"
        if s_st == "unsourced":
            keys.add(f"pairs.{pk}.mu_static")
        if k_st == "unsourced":
            keys.add(f"pairs.{pk}.mu_kinetic")
        weak = "unsourced" if "unsourced" in (s_st, k_st) else ("estimated" if "estimated" in (s_st, k_st) else "sourced")
        if ca in pair_statuses_by_cls and cb in pair_statuses_by_cls:
            pair_statuses_by_cls[ca].append(weak)
            pair_statuses_by_cls[cb].append(weak)

        if "restitution" in pdict and pdict["restitution"].get("impact_speed_above_0.5_m_s"):
            pr = pdict["restitution"]
            fo = (pr.get("fill_overrides") or {}).get(fill_mode) if fill_mode != "empty" else None
            if isinstance(fo, dict):
                rst = fo.get("range_status") or fo.get("status") or pr.get("range_status") or pr.get("status") or "unsourced"
                if rst == "unsourced":
                    keys.add(f"pairs.{pk}.restitution.fill_overrides.{fill_mode}")
            else:
                rst = pr.get("range_status") or pr.get("status") or "unsourced"
                if rst == "unsourced":
                    keys.add(f"pairs.{pk}.restitution")

    for cname, sts in sorted(pair_statuses_by_cls.items()):
        if not sts or "unsourced" in sts:
            keys.add(f"unary_values.{cname}")

    for aid, adict in sorted(assets_doc["assets"].items()):
        for sk in ("size_m", "collider_extents_m", "solid_collider_inertia_over_mass_m2"):
            if sk in adict and isinstance(adict[sk], dict) and adict[sk].get("status") == "unsourced":
                keys.add(f"assets.{aid}.{sk}")
        if "mass_kg" in adict and adict["mass_kg"].get("status") == "unsourced":
            keys.add(f"assets.{aid}.mass_kg")
        for tname, tv in sorted((adict.get("task_variants") or {}).items()):
            for tk in ("mass_kg", "size_m"):
                if tk in tv and tv[tk].get("status") == "unsourced":
                    keys.add(f"assets.{aid}.task_variants.{tname}.{tk}")
        if "mass_by_fill" in adict:
            mf = adict["mass_by_fill"][fill_mode]
            if mf.get("status") == "unsourced":
                keys.add(f"assets.{aid}.mass_by_fill.{fill_mode}")
        for bf_name in ("inertia_over_mass_by_fill", "com_offset_by_fill"):
            if bf_name in adict and adict[bf_name][fill_mode].get("status") == "unsourced":
                keys.add(f"assets.{aid}.{bf_name}.{fill_mode}")
        if "material_override" in adict:
            mo = adict["material_override"]
            if mo.get("status") == "unsourced":
                keys.add(f"assets.{aid}.material_override")
            if "restitution_by_fill" in mo and mo["restitution_by_fill"][fill_mode].get("status") == "unsourced":
                keys.add(f"assets.{aid}.material_override.restitution_by_fill.{fill_mode}")
        if "collision" in adict and adict["collision"].get("status") == "unsourced":
            keys.add(f"assets.{aid}.collision")

    for rname, rdict in sorted(robots_doc["robots"].items()):
        for lk, lv in sorted((rdict.get("non_pad_link_material_classes") or {}).items()):
            if lv.get("status") == "unsourced":
                keys.add(f"robots.{rname}.non_pad_link_material_classes.{lk}")
        if "sanitize_camera_inertials" in rdict and rdict["sanitize_camera_inertials"].get("status") == "unsourced":
            keys.add(f"robots.{rname}.sanitize_camera_inertials")

    return sorted(keys)


def verify_reviewed_opt_in_list(actual_keys: list[str], reviewed_keys: list[str], *, profile_name: str) -> list[str]:
    """Verify computed unsourced keys match the committed reviewed opt-in list (never auto-admit new unsourced leaves)."""
    expected = sorted(str(k) for k in reviewed_keys)
    if list(reviewed_keys) != expected:
        raise RuntimeError(f"Reviewed opt-in profile {profile_name!r} must be sorted and deduplicated")
    if actual_keys != expected:
        unreviewed_new = sorted(set(actual_keys) - set(expected))
        stale_removed = sorted(set(expected) - set(actual_keys))
        raise RuntimeError(
            f"Unsourced leaf set for profile {profile_name!r} drifted from committed reviewed list: "
            f"unreviewed_new={unreviewed_new}, stale_removed={stale_removed}"
        )
    return list(expected)


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--population", default=None, help="Path to audit/out/31/population.json")
    ap.add_argument("--classes", default=None, help="Path to audit/out/63/classes.json")
    ap.add_argument("--class-values", default=None, help="Path to audit/out/63/class_values.json")
    ap.add_argument("--feasibility", default=None, help="Path to audit/out/63/feasibility.json")
    ap.add_argument(
        "--bottle-slice",
        default=PINNED_BOTTLE_SLICE_SPEC,
        help="Path or git:<rev>:<path> to #84 bottle slice JSON",
    )
    ap.add_argument(
        "--task-slice",
        action="append",
        default=[],
        metavar="TASK=SPEC",
        help="Override a non-bottle task slice (path or git:<rev>:<path>); its SHA-256 pin is then not enforced",
    )
    ap.add_argument("--usd-facts", default=None, help="Path to audit/out/63/usd_geometry_facts.json")
    ap.add_argument(
        "--usd-facts-cb-swapt", default=None, help="Path to audit/out/63/usd_geometry_facts_cb_swapT.json"
    )
    ap.add_argument(
        "--usd-facts-t1", default=None, help="Path to audit/out/63/usd_geometry_facts_population_t1.json"
    )
    ap.add_argument(
        "--t1-rows", default=None, help="Path to audit/out/63/population/t1_rows.json"
    )
    ap.add_argument(
        "--basis-drafts", default=None, help="Path to audit/out/63/population/basis_drafts.md"
    )
    ap.add_argument(
        "--usd-facts-t2", default=None, help="Path to audit/out/63/usd_geometry_facts_population_t2.json.gz"
    )
    ap.add_argument(
        "--t2-rows", default=None, help="Path to audit/out/63/population/t2_rows.json.gz"
    )
    ap.add_argument(
        "--basis-drafts-t2", default=None, help="Path to audit/out/63/population/basis_drafts_t2.md"
    )
    ap.add_argument(
        "--reviewed-opt-in",
        default=None,
        help="Path to committed audit/tools/reviewed_opt_in_profiles.json",
    )
    ap.add_argument(
        "--gripper-sources",
        default=None,
        help="Path to audit/out/d3/gripper_sources.json (D3 gripper constants)",
    )
    ap.add_argument(
        "--arm-limits-sources",
        default=None,
        help="Path to audit/out/d3/arm_limits_sources.json (#11 arm limits)",
    )
    ap.add_argument(
        "--contact-census",
        action="append",
        default=[],
        metavar="TASK=SPEC",
        help="Override a pinned contact census JSON (path or git:<rev>:<path>); its SHA-256 pin is then not enforced",
    )
    ap.add_argument("--out", default=None, help="Output directory (default: <repo>/docs/params)")
    args = ap.parse_args()

    need_repo = (
        args.population is None
        or args.classes is None
        or args.class_values is None
        or args.feasibility is None
        or args.usd_facts is None
        or args.usd_facts_cb_swapt is None
        or bool(PINNED_TASK_SLICES[1:])
        or bool(PINNED_CONTACT_CENSUSES)
        or args.reviewed_opt_in is None
        or args.gripper_sources is None
        or args.arm_limits_sources is None
        or args.out is None
        or str(args.bottle_slice).startswith("git:")
    )
    repo_dir = _repo_root(here) if need_repo else None

    pop_path = args.population or os.path.join(repo_dir, "audit/out/31/population.json")
    classes_path = args.classes or os.path.join(repo_dir, "audit/out/63/classes.json")
    cv_path = args.class_values or os.path.join(repo_dir, "audit/out/63/class_values.json")
    feas_path = args.feasibility or os.path.join(repo_dir, "audit/out/63/feasibility.json")
    usd_path = args.usd_facts or os.path.join(repo_dir, "audit/out/63/usd_geometry_facts.json")
    rev_opt_path = args.reviewed_opt_in or os.path.join(repo_dir, "audit/tools/reviewed_opt_in_profiles.json")
    gripper_src_path = args.gripper_sources or os.path.join(repo_dir, "audit/out/d3/gripper_sources.json")
    arm_src_path = args.arm_limits_sources or os.path.join(repo_dir, "audit/out/d3/arm_limits_sources.json")
    out_dir = args.out or os.path.join(repo_dir, "docs/params")

    pop_src, pop_sha = _load_json_with_sha(pop_path, repo_dir)
    classes_src, classes_sha = _load_json_with_sha(classes_path, repo_dir)
    cv_src, cv_sha = _load_json_with_sha(cv_path, repo_dir)
    feas_src, feas_sha = _load_json_with_sha(feas_path, repo_dir)
    slice_src, slice_sha = _load_json_with_sha(args.bottle_slice, repo_dir)
    usd_src, usd_sha = _load_json_with_sha(usd_path, repo_dir)
    usd_cb_path = args.usd_facts_cb_swapt or os.path.join(repo_dir, USD_FACTS_CB_SWAPT[0])
    usd_cb_src, usd_cb_sha = _load_json_with_sha(
        usd_cb_path, repo_dir, expected_sha256=None if args.usd_facts_cb_swapt else USD_FACTS_CB_SWAPT[1]
    )
    usd_t1_path = args.usd_facts_t1 or os.path.join(repo_dir, USD_FACTS_POP_T1[0])
    usd_t1_src, usd_t1_sha = _load_json_with_sha(
        usd_t1_path, repo_dir, expected_sha256=None if args.usd_facts_t1 else USD_FACTS_POP_T1[1]
    )
    t1_rows_path = args.t1_rows or os.path.join(repo_dir, T1_POPULATION_ROWS[0])
    t1_rows_src, t1_rows_sha = _load_json_with_sha(
        t1_rows_path, repo_dir, expected_sha256=None if args.t1_rows else T1_POPULATION_ROWS[1]
    )
    basis_md_path = args.basis_drafts or os.path.join(repo_dir, BASIS_DRAFTS_MD[0])
    basis_md_txt, basis_md_sha = _load_text_with_sha(
        basis_md_path, expected_sha256=None if args.basis_drafts else BASIS_DRAFTS_MD[1]
    )
    basis_drafts_map = _parse_basis_drafts_md(basis_md_txt, expected_count=85)
    usd_t2_path = args.usd_facts_t2 or os.path.join(repo_dir, USD_FACTS_POP_T2[0])
    usd_t2_src, usd_t2_sha = _load_json_with_sha(
        usd_t2_path, repo_dir, expected_sha256=None if args.usd_facts_t2 else USD_FACTS_POP_T2[1]
    )
    t2_rows_path = args.t2_rows or os.path.join(repo_dir, T2_POPULATION_ROWS[0])
    t2_rows_src, t2_rows_sha = _load_json_with_sha(
        t2_rows_path, repo_dir, expected_sha256=None if args.t2_rows else T2_POPULATION_ROWS[1]
    )
    basis_t2_path = args.basis_drafts_t2 or os.path.join(repo_dir, BASIS_DRAFTS_T2_MD[0])
    basis_t2_txt, basis_t2_sha = _load_text_with_sha(
        basis_t2_path, expected_sha256=None if args.basis_drafts_t2 else BASIS_DRAFTS_T2_MD[1]
    )
    basis_drafts_t2_raw = _parse_basis_drafts_md(basis_t2_txt, expected_count=30)
    t2_q_tags: dict[str, str] = {str(k): str(v) for k, v in (t2_rows_src.get("decision_status_tags") or {}).items()}
    q1_sfx = f" [{t2_q_tags['Q1']}]" if t2_q_tags.get("Q1") else ""
    basis_drafts_t2_map = {k: (f"{v}{q1_sfx}" if not v.endswith(q1_sfx) else v) for k, v in basis_drafts_t2_raw.items()}
    slice_overrides: dict[str, str] = {}
    for item in args.task_slice:
        tname_o, _, spec_o = str(item).partition("=")
        if not spec_o or tname_o not in {t for t, _, _ in PINNED_TASK_SLICES[1:]}:
            raise SystemExit(f"--task-slice expects TASK=SPEC for one of {[t for t, _, _ in PINNED_TASK_SLICES[1:]]}")
        slice_overrides[tname_o] = spec_o
    task_slices: dict[str, dict[str, Any]] = {"put_bottles_into_dustbin": slice_src}
    task_slice_shas: dict[str, tuple[str, str]] = {}
    for tname_s, spec_s, sha_s in PINNED_TASK_SLICES[1:]:
        if tname_s in slice_overrides:
            spec_use, pin = slice_overrides[tname_s], None
        else:
            spec_use = spec_s if spec_s.startswith("git:") else os.path.join(repo_dir, spec_s)
            pin = sha_s
        src_s, got_sha = _load_json_with_sha(spec_use, repo_dir, expected_sha256=pin)
        if src_s.get("task") != tname_s:
            raise RuntimeError(f"Slice {spec_use!r} is for task {src_s.get('task')!r}, expected {tname_s!r}")
        task_slices[tname_s] = src_s
        task_slice_shas[tname_s] = (spec_s if tname_s not in slice_overrides else f"{tname_s}_slice_override", got_sha)
    rev_opt_src, rev_opt_sha = _load_json_with_sha(rev_opt_path, repo_dir)
    gripper_src, gripper_src_sha = _load_json_with_sha(gripper_src_path, repo_dir)
    arm_src, arm_src_sha = _load_json_with_sha(arm_src_path, repo_dir)

    census_overrides: dict[str, str] = {}
    for item_c in args.contact_census:
        tname_c, _, spec_c = str(item_c).partition("=")
        if not spec_c or tname_c not in {t for t, _, _ in PINNED_CONTACT_CENSUSES}:
            raise SystemExit(
                f"--contact-census expects TASK=SPEC for one of {[t for t, _, _ in PINNED_CONTACT_CENSUSES]}"
            )
        census_overrides[tname_c] = spec_c
    contact_censuses: dict[str, dict[str, Any]] = {}
    contact_census_shas: dict[str, tuple[str, str]] = {}
    for tname_c, spec_c, sha_c in PINNED_CONTACT_CENSUSES:
        if tname_c in census_overrides:
            spec_use_c, pin_c = census_overrides[tname_c], None
        else:
            spec_use_c = spec_c if spec_c.startswith("git:") else os.path.join(repo_dir, spec_c)
            pin_c = sha_c
        src_c, got_sha_c = _load_json_with_sha(spec_use_c, repo_dir, expected_sha256=pin_c)
        if src_c.get("task") != tname_c:
            raise RuntimeError(f"Contact census {spec_use_c!r} is for task {src_c.get('task')!r}, expected {tname_c!r}")
        contact_censuses[tname_c] = src_c
        contact_census_shas[tname_c] = (
            spec_c if tname_c not in census_overrides else f"{tname_c}_census_override",
            got_sha_c,
        )

    slice_key_label = (
        args.bottle_slice
        if str(args.bottle_slice).startswith("git:")
        else "bottle_slice_override"
    )
    provenance_block: dict[str, Any] = {
        "generator": "audit/tools/build_docs_params.py",
        "hf_dataset_revision": HF_REVISION,
        "pinned_bottle_slice": PINNED_BOTTLE_SLICE_SPEC,
        "pinned_task_slices": {t: spec for t, spec, _ in PINNED_TASK_SLICES},
        "pinned_t1_population_rows": T1_POPULATION_ROWS[0],
        "pinned_t2_population_rows": T2_POPULATION_ROWS[0],
        "pinned_contact_censuses": {t: spec for t, spec, _ in PINNED_CONTACT_CENSUSES},
        "input_checksums_sha256": {
            "audit/out/31/population.json": pop_sha,
            "audit/out/63/classes.json": classes_sha,
            "audit/out/63/class_values.json": cv_sha,
            "audit/out/63/feasibility.json": feas_sha,
            slice_key_label: slice_sha,
            **{lbl: sha for lbl, sha in task_slice_shas.values()},
            "audit/out/63/usd_geometry_facts.json": usd_sha,
            USD_FACTS_CB_SWAPT[0]: usd_cb_sha,
            USD_FACTS_POP_T1[0]: usd_t1_sha,
            T1_POPULATION_ROWS[0]: t1_rows_sha,
            BASIS_DRAFTS_MD[0]: basis_md_sha,
            USD_FACTS_POP_T2[0]: usd_t2_sha,
            T2_POPULATION_ROWS[0]: t2_rows_sha,
            BASIS_DRAFTS_T2_MD[0]: basis_t2_sha,
            "audit/tools/reviewed_opt_in_profiles.json": rev_opt_sha,
            **{lbl_c: sha_c for lbl_c, sha_c in contact_census_shas.values()},
        },
    }

    models_by_id: dict[str, dict[str, Any]] = {m["id"]: m for m in classes_src["models"]}
    slice_unaries = slice_src["physx_combine_and_unary_realisation"]["unary_materials_average_combine"]["classes"]
    t1_tasks_list: list[str] = list(t1_rows_src["t1_tasks"])
    t1_assets_src: dict[str, Any] = t1_rows_src["assets"]
    t1_task_census: dict[str, Any] = t1_rows_src["task_layout_census"]
    t2_tasks_list: list[str] = list(t2_rows_src["t2_tasks"])
    t2_assets_src: dict[str, Any] = t2_rows_src["assets"]
    t2_task_census: dict[str, Any] = t2_rows_src["task_layout_census"]

    # 1. Build classes base entries & combine modes
    out_classes: dict[str, Any] = {}
    u_fmode: dict[str, str] = {}
    u_rmode: dict[str, str] = {}

    t0_t1_cls_set: set[str] = (
        {c for sl in task_slices.values() for c in sl["classes"]}
        | {str(tr["material_class"]) for tr in t1_assets_src.values()}
        | set(ROBOT_LINK_CLASSES)
    )
    # Classes in scope: every class a task slice, T1 row, or T2 row carries, plus the X5 non-pad link classes (robots.json).
    task_classes: tuple[str, ...] = tuple(
        sorted(
            t0_t1_cls_set
            | {str(tr["material_class"]) for tr in t2_assets_src.values()}
        )
    )
    t2_only_cls_set: set[str] = set(task_classes) - t0_t1_cls_set
    for cname in task_classes:
        raw_c = cv_src["classes"][cname]
        is_t2_cls = cname in t2_only_cls_set
        c_entry: dict[str, Any] = {}
        if "density_kg_m3" in raw_c and raw_c["density_kg_m3"] is not None:
            d_clean = _clean_band(raw_c["density_kg_m3"])
            d_bkey = f"classes.{cname}.density_kg_m3"
            if is_t2_cls and d_bkey in basis_drafts_t2_map and not (isinstance(d_clean.get("basis"), str) and d_clean["basis"].strip()):
                d_clean["basis"] = basis_drafts_t2_map[d_bkey]
            c_entry["density_kg_m3"] = d_clean
        if "restitution" in raw_c and raw_c["restitution"] is not None:
            r_clean = sanitize_wood_restitution_entry(raw_c["restitution"])
            r_bkey = f"classes.{cname}.restitution"
            r_sts = {r_clean.get(k) for k in ("status", "value_status", "range_status") if k in r_clean}
            if (r_sts & {"unsourced", "estimated"}) and not _entry_has_basis_text(r_clean) and r_bkey in basis_drafts_map:
                r_clean["basis"] = basis_drafts_map[r_bkey]
            elif is_t2_cls and r_bkey in basis_drafts_t2_map and not (isinstance(r_clean.get("basis"), str) and r_clean["basis"].strip()):
                r_clean["basis"] = basis_drafts_t2_map[r_bkey]
            elif is_t2_cls and q1_sfx and isinstance(r_clean.get("basis"), str) and r_clean["basis"].strip() and not r_clean["basis"].endswith(q1_sfx):
                r_clean["basis"] = f"{r_clean['basis']}{q1_sfx}"
            c_entry["restitution"] = r_clean
        if "scenario_membership" in raw_c:
            c_entry["scenario_membership"] = raw_c["scenario_membership"]
        if "notes" in raw_c:
            c_entry["notes"] = raw_c["notes"]
        out_classes[cname] = c_entry

        u_fmode[cname] = str(slice_unaries[cname]["friction_combine_mode"]) if cname in slice_unaries else "average"
        u_rmode[cname] = "average"

    # 2. Build pairs.json entries
    slice_pairs = slice_src["pairs"]
    out_pairs: dict[str, Any] = {}
    task_cls_set = set(task_classes)
    t2_new_pair_keys: set[str] = set()
    for p in sorted(cv_src["pairs"], key=lambda r: r["key"]):
        ca, cb = p["a"], p["b"]
        if ca not in task_cls_set or cb not in task_cls_set:
            continue
        pk = p["key"]
        is_t2_pair = (ca in t2_only_cls_set) or (cb in t2_only_cls_set)
        if is_t2_pair:
            t2_new_pair_keys.add(pk)
        ms_clean = _clean_band(p["mu_static"])
        mk_clean = _clean_band(p["mu_kinetic"])
        for fk_b, leaf_b in (("mu_static", ms_clean), ("mu_kinetic", mk_clean)):
            st_b_set = {leaf_b.get(k) for k in ("status", "value_status", "range_status") if k in leaf_b}
            bkey_p = f"pairs.{pk}.{fk_b}"
            if not is_t2_pair:
                if (st_b_set & {"unsourced", "estimated"}) and not (isinstance(leaf_b.get("basis"), str) and leaf_b["basis"].strip()) and bkey_p in basis_drafts_map:
                    leaf_b["basis"] = basis_drafts_map[bkey_p]
            else:
                if (st_b_set & {"unsourced", "estimated"}) and not (isinstance(leaf_b.get("basis"), str) and leaf_b["basis"].strip()):
                    if bkey_p in basis_drafts_t2_map:
                        leaf_b["basis"] = basis_drafts_t2_map[bkey_p]
                    elif bkey_p in basis_drafts_map:
                        bs_v = basis_drafts_map[bkey_p]
                        leaf_b["basis"] = f"{bs_v}{q1_sfx}" if (q1_sfx and not bs_v.endswith(q1_sfx)) else bs_v
        p_out: dict[str, Any] = {
            "a": ca,
            "b": cb,
            "key": pk,
            "kind": p["kind"],
            "scenario_membership": p["scenario_membership"],
            "origin": p["origin"],
            "mu_static": ms_clean,
            "mu_kinetic": mk_clean,
        }
        if pk in slice_pairs and "restitution_by_fill" in slice_pairs[pk]:
            rbf = slice_pairs[pk]["restitution_by_fill"]
            emp = rbf["empty"]
            ful = rbf["full"]
            is_pad_pair = ca.startswith("pad_") or cb.startswith("pad_")
            p_out["restitution"] = {
                "value": float(emp["value"]),
                "range": [float(emp["range"][0]), float(emp["range"][1])],
                "unit": "dimensionless",
                "status": str(emp["status"]),
                "value_status": str(emp["status"]),
                "range_status": str(emp["status"]),
                "value_is_for_fill": "empty",
                "impact_speed_above_0.5_m_s": not is_pad_pair,
                "basis": str(emp.get("basis") or ""),
                "sources": list(emp.get("sources") or []),
                "fill_overrides": {
                    "full": {
                        "value": float(ful["value"]),
                        "range": [float(ful["range"][0]), float(ful["range"][1])],
                        "status": str(ful["status"]),
                        "value_status": str(ful["status"]),
                        "range_status": str(ful["status"]),
                        "basis": str(ful.get("basis") or ""),
                        "sources": list(ful.get("sources") or []),
                    }
                },
            }
        if p.get("cooccurrence") is not None:
            p_out["cooccurrence"] = p["cooccurrence"]
        if p.get("notes") is not None:
            p_out["notes"] = p["notes"]
        out_pairs[pk] = p_out

    # 2b. Pairs banded only in a task slice (#167 pad-surface analogue bands). Every slice copy of a class-table pair
    # must equal the class table (main wins, #84 review B4); a slice-only band must be identical across slices.
    cv_by_key = {p["key"]: p for p in cv_src["pairs"]}
    slice_only_pairs: dict[str, tuple[str, dict[str, Any]]] = {}
    for tname_p, sl in sorted(task_slices.items()):
        for pk, sp in sorted((sl.get("pairs") or {}).items()):
            if sp["a"] not in task_cls_set or sp["b"] not in task_cls_set:
                raise RuntimeError(f"Slice {tname_p} pair {pk} uses a class outside scope {task_classes}")
            if pk in cv_by_key:
                for fld in ("mu_static", "mu_kinetic"):
                    if "value" in sp[fld] and (
                        _d(sp[fld]["value"]) != _d(cv_by_key[pk][fld]["value"])
                        or [_d(x) for x in sp[fld]["range"]] != [_d(x) for x in cv_by_key[pk][fld]["range"]]
                    ):
                        raise RuntimeError(f"Slice {tname_p} pair {pk}.{fld} differs from class_values.json")
                continue
            entry = {
                "a": sp["a"],
                "b": sp["b"],
                "key": pk,
                "kind": sp["kind"],
                "scenario_membership": sp["scenario_membership"],
                "origin": "analogue_167",
                "mu_static": _clean_band(sp["mu_static"]),
                "mu_kinetic": _clean_band(sp["mu_kinetic"]),
                "notes": "Band added on #167 (pad-surface pair formed by engine objects but unbanded in the class table): copied from the class-table analogue named in its derivation, status not raised.",
            }
            if pk in slice_only_pairs:
                prev_t, prev = slice_only_pairs[pk]
                if prev["mu_static"] != entry["mu_static"] or prev["mu_kinetic"] != entry["mu_kinetic"]:
                    raise RuntimeError(f"Slice-only pair {pk} differs between slices {prev_t} and {tname_p}")
                continue
            slice_only_pairs[pk] = (tname_p, entry)
    for pk, (_, entry) in slice_only_pairs.items():
        out_pairs[pk] = entry
    out_pairs = {pk: out_pairs[pk] for pk in sorted(out_pairs)}

    # 3. Build assets.json
    out_assets: dict[str, Any] = {}
    bottle_ids = sorted(k for k in slice_src["assets"] if k.startswith("Rigid/bottle/"))
    for aid in bottle_ids:
        m_row = models_by_id[aid]
        s_row = slice_src["assets"][aid]
        u_row = usd_src["bottles"][aid]
        flags = list(m_row["flags"])
        if (
            aid == "Rigid/bottle/00022"
            and feas_src["per_asset_override_bottle_00022"]["scored_layouts_with_two_or_more_00022"] == 0
        ):
            flags.append("no_self_contact")

        emp_mass_band = _clean_band(s_row["mass_by_fill"]["empty"])
        ful_mass_band = _clean_band(s_row["mass_by_fill"]["full"])
        m_emp_d = _d(emp_mass_band["value"])
        m_ful_d = _d(ful_mass_band["value"])

        com_sh = u_row["shell_com_offset_m"]
        iom_sh = u_row["shell_inertia_over_mass_m2"]
        com_so = u_row["solid_com_offset_m"]
        iom_so = u_row["solid_inertia_over_mass_m2"]
        com_ful, iom_ful = _composite_full_moments(m_emp_d, m_ful_d, com_sh, iom_sh, com_so, iom_so)
        hf_usd_url = (
            f"https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/resolve/{HF_REVISION}/{u_row['usd_relpath']}"
        )
        usd_src_ref = [
            {
                "url": hf_usd_url,
                "quote": f"sha256={u_row['usd_sha256']} (outer convex hull: {u_row['hull_vertex_count']} vertices, {u_row['hull_face_count']} faces, volume={u_row['outer_hull_volume_m3']} m^3, area={u_row['outer_hull_area_m2']} m^2)",
                "conditions": "Computed deterministically by audit/tools/extract_usd_geometry_facts.py from pinned HF 1127bda9 USD mesh vertices.",
            }
        ]

        sz_raw = s_row["size_m"]
        a_out: dict[str, Any] = {
            "id": aid,
            "type": m_row["type"],
            "category": m_row["category"],
            "material_class": m_row["material_class"],
            "alternative_material_classes": list(m_row["alternative_material_classes"]),
            "confidence": m_row["confidence"],
            "identity": s_row["identity"],
            "flags": flags,
            "size_m": {
                "value": list(sz_raw["value"]),
                "unit": sz_raw["unit"],
                "status": sz_raw["status"],
                "sources": list(sz_raw.get("sources") or []),
            },
            "mass_by_fill": {
                "empty": emp_mass_band,
                "full": ful_mass_band,
            },
            "inertia_over_mass_by_fill": {
                "empty": {
                    "value": list(iom_sh),
                    "unit": "m^2",
                    "status": "estimated",
                    "basis": f"Exact closed-surface thin-shell I/m over outer convex hull ({u_row['shell_over_solid_ratio']}x solid hull I/m; max off-diagonal ratio {u_row['shell_offdiag_max_rel']}).",
                    "form_limit": "Uniform areal wall density over outer convex hull; cap/neck/base thickness variation unmeasured (#33).",
                    "sources": usd_src_ref,
                },
                "full": {
                    "value": iom_ful,
                    "unit": "m^2",
                    "status": "estimated",
                    "basis": "Parallel-axis composite of empty thin-shell wall mass plus uniform-density interior liquid over outer convex hull.",
                    "form_limit": "Rigid composite body; internal liquid sloshing omitted in rigid-body PhysX (#85).",
                    "sources": usd_src_ref,
                },
            },
            "com_offset_by_fill": {
                "empty": {
                    "value": list(com_sh),
                    "unit": "m",
                    "status": "estimated",
                    "basis": "Area-weighted thin-shell centroid over outer convex hull in rigid-body root frame.",
                    "form_limit": "Uniform areal wall density; cap/base mass concentration unmeasured (#33).",
                    "sources": usd_src_ref,
                },
                "full": {
                    "value": com_ful,
                    "unit": "m",
                    "status": "estimated",
                    "basis": "Mass-weighted centroid of thin-shell wall plus interior liquid volume.",
                    "form_limit": "Assumes full liquid fill without headspace sloshing (#85).",
                    "sources": usd_src_ref,
                },
            },
            "solid_collider_inertia_over_mass_m2": {
                "value": list(iom_so),
                "unit": "m^2",
                "status": "sourced",
                "basis": "Exact uniform-density tetrahedral integral over outer convex hull of USD mesh.",
                "sources": usd_src_ref,
            },
        }

        if "material_override" in s_row:
            mo = s_row["material_override"]
            rbf_mo = mo["restitution_by_fill"]
            mo_st = (
                "estimated"
                if (
                    mo["mu_static_unary"]["status"] == "estimated"
                    and mo["mu_kinetic_unary"]["status"] == "estimated"
                )
                else "unsourced"
            )
            a_out["material_override"] = {
                "static_friction": float(mo["mu_static_unary"]["value"]),
                "dynamic_friction": float(mo["mu_kinetic_unary"]["value"]),
                "friction_combine_mode": str(mo["friction_combine_mode"]),
                "restitution_combine_mode": "average",
                "status": mo_st,
                "notes": str(mo["reason"]),
                "restitution_by_fill": {
                    "empty": {
                        "value": float(rbf_mo["empty"]["value"]),
                        "unit": "dimensionless",
                        "status": "unsourced",
                        "basis": "Empty coated stainless-steel bottle restitution from #84/#91 note 1.",
                    },
                    "full": {
                        "value": float(rbf_mo["full"]["value"]),
                        "unit": "dimensionless",
                        "status": "unsourced",
                        "basis": "Liquid-filled coated stainless-steel bottle restitution from #84/#91 note 1.",
                    },
                },
            }
        out_assets[aid] = a_out

    # Geometry/dustbin/00000 (static open receptacle: #91 Note 1 keeps polyolefin_plastic empty restitution 0.25)
    db_m = models_by_id["Geometry/dustbin/00000"]
    db_s = slice_src["assets"]["Geometry/dustbin/00000"]
    out_assets["Geometry/dustbin/00000"] = {
        "id": "Geometry/dustbin/00000",
        "type": db_m["type"],
        "category": db_m["category"],
        "material_class": db_m["material_class"],
        "alternative_material_classes": list(db_m["alternative_material_classes"]),
        "confidence": db_m["confidence"],
        "identity": db_s["identity"],
        "flags": [*db_m["flags"], "open_receptacle"],
        "size_m": {
            "value": list(db_s["size_m"]["value"]),
            "unit": db_s["size_m"]["unit"],
            "status": db_s["status"] if "status" in db_s else db_s["size_m"]["status"],
            "sources": list(db_s["size_m"].get("sources") or []),
        },
    }

    # Geometry/camera_stand/00000
    cs_m = models_by_id["Geometry/camera_stand/00000"]
    cs_u = usd_src["geometry_objects"]["Geometry/camera_stand/00000"]
    cs_meta_url = f"https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/resolve/{HF_REVISION}/Assets/Object/RoboDojo/Geometry/camera_stand/00000/metadata.json"
    cs_usd_url = (
        f"https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/resolve/{HF_REVISION}/{cs_u['usd_relpath']}"
    )
    out_assets["Geometry/camera_stand/00000"] = {
        "id": "Geometry/camera_stand/00000",
        "type": cs_m["type"],
        "category": cs_m["category"],
        "material_class": cs_m["material_class"],
        "alternative_material_classes": list(cs_m["alternative_material_classes"]),
        "confidence": cs_m["confidence"],
        "identity": cs_m["identity"],
        "flags": list(cs_m["flags"]),
        "size_decision": "asset",
        "size_m": {
            "value": list(cs_u["metadata_aligned_bbox_extents_m"]),
            "unit": "m",
            "status": "sourced",
            "sources": [
                {
                    "url": cs_meta_url,
                    "quote": f"sha256={cs_u['metadata_sha256']} aligned_bbox.extents={cs_u['metadata_aligned_bbox_extents_m']}",
                    "conditions": f"Matches exact transformed USD collider vertices {cs_u['collider_union_extents_m']} and PhysX engine overlap_box bounds within 0.031% (0.035 mm); BBoxCache rotated-OBB corner bound {cs_u['bbox_cache_rotated_obb_extents_m']} was a 4.37% 45-deg child-OBB projection artifact.",
                },
                {
                    "url": cs_usd_url,
                    "quote": f"sha256={cs_u['usd_sha256']} exact transformed vertex union of {cs_u['enabled_gprim_collider_count']} enabled Mesh colliders min={cs_u['collider_union_min_m']} max={cs_u['collider_union_max_m']} extents={cs_u['collider_union_extents_m']}",
                    "conditions": "Verified in PhysX engine via 6-sided overlap_box bisection ([0.3, 0.58751, 0.109665] m).",
                },
            ],
        },
    }

    # Scene/Table, Scene/Ground, Scene/Room
    tb_s = slice_src["assets"]["Scene/Table"]
    gr_s = slice_src["assets"]["Scene/Ground"]
    out_assets["Scene/Table"] = {
        "id": "Scene/Table",
        "type": "static",
        "category": "Table",
        "material_class": tb_s["material_class"],
        "alternative_material_classes": list(tb_s["alternative_material_classes"]),
        "confidence": tb_s["confidence"],
        "identity": tb_s["identity"],
        "size_m": {
            "value": list(tb_s["size_m"]["value"]),
            "unit": tb_s["size_m"]["unit"],
            "status": tb_s["size_m"]["status"],
            "sources": list(tb_s["size_m"].get("sources") or []),
        },
        "collision": {
            "enabled": True,
            "approximation": "none",
            "status": "sourced",
            "sources": [
                {
                    "url": "env/scene_manager/objects/table.py:64-79",
                    "quote": "CreateMeshPrimCommand prim_type='Cube'",
                    "conditions": "Procedural axis-aligned box mesh requires exact mesh collision ('none') rather than convexDecomposition.",
                }
            ],
        },
    }
    out_assets["Scene/Ground"] = {
        "id": "Scene/Ground",
        "type": "static",
        "category": "Ground",
        "material_class": gr_s["material_class"],
        "confidence": gr_s["confidence"],
        "identity": gr_s["identity"],
        "size_m": {
            "value": list(gr_s["size_m"]["value"]),
            "unit": gr_s["size_m"]["unit"],
            "status": gr_s["size_m"]["status"],
            "sources": list(gr_s["size_m"].get("sources") or []),
        },
        "collision": {
            "enabled": True,
            "approximation": "none",
            "status": "sourced",
            "sources": [
                {
                    "url": "env/scene_manager/objects/ground.py:290-305",
                    "quote": "set_collision_approximation('none')",
                    "conditions": "Shipped and corrected static floor box collider.",
                }
            ],
        },
    }
    out_assets["Scene/Room"] = {
        "id": "Scene/Room",
        "type": "static",
        "category": "Room",
        "material_class": "ground",
        "confidence": "low",
        "identity": "Simple_Room_nolight static architectural shell (50 enabled Gprim colliders at scale 0.5, floor top z = +0.0152 m).",
        "notes": "Uses class 'ground' unary material so bottles dropped inside the room footprint see the same floor friction/combine mode as Scene/Ground.",
    }

    # 3b. Assets of the other tasks in scope (#63 lane 2 slices, #167). Rigid objects take mass_kg and fill-independent
    # COM and I/m; shared fixtures must agree with the bottle task's rows; the table gets a per-task entry.
    usd_cb_rigid = usd_cb_src["rigid_objects"]
    for tname_a, sl in sorted(task_slices.items()):
        if tname_a == "put_bottles_into_dustbin":
            continue
        for aid, sa in sorted(sl["assets"].items()):
            if aid.startswith("Rigid/"):
                if aid in out_assets:
                    raise RuntimeError(f"Asset {aid} appears in more than one task slice")
                out_assets[aid] = _solid_rigid_asset(aid, sa, usd_cb_rigid[aid])
            elif aid == "Scene/Table":
                if sa["material_class"] != out_assets[aid]["material_class"]:
                    raise RuntimeError(f"{tname_a} table class {sa['material_class']} != {out_assets[aid]['material_class']}")
                tv = out_assets[aid].setdefault("task_variants", {})
                tv[tname_a] = {
                    "reason": f"{tname_a} uses its own table box (layout scale, identical in every layout of the task); the base size_m is put_bottles_into_dustbin's.",
                    "size_m": {
                        "value": list(sa["size_m"]["value"]),
                        "unit": sa["size_m"]["unit"],
                        "status": sa["size_m"]["status"],
                        "basis": sa["size_m"]["basis"],
                        "sources": list(sa["size_m"].get("sources") or []),
                    },
                }
            elif aid in ("Scene/Ground", "Geometry/camera_stand/00000"):
                if sa["material_class"] != out_assets[aid]["material_class"]:
                    raise RuntimeError(f"{tname_a} {aid} class {sa['material_class']} != {out_assets[aid]['material_class']}")
                if aid == "Geometry/camera_stand/00000" and list(sa["size_m"]["value"]) != list(out_assets[aid]["size_m"]["value"]):
                    raise RuntimeError(f"{tname_a} camera stand size differs from the bottle task's")
            elif aid == "Robot/x5/finger_pad":
                if sa["material_class"] != slice_src["scenarios"]["default"]["finger_pad"]:
                    raise RuntimeError(f"{tname_a} finger pad class {sa['material_class']} differs from the bottle task's")
            else:
                raise RuntimeError(f"Unsupported asset {aid} in slice {tname_a}")

    # 3c. Assets and table variants of the 14 T1 population configs (#59, #63 lane 3, #175 decisions).
    for aid_t1, tr_t1 in sorted(t1_assets_src.items()):
        if aid_t1 in out_assets:
            if tr_t1["material_class"] != out_assets[aid_t1]["material_class"]:
                raise RuntimeError(
                    f"Shared T1 asset {aid_t1} class {tr_t1['material_class']} != {out_assets[aid_t1]['material_class']}"
                )
            if aid_t1 == "Rigid/t/00000":
                if _d(tr_t1["mass_kg"]["value"]) != _d(out_assets[aid_t1]["mass_kg"]["value"]):
                    raise RuntimeError("Shared T1 asset Rigid/t/00000 mass differs from swap_T slice")
                if list(tr_t1["size_m"]["value"]) != list(out_assets[aid_t1]["size_m"]["value"]):
                    raise RuntimeError("Shared T1 asset Rigid/t/00000 size_m differs from swap_T slice")
            continue
        out_assets[aid_t1] = _t1_asset_row(aid_t1, tr_t1, usd_t1_src)
    for tname_t1 in t1_tasks_list:
        ts_t1 = t1_task_census[tname_t1]["table_size_m"]
        tv_t1 = out_assets["Scene/Table"].setdefault("task_variants", {})
        tv_t1[tname_t1] = {
            "reason": f"{tname_t1} uses its own table box (layout scale, identical in every layout of the task); the base size_m is put_bottles_into_dustbin's.",
            "size_m": {
                "value": list(ts_t1["value"]),
                "unit": ts_t1["unit"],
                "status": ts_t1["status"],
                "basis": ts_t1["basis"],
                "sources": list(ts_t1.get("sources") or []),
            },
        }

    # 3d. Assets and table variants of the 20 T2 population configs (#59, #63 lane 3, #175).
    t2_new_asset_ids: set[str] = set()
    for aid_t2, tr_t2 in sorted(t2_assets_src.items()):
        if aid_t2 in out_assets:
            if tr_t2["material_class"] != out_assets[aid_t2]["material_class"]:
                raise RuntimeError(
                    f"Shared T2 asset {aid_t2} class {tr_t2['material_class']} != {out_assets[aid_t2]['material_class']}"
                )
            if aid_t2 == "Rigid/t/00001":
                if _d(tr_t2["mass_kg"]["value"]) != _d(out_assets[aid_t2]["mass_kg"]["value"]):
                    raise RuntimeError("Shared T2 asset Rigid/t/00001 mass differs from swap_T slice")
                if list(tr_t2["size_m"]["value"]) != list(out_assets[aid_t2]["size_m"]["value"]):
                    raise RuntimeError("Shared T2 asset Rigid/t/00001 size_m differs from swap_T slice")
            continue
        out_assets[aid_t2] = _t2_asset_row(aid_t2, tr_t2, usd_t2_src, t2_q_tags)
        t2_new_asset_ids.add(aid_t2)
    for tname_t2 in t2_tasks_list:
        ts_t2 = t2_task_census[tname_t2]["table_size_m"]
        tv_t2 = out_assets["Scene/Table"].setdefault("task_variants", {})
        tv_t2[tname_t2] = {
            "reason": f"{tname_t2} uses its own table box (layout scale, identical in every layout of the task); the base size_m is put_bottles_into_dustbin's.",
            "size_m": {
                "value": list(ts_t2["value"]),
                "unit": ts_t2["unit"],
                "status": ts_t2["status"],
                "basis": ts_t2["basis"],
                "sources": list(ts_t2.get("sources") or []),
            },
        }

    for aid_c, a_c in out_assets.items():
        if a_c.get("material_class") is not None and a_c["material_class"] not in task_cls_set:
            raise RuntimeError(f"Asset {aid_c} class {a_c['material_class']} outside classes in scope {task_classes}")

    # Record per-asset reviewed-task membership so PhysicsParamsTable derives and validates pair co-occurrence directly
    # from the tasks' spawned objects (#175 review item 3).
    all_reviewed_task_names = [t for t, _, _ in PINNED_TASK_SLICES] + list(t1_tasks_list) + list(t2_tasks_list)
    pop_cfg_by_name = {str(c["task_name"]): c for c in pop_src["configs"]}
    asset_tasks_map: dict[str, set[str]] = {aid_k: set() for aid_k in out_assets}
    for tname_sl, _, _ in PINNED_TASK_SLICES:
        for aid_sl in (task_slices[tname_sl].get("assets") or {}):
            if aid_sl in asset_tasks_map and not aid_sl.startswith("Scene/"):
                asset_tasks_map[aid_sl].add(tname_sl)
        for og_sl in pop_cfg_by_name[tname_sl]["assets"]["objects"]:
            for idx_sl in og_sl["indices"]:
                aid_pop = f"{og_sl['kind']}/{og_sl['category']}/{int(idx_sl):05d}"
                if aid_pop in asset_tasks_map:
                    asset_tasks_map[aid_pop].add(tname_sl)
    for aid_tr, tr_row in t1_assets_src.items():
        if aid_tr in asset_tasks_map:
            asset_tasks_map[aid_tr].update(str(x) for x in (tr_row.get("tasks_in_t1") or ()))
    for aid_tr2, tr2_row in t2_assets_src.items():
        if aid_tr2 in asset_tasks_map:
            asset_tasks_map[aid_tr2].update(str(x) for x in (tr2_row.get("tasks_in_t2") or ()))
    for fx_k in ("Scene/Table", "Scene/Ground", "Scene/Room"):
        if fx_k in asset_tasks_map:
            asset_tasks_map[fx_k].update(all_reviewed_task_names)
    for aid_k, a_entry in out_assets.items():
        ts_sorted = sorted(asset_tasks_map[aid_k])
        if not ts_sorted:
            raise RuntimeError(f"Asset {aid_k} has empty reviewed-task membership")
        a_entry["tasks"] = ts_sorted

    # 4. Build robots.json (all 11 X5 RigidBody links = all 42 shapes covered: 0 shapes left at defaultMaterial)
    x5_info = usd_src["robots"]["x5"]
    pad_links = {"link7", "link8"}
    non_pad_map: dict[str, Any] = {}
    for lrec in x5_info["links"]:
        lname = lrec["link"]
        if lname in pad_links:
            continue
        assigned_cls = "abs_device_housing" if lname == "camera" else "aluminium"
        non_pad_map[lname] = {
            "value": assigned_cls,
            "status": "unsourced",
            "basis": (
                "Wrist RGB-D camera shell assigned to abs_device_housing (#73 class taxonomy); surface friction unmeasured."
                if lname == "camera"
                else "ARX X5 structural arm/bracket link assigned to aluminium (#73 class taxonomy: CNC aluminium / carbon-fibre links map to nearest metal class); surface finish unmeasured."
            ),
            "sources": [
                {
                    "url": f"https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/resolve/{HF_REVISION}/{x5_info['usd_relpath']}",
                    "quote": f"sha256={x5_info['usd_sha256']} prim={lrec['prim_path']} approximations={lrec['approximations']}",
                    "conditions": "Replaces /physicsScene/defaultMaterial (0.5/0.5/0.0) on all 40 non-pad X5 collider shapes.",
                }
            ],
        }

    # 4b. Derive formable task pairs from population.json + assets + X5 links + fixtures, and fit Variant A unaries (#62 B1/B5)
    per_task_formability: list[dict[str, Any]] = []
    for tname_f, _, _ in PINNED_TASK_SLICES:
        sl = task_slices[tname_f]
        slot_models_f: list[str] | None = None
        census = (sl.get("params_diff") or {}).get("layout_census") or {}
        if tname_f != "put_bottles_into_dustbin":
            n_files = int(census["layout_files_3_seeds"])
            slot_models_f = []
            for lab, srow in sorted(census["slots"].items()):
                if not str(srow["model"]).startswith("Rigid/"):
                    continue
                if int(srow["instances"]) != n_files:
                    raise RuntimeError(f"{tname_f}: slot {lab} present in {srow['instances']} of {n_files} layouts")
                slot_models_f.append(str(srow["model"]))
        per_task_formability.append(
            derive_task_formable_pairs(
                pop_src,
                task_name=tname_f,
                out_assets=out_assets,
                out_pairs=out_pairs,
                x5_links=x5_info["links"],
                non_pad_map=non_pad_map,
                default_pad_cls=str(sl["scenarios"]["default"]["finger_pad"]),
                default_table_cls=str(sl["scenarios"]["default"]["table"]),
                slot_models=slot_models_f,
            )
        )
    for tname_t1f in t1_tasks_list:
        per_task_formability.append(
            derive_t1_task_formable_pairs(
                pop_src,
                task_name=tname_t1f,
                task_census=t1_task_census[tname_t1f],
                out_assets=out_assets,
                x5_links=x5_info["links"],
                non_pad_map=non_pad_map,
                default_pad_cls=str(slice_src["scenarios"]["default"]["finger_pad"]),
                default_table_cls=str(slice_src["scenarios"]["default"]["table"]),
            )
        )
    for tname_t2f in t2_tasks_list:
        per_task_formability.append(
            derive_t1_task_formable_pairs(
                pop_src,
                task_name=tname_t2f,
                task_census=t2_task_census[tname_t2f],
                out_assets=out_assets,
                x5_links=x5_info["links"],
                non_pad_map=non_pad_map,
                default_pad_cls=str(slice_src["scenarios"]["default"]["finger_pad"]),
                default_table_cls=str(slice_src["scenarios"]["default"]["table"]),
            )
        )
    formability = combine_task_formability(per_task_formability)
    census_weights = derive_census_presence_weights(
        contact_censuses,
        task_order=tuple(t for t, _, _ in PINNED_CONTACT_CENSUSES),
        arm="corrected",
    )
    u_static, u_kinetic, fit_meta, realised_vs_nominal = solve_nominal_tracking_unaries(
        out_pairs,
        task_classes,
        u_fmode,
        out_assets,
        formability,
        census_weights=census_weights,
        t1_uncensused_classes={str(tr["material_class"]) for tr in t1_assets_src.values()},
    )
    _attach_restitution_to_realised_table(realised_vs_nominal, out_classes, out_pairs, out_assets)

    # robots.json alone also records the D3 gripper input, so the other three files stay byte-identical.
    robots_provenance = json.loads(json.dumps(provenance_block))
    robots_provenance["input_checksums_sha256"]["audit/out/d3/gripper_sources.json"] = gripper_src_sha
    robots_provenance["input_checksums_sha256"]["audit/out/d3/arm_limits_sources.json"] = arm_src_sha
    robots_doc: dict[str, Any] = {
        "schema": "v3",
        "schema_version": "d2-pr2-v1",
        "provenance": robots_provenance,
        "status_vocabulary": ["sourced", "estimated", "scenario", "unsourced", "test_only"],
        "robots": {
            "x5": {
                "robot_name": "x5",
                "pad_material_class": slice_src["scenarios"]["default"]["finger_pad"],
                "non_pad_link_material_classes": non_pad_map,
                "sanitize_camera_inertials": {
                    "value": True,
                    "status": "unsourced",
                    "basis": "Replaces authored mass=0 and centerOfMass=(-inf,-inf,-inf) on camera_base and camera in ARX.usd (#48) with 1e-4 kg and (0,0,0) under disable_gravity=True.",
                    "sources": [
                        {
                            "url": f"https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/resolve/{HF_REVISION}/{x5_info['usd_relpath']}",
                            "quote": "camera_base / camera MassAPI mass=0, centerOfMass=(-inf, -inf, -inf) (#48)",
                            "conditions": "Numerical sanitization floor only; real wrist camera mass unweighed (#32).",
                        }
                    ],
                },
                "gripper": build_gripper_block(gripper_src),
                "arm_limits": build_arm_limits_block(arm_src),
                "notes": "Covers all 11 RigidBody links (42/42 PhysX collider shapes) on each ARX X5 robot. Pad torsional radius is left unset so ARX.usd's native 4-corner convexHull contact patch (r_eff = 8.26 mm, #26/#86) is preserved.",
            }
        },
    }

    reviewed_tasks = {
        tname_r: hashlib.sha256(
            json.dumps(task_slices[tname_r]["params_diff"], sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        for tname_r, _, _ in PINNED_TASK_SLICES
    }
    for tname_rt1 in t1_tasks_list:
        reviewed_tasks[tname_rt1] = hashlib.sha256(
            json.dumps(t1_task_census[tname_rt1]["params_diff"], sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
    for tname_rt2 in t2_tasks_list:
        reviewed_tasks[tname_rt2] = hashlib.sha256(
            json.dumps(t2_task_census[tname_rt2]["params_diff"], sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
    all_scope_tasks = [t for t, _, _ in PINNED_TASK_SLICES] + t1_tasks_list + t2_tasks_list
    scope_txt = (
        f"{', '.join(t for t, _, _ in PINNED_TASK_SLICES)} + 14 T1 + 20 T2 population configs "
        f"({len(all_scope_tasks)} configs total)"
    )

    classes_doc: dict[str, Any] = {
        "schema": "v3",
        "schema_version": "d2-pr2-v1",
        "description": f"Class bulk densities, restitutions, scenarios, and one global Variant A presence-weighted L2 unary friction map (v2_presence) for {scope_txt} (#62 PR 2, #167, #59; generated by audit/tools/build_docs_params.py).",
        "provenance": provenance_block,
        "status_vocabulary": ["sourced", "estimated", "scenario", "unsourced", "test_only"],
        "classes": out_classes,
        "unary_values": {
            "note": (
                f"One global Variant A nominal-tracking joint L2 unary friction map (map v2_presence; #59 option 3, #167 "
                f"decisions 2, 3 and (b)) for {scope_txt}: presence weight 1 over the {len(fit_meta['counted_primary_pairs'])} "
                f"primary-scenario pairs that carry normal load on a non-robot-self contact in the committed corrected "
                f"scripted-solution censuses (robot-link x robot-link contacts such as {sorted(census_weights['excluded_robot_self_only_pairs'])} "
                f"and finger-on-finger {sorted(fit_meta['dropped_not_load_bearing_pairs'])} excluded; {len(fit_meta['unloaded_primary_formable_pairs'])} "
                f"unloaded/robot-self formable primary pairs are band constraints only), lexicographic Stage 2 presence weight 1 "
                f"over the {len(fit_meta['counted_sensitivity_pairs'])} counterpart-loaded sensitivity pairs for "
                f"{fit_meta['stage2_classes']}, and Stage 3 feasible solve for zero-weight classes "
                f"{fit_meta['stage3_zero_weight_classes']}; {fit_meta['constraints']}; PhysX average combine."
            ),
            "fit": fit_meta,
            "static": u_static,
            "kinetic": u_kinetic,
            "friction_combine_mode": u_fmode,
            "restitution_combine_mode": u_rmode,
        },
    }

    pairs_doc: dict[str, Any] = {
        "schema": "v3",
        "schema_version": "d2-pr2-v1",
        "description": f"Canonical class-pair friction bands, >0.5 m/s impact COR bands, and realised-versus-nominal table with census evidence at the global presence-weighted map (v2_presence) for {scope_txt} (#62 PR 2, #167, #59; generated by audit/tools/build_docs_params.py).",
        "provenance": provenance_block,
        "status_vocabulary": ["sourced", "estimated", "scenario", "unsourced", "test_only"],
        "restitution_schema": cv_src["restitution_schema"],
        "camera_stand": cv_src["camera_stand"],
        "pairs": out_pairs,
        "realised_vs_nominal": realised_vs_nominal,
    }

    assets_doc: dict[str, Any] = {
        "schema": "v3",
        "schema_version": "d2-pr2-v1",
        "description": f"Per-asset intrinsic parameters (masses, shell/composite/solid inertias, extents, overrides, per-task table entries) and reviewed dormant-diff hashes for {scope_txt} (#62 PR 2, #167; generated by audit/tools/build_docs_params.py).",
        "provenance": provenance_block,
        "status_vocabulary": ["sourced", "estimated", "scenario", "unsourced", "test_only"],
        "reviewed_tasks": reviewed_tasks,
        "assets": out_assets,
    }

    computed_empty_opt_in = _collect_unsourced_keys_for_fill(classes_doc, pairs_doc, assets_doc, robots_doc, "empty")
    computed_full_opt_in = _collect_unsourced_keys_for_fill(classes_doc, pairs_doc, assets_doc, robots_doc, "full")
    rev_profiles = rev_opt_src["profiles"]
    empty_opt_in = verify_reviewed_opt_in_list(
        computed_empty_opt_in,
        rev_profiles["bottles_primary_empty"]["allow_unsourced_keys"],
        profile_name="bottles_primary_empty",
    )
    full_opt_in = verify_reviewed_opt_in_list(
        computed_full_opt_in,
        rev_profiles["bottles_stress_full"]["allow_unsourced_keys"],
        profile_name="bottles_stress_full",
    )
    # Per-task profiles of the other tasks: fill does not apply (fill=empty). The loader gates every leaf of the
    # loaded table, not only the task's, so each list equals the table-wide unsourced set at that fill.
    task_profile_names = {"cover_blocks": "cover_blocks_primary", "swap_T": "swap_T_primary"}
    task_opt_in: dict[str, list[str]] = {}
    for tname_o, _, _ in PINNED_TASK_SLICES[1:]:
        prof_name = task_profile_names[tname_o]
        task_opt_in[prof_name] = verify_reviewed_opt_in_list(
            computed_empty_opt_in,
            rev_profiles[prof_name]["allow_unsourced_keys"],
            profile_name=prof_name,
        )
    t1_opt_in = verify_reviewed_opt_in_list(
        computed_empty_opt_in,
        rev_profiles["population_t1_primary"]["allow_unsourced_keys"],
        profile_name="population_t1_primary",
    )
    t2_opt_in = verify_reviewed_opt_in_list(
        computed_empty_opt_in,
        rev_profiles["population_t2_primary"]["allow_unsourced_keys"],
        profile_name="population_t2_primary",
    )

    classes_doc["scenarios"] = {
        "default": {
            "finger_pad": "pad_hard",
            "table": "table_laminate",
            "ground_combine": "average",
            "fill": "empty",
        },
        "sensitivity": slice_src["scenarios"]["sensitivity"],
        "fill_convention": slice_src["scenarios"]["fill_convention"],
        "surface_basis": slice_src["scenarios"]["surface_basis"],
        "opt_in_profiles": {
            "bottles_primary_empty": {
                "description": "Explicit per-key opt-in for put_bottles_into_dustbin primary scenario (pad_hard + table_laminate, fill=empty, #85 default). Admits only the listed unsourced bottle masses, polymer/metal/surface restitutions, laminate/ground friction bands, and X5 non-pad link assignments; run remains below G1 until sourced or G1 wording is updated.",
                "scenario": {
                    "finger_pad": "pad_hard",
                    "table": "table_laminate",
                    "ground_combine": "average",
                    "fill": "empty",
                },
                "allow_unsourced_keys": empty_opt_in,
            },
            "bottles_stress_full": {
                "description": "Explicit per-key opt-in for put_bottles_into_dustbin liquid-filled stress case (pad_hard + table_laminate, fill=full, #85 stress case).",
                "scenario": {
                    "finger_pad": "pad_hard",
                    "table": "table_laminate",
                    "ground_combine": "average",
                    "fill": "full",
                },
                "allow_unsourced_keys": full_opt_in,
            },
            **{
                task_profile_names[tname_o]: {
                    "description": (
                        f"Explicit per-key opt-in for {tname_o} primary scenario (pad_hard + table_laminate; fill does not apply "
                        f"and stays empty). Admits only the listed unsourced keys of the loaded table (the gate is table-wide); "
                        f"run remains below G1 until they are sourced."
                    ),
                    "scenario": {
                        "finger_pad": "pad_hard",
                        "table": "table_laminate",
                        "ground_combine": "average",
                        "fill": "empty",
                    },
                    "allow_unsourced_keys": task_opt_in[task_profile_names[tname_o]],
                }
                for tname_o, _, _ in PINNED_TASK_SLICES[1:]
            },
            "population_t1_primary": {
                "description": (
                    "Explicit per-key opt-in for the 14 T1 population configs (#59, #63 lane 3, #175) under the primary "
                    "scenario (pad_hard + table_laminate, fill=empty). Admits only the reviewed unsourced keys of the "
                    "loaded table (table-wide gate; under #173 G1 = delivered and plausible with basis lines)."
                ),
                "scenario": {
                    "finger_pad": "pad_hard",
                    "table": "table_laminate",
                    "ground_combine": "average",
                    "fill": "empty",
                },
                "allow_unsourced_keys": t1_opt_in,
            },
            "population_t2_primary": {
                "description": (
                    "Explicit per-key opt-in for the 20 T2 population configs (#59, #63 lane 3, #175) under the primary "
                    "scenario (pad_hard + table_laminate, fill=empty). Admits only the reviewed unsourced keys of the "
                    "loaded table (table-wide gate; every unsourced and estimated T2 leaf carries an explicit basis line, "
                    "with Bo's #59 Q1..Q5 decisions applied)."
                ),
                "scenario": {
                    "finger_pad": "pad_hard",
                    "table": "table_laminate",
                    "ground_combine": "average",
                    "fill": "empty",
                },
                "allow_unsourced_keys": t2_opt_in,
            },
        },
    }

    verify_all_estimated_leaves_have_basis(
        {
            "classes.json": classes_doc,
            "pairs.json": pairs_doc,
            "assets.json": assets_doc,
            "robots.json": robots_doc,
        }
    )
    verify_all_t2_leaves_have_basis(
        classes_doc,
        pairs_doc,
        assets_doc,
        t2_classes=t2_only_cls_set,
        t2_pair_keys=t2_new_pair_keys,
        t2_new_asset_ids=t2_new_asset_ids,
    )

    os.makedirs(out_dir, exist_ok=True)
    for fname, doc in (
        ("classes.json", classes_doc),
        ("pairs.json", pairs_doc),
        ("assets.json", assets_doc),
        ("robots.json", robots_doc),
    ):
        fpath = os.path.join(out_dir, fname)
        with open(fpath, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=2)
            f.write("\n")

    print(
        f"Generated {out_dir}: {len(out_classes)} classes, {len(out_pairs)} pairs, "
        f"{len(out_assets)} assets, {len(empty_opt_in)} empty opt-in keys, {len(full_opt_in)} full opt-in keys, "
        f"{len(fit_meta['counted_primary_pairs'])} primary + {len(fit_meta['counted_sensitivity_pairs'])} sensitivity fit pairs"
    )


if __name__ == "__main__":
    main()
