"""Write the G4 run queue for put_bottles_into_dustbin (one line per harness process: `tag|args`).

usage: python3 -I audit/harness/g4/make_queue.py <repo on Della> <queue out> <arm>[,<arm>...] [--seeds=0,1,2] [--layouts=10-16]
       [--suffix=_x] [--evaluator=v2]

Every episode runs the frozen `script_bottles` (tag `script-bottles-v2`) under evaluator v2 (or `--evaluator=v1`) with
`--fast_log --contact_processing_off --bottle_trace` (final poses identical to the script's own settings, `g4t_ship_s0_l10*`).
One process per (arm, seed) runs that seed's layouts in turn.

Arms: `shipped`; `corrected` (all seven fixes, D3's gripper, #11's arm limits and the solver profile included);
one change from shipped (`ordval`: order with sourced values; `materials`; `masses`; `geom`: size and collision;
`gripper`; `limits`: robot_limits; `solver`); `corr_nosolver` (corrected without the solver profile, for the solver
checks; `corr_nolimits`, `corr_nogripper` likewise: leave one out). Levels at corrected physics: pad friction (`pad*`: pad unary so that the pad-PET pair under `average` is the
named pair; `pmin*`: pad unary with combine `min`, so every pad pair is the pad value, below what `average` can reach),
bottle masses at the ends of their empty ranges (`light`, `heavy`), and physics dt 2 ms (`dt2`).
"""
import sys

ARMS = {
    # arm: (preset, fixes or None, needs table)
    "shipped": ("shipped", None, False),
    "corrected": ("corrected", None, True),
    "corr_nosolver": ("corrected", "-solver", True),
    "corr_nolimits": ("corrected", "-robot_limits", True),
    "corr_nogripper": ("corrected", "-gripper", True),
    "noorder": ("corrected", "values,size,collision", True),
    "ordval": ("shipped", "order,values", True),
    "materials": ("shipped", "order,materials", True),
    "masses": ("shipped", "masses", True),
    "geom": ("shipped", "size,collision", False),
    "gripper": ("shipped", "gripper", True),
    "limits": ("shipped", "robot_limits", True),
    "solver": ("shipped", "solver", False),
}
PADS = {  # pad unary (static, dynamic[, combine]); pad-PET pair in the comment (PET unary 0.26744/0.2217)
    "pad020": (0.13256, 0.0783),    # pair 0.20/0.15 under average: pad_hard band low end
    "pad024": (0.21256, 0.1783),    # pair 0.24/0.20
    "pad035": (0.43256, 0.3783),    # pair 0.35/0.30
    "pad050": (0.73256, 0.5783),    # pair 0.50/0.40: pad_hard band high end
    "pad074": (1.21583, 1.037567),  # pair 0.742/0.630: pad_elastomer unary (realised elastomer nominal)
    "pad015": (0.03256, 0.0283),    # pair 0.15/0.125: below the sourced range
    "pad010": (0.0, 0.0),           # pair 0.134/0.111: pad frictionless, the floor of `average`
    "pmin015": (0.15, 0.1125, "min"),   # every pad pair 0.15/0.1125
    "pmin010": (0.10, 0.075, "min"),    # every pad pair 0.10/0.075
    "pmin005": (0.05, 0.0375, "min"),   # 0.05/0.0375
    "pmin0025": (0.025, 0.01875, "min"),  # 0.025/0.019
    "pmin00125": (0.0125, 0.009375, "min"),  # 0.0125/0.0094
}
LEVELS = {  # other levels at corrected physics: extra harness flags
    "light": "--bottle_mass_bound low",
    "heavy": "--bottle_mass_bound high",
    "dt2": "--episode_dt 0.002",
    "dt4": "--episode_dt 0.004",  # control: must equal `corrected`
}


def rng(s):
    a, _, b = s.partition("-")
    return list(range(int(a), int(b or a) + 1))


def main(repo, out, arms, seeds="0,1,2", layouts="10-16", suffix="", evaluator="v2"):
    p = f"--physics_params {repo}/docs/params --physics_scenario_opt_in bottles_primary_empty"
    base = f"--task put_bottles_into_dustbin --policy script_bottles --evaluator {evaluator} --fast_log --contact_processing_off --bottle_trace"
    lines = []
    for arm in arms.split(","):
        if arm in PADS:
            phys = f"--physics_preset corrected {p} --pad_friction " + ",".join(str(x) for x in PADS[arm])
        elif arm in LEVELS:
            phys = f"--physics_preset corrected {p} {LEVELS[arm]}"
        else:
            preset, fixes, table = ARMS[arm]
            phys = f"--physics_preset {preset}" + (f" --physics_fixes={fixes}" if fixes else "") + (f" {p}" if table else "")
        for seed in seeds.split(","):
            lay = ",".join(str(x) for x in rng(layouts))
            ev = "" if evaluator == "v2" else f"_{evaluator}"
            lines.append(f"g4_{arm}{ev}_s{seed}_l{layouts.replace('-', 'to')}{suffix}|{base} --seed {seed} --layouts {lay} {phys}")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    a = sys.argv[1:]
    pos = [x for x in a if not x.startswith("--")]
    opts = dict(x[2:].split("=", 1) for x in a if x.startswith("--"))
    main(*pos[:3], **opts)
