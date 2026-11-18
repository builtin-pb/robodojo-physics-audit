"""Runtime draws seeded by the layout id (the env seed is the layout id: src/eval_client/main.py:459,466;
env/environment/base_env.py:268-271,288-296). Reproduces the first np.random draw after seed_everywhere(k).

play_tic_tac_toe.py:135-136: before each opponent move, _set_env_seed(env) then np.random.choice(empty_cells);
the opponent moves when 2m+1 pieces are down (m = 0..3), so len(empty_cells) = 8, 6, 4, 2.
make_kong.py:78-79: _set_env_seed(env) then np.random.choice(4, replace=False).
Usage: python3 -I runtime_draws.py
"""
from collections import Counter

import numpy as np


def first_choice(k, a, **kw):
    np.random.seed(k)  # seed_everywhere -> np.random.seed(seed) (env/seeding.py)
    return int(np.random.choice(a, **kw))


for name, n_ids in (("scored 0..49", 50), ("all 0..74", 75)):
    seqs = Counter(tuple(first_choice(k, list(range(n))) for n in (8, 6, 4, 2)) for k in range(n_ids))
    print(f"tic-tac-toe opponent index sequences over layout ids {name}: {len(seqs)} distinct; "
          f"largest class {max(seqs.values())}; first-move index alone: "
          f"{len({s[0] for s in seqs})} distinct")
    print("   ", sorted(seqs.items(), key=lambda kv: -kv[1]))
kong = Counter(first_choice(k, 4, replace=False) for k in range(50))
print("make_kong pushed-tile index over layout ids 0..49:", dict(sorted(kong.items())))
