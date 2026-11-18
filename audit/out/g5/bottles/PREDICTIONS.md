# Predictions, written before launch (November 14)

Seed 0, layouts 0-39, InternW0-Δ, put_bottles_into_dustbin, NGX bound, declared render settings. Reference: the gate scored 20/20 on layouts 0-19 (shipped, v1); the published rate is 98%.

- A (shipped, v1): 38/40, band 34-40. Decision: below 34 means the 6.2 render/harness differs from the gate and the A-vs-B contrast is not interpretable.
- B (corrected, v1): 30/40, band 12-39. The demonstrations were recorded under shipped physics (gripper force, friction, limits), so the policy's grasps may fail more; no basis for a tighter band. Decision: B within 3 of A means the shipped-physics success does not depend on the faults fixed here.
- C (shipped, v2): 35/40, band 20-40; never above A by more than the v1 over-count. v2 adds the pick-up check, so it can only turn v1 successes into failures on the same trajectories (same seed and physics, up to GPU nondeterminism). Decision: C well below A means v1 counts passes the instruction does not ask for.
