# Population sweep: predictions made before any run (November 12, 2026)

Shipped physics (`--physics_preset shipped`, evaluator v1 as shipped), seed 0, layout 0, all 54 configs of `audit/out/31/population.json`, `run_harness.py --policy sweep` on della-vis2 GPU 0 (another user's vLLM job holds about 43 of 46 GB there). Static basis: `audit/out/e1_census.md` (#29) and the issues it filed (#42 to #49).

| Check | Prediction (number, band) | Decision each outcome changes |
|---|---|---|
| Configs that build and reset | 46 of 54 (band 40 to 52). Most likely failures: GPU memory for the fluid and cloth configs (6), conveyor (2). The three support-arm configs build and reset but cannot be scored (`Assets/Traj` pickles refused) | Each failure is recorded with its exact error; a RoboDojo-side failure becomes an issue, an environment failure (GPU memory) is retried when the GPU frees up |
| Rigid engine mass equals `min(declared or 0.5, 0.5)`, floor 0.05 | every rigid instance, within 1e-5 kg | Any mismatch is a new delivery fault (issue) |
| Rigid engine materials on cpu | every shape 0.5 / 0.5 / 0 (#58) | Any other value means #58 does not hold everywhere |
| Rigid objects with no enabled collider | 0 (band 0 to 3) | A collider-less object falls through: issue |
| Rigid objects moving more than 5 mm during a 4 s no-action settle after reset | 3 (band 0 to 15 of about 400) | Each mover is classified (rolling, sliding, sinking, pushed by a neighbour) |
| Objects falling below the table top by more than 5 cm, or flying off (speed above 1 m/s) | 0 (band 0 to 2) | Any case is a top blog candidate |
| Declared spawn to post-reset displacement: horizontal above 2 cm | 2 (band 0 to 8); vertical drops below 2 cm are expected (spawn margin) | Large horizontal moves during reset mean spawn interpenetration or instability that RoboDojo's own check (4 cm, 30 degrees, after the first 20 steps) tolerates |
| Contact pairs penetrating more than 5 mm at rest | 1 (band 0 to 5) | Penetration at rest means spawn overlap or thin colliders |
| Do-nothing success (one hold action, then the step-limit check) | 0 of the configs that can be scored (band 0 to 2) | Any success is a top scoring exploit (E3 issue) |
| Do-nothing partial score above 0 | 1 (band 0 to 4) | Partial credit for inaction is an E3 issue |
| Articulated-object joints moving more than 5% of their range with no action | 1 of about 7 (band 0 to 3) | A drifting door, lid or button is a blog candidate |
| Robot joints drifting more than 0.01 rad with no action | 0 (band 0 to 1 configs) | Drift means the hold drives cannot carry the arm |
| Cloth or fluid points that are non-finite, or fluid particles below the table top | 0 configs (band 0 to 2) | Either is a blog candidate |
| Declared `size` against engine collider extents, off by more than 5% | about a third of the instances that declare a size (#46: 52 of 157) | Confirms #46 from the engine side |
| GPU time | 0.8 GPU-hours (band 0.5 to 1.5) | Above 2 stops the sweep for review |
