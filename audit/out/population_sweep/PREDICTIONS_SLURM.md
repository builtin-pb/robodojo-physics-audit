# Slurm follow-up: predictions made before any run (November 13, 2026)

Six configs the della-vis2 sweep could not simulate (`fasten_screws`, `pour_by_language`, `pour_liquid_into_cup`, `pour_liquid_into_cup_random`, `fold_clothes`, `fold_clothes_random`), rerun on a Slurm GPU node (1 GPU, at most 1 h per job, under 3 GPU-hours in total). Same procedure and flags as `popsweep_v2` (settle) and `popsweep_drop` (3 cm drop), seed 0, layout 0, shipped physics, commit `462853a`.

| Check | Prediction (number, band) | Decision each outcome changes |
|---|---|---|
| Apptainer and the Isaac Sim 5.1 container start physics-only kit on a Slurm A100/H100 node | works (85%); failure modes: driver mismatch, `--nv` binding, no scratch access | If it fails, stop and report the setup blocker |
| Configs whose physics runs without "Scene state is corrupted" or GPU-kernel launch failures | 6 of 6 (band 5 to 6) | Gate 0 for those configs; more than 1 failure means the cause was not only free memory |
| `fasten_screws` control: "Scene state is corrupted" lines | 0 (band 0 to 5) | 0 supports #114's cause (free GPU memory); many contradicts it |
| `fasten_screws` control: SDF cook succeeds, nuts lifted 3 cm fall back and rest within 2 mm of start by 0.2 s | 3 of 3 | If they still hang, the freeze is not memory |
| `fasten_screws` control: gripper joints at reset | about 0.044 (0.03 to 0.05), not 0.0 | Same |
| `fasten_screws` control: rigid contacts at rest | above 0 (at least 6: 3 nuts on the table, 3 on the bolt or tray) | Same |
| `fasten_screws` peak extra GPU memory over the pre-launch baseline | about 2 GB (1.5 to 4 GB) | Sizes the #114 memory threshold |
| Settle motion, rigid objects, 4 s no-action | at most 5 mm in all 6 configs | Rigid anomalies |
| Cloth (`fold_clothes*`): garment settle motion | below 10 mm; no non-finite positions | Garment rest is stable under shipped physics |
| Fluid (`pour_*`): particles | stay finite and inside the cup or its vicinity; bottles creep less than 12 mm | The 12 mm creep seen on the starved GPU was a failure artefact |
| Do-nothing verdict | fail, score 0, in all 6 | Scoring unchanged |
| GPU time | 0.5 h (0.3 to 1.0) for settle plus drop; under 3 GPU-hours total | Budget |
