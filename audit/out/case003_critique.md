# Case 003 critique (fresh critic, read-only)

**Verdict: supported only for the harness, and worded too broadly.** The evidence comes only from the harness. But the code points to a cause in unmodified shared code, so the bottle result very likely holds in evaluation too. The same cause predicts exceptions to "no material bound at runtime" and to "every contact".

## 1. Order of material creation, binding and PhysX parsing

- Evaluation runs on device `cpu` (`base_env.py:15`; no config overrides it) and on Isaac Sim 5.1 (`Dockerfile:156`).
- On cpu, `scene_manager.initialize` spawns only articulations (`scene_manager.py:124-133`). Rigid objects are spawned in `reload_env_scene` (`:238-246`), which `TaskEnv.reset` calls after `sim.reset()` (`task_env.py:83-85`). By then the physics view exists.
- `RigidObject.__init__` runs in this order: reference the asset, create the material prim, `SingleGeometryPrim.__init__`, `SingleRigidPrim.__init__`, then bind in `_setup_physics` (`rigid.py:33-100,215-216`).
- In Isaac Sim 5.1, `RigidPrim.__init__` calls `SimulationManager._physx_sim_interface.flush_changes()` whenever a physics view exists, then creates a tensor view (`isaacsim.core.prims/python/impl/rigid_prim.py:200-202`, tag v5.1.0). That flush makes omni.physx parse the new bottle before any binding exists, so its shape gets the default material. The binding then arrives as an edit to a body PhysX has already parsed.
- `set_mass` writes through the tensor view after the flush. That is why the engine honours the masses (0.15, capped 0.5) but not the material.
- This sequence is RoboDojo and Isaac Sim code, which the harness runs unmodified. No harness deviation acts between referencing and binding:
  - The camera manager builds its prims before `sim.reset()` and takes its one step after construction.
  - No render runs inside the constructor, and turning render emulation off left poses identical.
  - The launcher bypass changes only the offscreen and camera flags.
  - The policy runs after construction.
  - The contact-report wrapper applies its API after the binding.
- Evaluation could differ only through an extension in `isaaclab.python.kit` that re-parses bodies after construction. omni.physx is the same binary in both kits, and adding `omni.physx.bundle` changed nothing. I know of no such extension, but nobody has tested the full kit.

## 2. What omni.physx does with bindings authored after parse

NVIDIA's documentation says only that omni.physx applies "most" USD changes during simulation ([Simulation Control, 107.3](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/simulation_control/simulation_control.html)). It gives no rule for material bindings. `flush_changes()` exists because changes are buffered until the next step or flush. So the data here are the direct evidence:

- An ancestor binding authored after parse never took effect. That held in the probe (first builds) and in the harness (the bottles, plus both contrasts, with and without the bundle).
- A binding that existed at parse always took effect.

## 3. Internal consistency

- The three readouts agree: tensor readback 0.5/0.5, contact paths `/physicsScene/defaultMaterial` on both sides, and a slide of 24.55 mm against 25.5 mm predicted. The probe's known pairs land at 0.95-0.975 of the Coulomb prediction, and 24.55/25.5 falls in that range.
- **The probe artefact has a simpler cause than build order.** `World.clear_instance()` keeps the stage. So the binding that build 1 authored after reset is already present before reset in every later build. The data show it: in `probe_case003.json`, the "realistic" condition binds no finger material, yet its finger contacts report `/World/materials/finger`. The probe and the harness therefore follow one rule: a binding present at parse counts, and one added later does not. That rule answers #57's open question.
- **Weaknesses:**
  - Both harness contrasts are also bound after parse. They repeat the mechanism rather than test the kit.
  - No binding has yet been shown to be honoured in the harness, so a positive control is missing.
  - The fingers carry no binding at all.
  - The static table is unbound by design (`table.py:206-211`).
  - The dustbin's material is created but never bound.

## 4. Where the claim overreaches

- **Ground contacts.** By the rule, a binding counts if nothing flushes between creating the collider and binding it. The ground fits: `SingleGeometryPrim` does not flush before `apply_physics_material` (`ground.py:289-296`). So its 1.0 multiply material should reach PhysX at the next flush. Nobody has measured it, so "every contact runs at 0.5/0.5" is unsupported for ground contacts.
- **Device.** The bottle result applies only on cpu. On cuda, rigid objects are spawned before `sim.reset()`, when there is no physics view and so no flush.

## 5. Cheapest decisive check

One short harness run on Della, needing no RTX:

1. **Positive control.** Move a bottle onto the ground and read the contact material path. The slide cannot decide this, because multiply against 0.5 also gives 0.5. Expect the ground's material.
2. **Order variant.** Bind before `SingleRigidPrim.__init__`, or no-op `flush_changes` during construction. Expect 10,000 in the readback, the bottle's own material in contacts, and a slide of about 0 mm. That result pins the cause to flush-then-bind in shared code.
3. **Optional standalone script.** Use a fresh stage per case (`new_stage()`) and compare binding before and after `RigidPrim` while a physics view is live.

If checks 1 and 2 match these predictions, only an unknown re-parsing extension in the full kit remains. One unmodified evaluation with this readback, on a machine whose driver runs RTX, would settle that.
