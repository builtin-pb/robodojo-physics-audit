"""Share of py-spy main-thread samples per category (first matching rule per stack): python3 -I pyspy_categories.py <raw>"""
import sys, re, collections
# categories by first matching frame (outermost-first scan of each stack, most specific rule wins by order)
RULES = [
    ("script IK (x5_ik solve_ik/ik/fk)", re.compile(r"\b(solve_ik|ik|fk|axis_angle)\b \((script_bottles|x5_ik)\.py")),
    ("PhysX step (SimulationContext.step)", re.compile(r"_step \(api/physics_context/physics_context\.py|step \(isaaclab/sim/simulation_context\.py")),
    ("Isaac Lab scene write/update", re.compile(r"(write_data_to_sim|update) \(.*(interactive_scene|articulation|rigid_object)")),
    ("RoboDojo reward/evaluator", re.compile(r"reward_manager\.py|func_parser\.py|evaluator_v2\.py")),
    ("RoboDojo control (push/pop, robot writes)", re.compile(r"(control_manager|robot_manager)\.py|process_control_info|take_action_batch \(eval_env\.py")),
    ("script trace logging (act's readouts)", re.compile(r"(gripper_readout|bottle_state|read_contacts|tracked_step) \(script_bottles")),
    ("render emulation (forward + app.update)", re.compile(r"_render_as_eval")),
    ("script planning other", re.compile(r"script_bottles\.py")),
]
tot = 0; cats = collections.Counter()
for line in open(sys.argv[1], errors="replace"):
    m = re.match(r"^(.*) (\d+)$", line.rstrip("\n"))
    if not m: continue
    frames = m.group(1).split(";")[1:]; n = int(m.group(2)); tot += n
    hit = "other"
    for name, rx in RULES:
        if any(rx.search(f) for f in frames):
            hit = name; break
    cats[hit] += n
for k, v in cats.most_common():
    print(f"{100 * v / tot:5.1f}%  {k}")
print("samples", tot)
