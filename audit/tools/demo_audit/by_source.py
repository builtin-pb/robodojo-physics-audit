"""Group parquet_audit.json by the repo's per-task data_source (task/RoboDojo/config/_task.yml via audit/out/31/population.json). Usage: python3 -I by_source.py <parquet_audit.json> <population.json>"""
import json, sys
R = json.load(open(sys.argv[1]))["per_episode"]; P = {c["task_name"]: c["scene"]["data_source"] for c in json.load(open(sys.argv[2]))["configs"]}
NAMES = "arrange_largest_number build_tower classify_objects cover_blocks deposit_coin dlc fasten_screws fill_egg_holder fill_pen_holder fold_clothes hang_mugs imitate_sorting_sequence insert_key insert_tubes make_kong make_toast match_and_pick_from_conveyor organize_table pack_objects_into_box play_Xylophone play_stacking_toy play_tic_tac_toe plug_in_charger pour_balls_into_vase pour_liquid_into_cup press_by_number push_T put_bottles_into_dustbin sort_nesting_dolls_by_size stack_blocks stack_bowls store_laptop_and_headphones swap_T swap_blocks sweep_blocks".split()
src = {i: P.get(n, "datagen? (dlc not in population)") for i, n in enumerate(NAMES)}
for s in sorted(set(src.values())):
    rs = [r for r in R if src[r["task"]] == s]; fr = sum(r["len"] for r in rs)
    print(s, "tasks", sorted(t for t in src if src[t] == s), "episodes", len(rs), "frames", fr)
    print("   episodes beyond ARX limits", sum(r["viol_frames"] > 0 for r in rs), f'({100 * sum(r["viol_frames"] > 0 for r in rs) / len(rs):.1f}%)', "frames", sum(r["viol_frames"] for r in rs), f'({100 * sum(r["viol_frames"] for r in rs) / fr:.2f}%)')
