import json,sys
for t in sys.argv[1:]:
    try: d=json.load(open(f"/scratch/<user>/isaac/harness_runs/{t}/result.json"))
    except Exception as e: print(t,"missing"); continue
    for r in d["runs"]:
        print(t,"L",r["layout"],"score",r.get("episode_score_pct"),"succ",r.get("success"),"acts",r.get("take_action_cnt"),"inbin_shipped",r.get("bottles_in_bin_shipped_check"),"credited",{k:v["credited"] and v["in_bin"] for k,v in r.get("pickup_ledger_v2",{}).items()},"ee",r.get("pickup_ee_links"),"err",r.get("run_eval_error"))
        print("  trace",json.dumps(r.get("pickup_trace")))
