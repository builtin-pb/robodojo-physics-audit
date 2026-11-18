import json, sys
R = sys.argv[1]
pop = json.load(open(R + '/audit/out/31/population.json'))
cfgs = pop['configs']
print(json.dumps(pop['counts'])[:1500])
print(json.dumps(cfgs[0]['assets']['objects'][0])[:1500])
kinds = {}
for c in cfgs:
    a = c['assets']
    objs = a['objects']
    ks = sorted({o['kind'] for o in objs})
    nmod = sum(len(o['indices']) for o in objs)
    arms = [ (x['robot_name'], x['role']) for x in c['robot']['arms']]
    print(f"{c['task_name']:36s} {c['variant']:10s} arms={arms} fixt={c['scene']['fixtures']} stab={c['scene']['enable_stabilization']} inst={a['layout0_seed0_instance_counts']} kinds={ks} nmodels={nmod} distinct={a['distinct_model_count_all_layouts']} clutter={a['distinct_clutter_model_count']} vis={len(a['visual_asset_paths'])}")
