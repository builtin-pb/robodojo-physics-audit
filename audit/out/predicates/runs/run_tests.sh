#!/bin/bash
# CPU-only unit tests of RoboDojo's scoring predicates in the Isaac Sim 5.1 container (no --nv, CUDA_VISIBLE_DEVICES empty).
D=/scratch/<user>/isaac; W=$D/work/predicates; H=$1; R=$W/repo_$H; O=$W/tests_$H; mkdir -p $O
run() {  # $1 tag, $2 extra env
  date -u +%FT%TZ > $O/$1.start
  ( cd $R && apptainer exec --home $D/home -B $D $D/isaac-sim_5.1.0.sif env CUDA_VISIBLE_DEVICES= PYTHONPATH=$D/pylib:$R $2 \
      PREDICATES_ACCEPTANCE_OUT=$O/acceptance_$1.json /isaac-sim/python.sh -m unittest audit.tools.test_predicates -v ) > $O/log_$1.txt 2>&1
  echo exit=$? >> $O/log_$1.txt; date -u +%FT%TZ > $O/$1.end
}
run fork ""
run upstream "PREDICATES_UPSTREAM_DIR=$W/upstream_f57e163"
apptainer exec --home $D/home -B $D $D/isaac-sim_5.1.0.sif env PYTHONPATH=$D/pylib /isaac-sim/python.sh -c "import numpy, shapely, torch, transforms3d, scipy, sys; print('python', sys.version.split()[0], 'numpy', numpy.__version__, 'shapely', shapely.__version__, 'torch', torch.__version__, 'transforms3d', transforms3d.__version__, 'scipy', scipy.__version__)" > $O/versions.txt 2>&1
