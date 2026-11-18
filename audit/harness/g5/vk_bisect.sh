#!/bin/bash
# G5 prep diagnosis (della-vis2): which part of run_g5.sh's host environment breaks the NVIDIA Vulkan ICD in the 6.2
# container? Runs vk_probe.py (vkCreateInstance only, no GPU work) under each variant.
# usage: G5=<scratch>/work/g5 bash vk_bisect.sh
G5=${G5:?}; D=${ISAAC_ROOT:-/scratch/<user>/isaac}; P=$(dirname "$(readlink -f "$0")")/vk_probe.py
probe() {  # $1 label, rest: host-side env assignments
  local label=$1; shift
  ( cd $G5/tmp && env "$@" APPTAINERENV_CUDA_VISIBLE_DEVICES=0 DISPLAY= apptainer exec --nv --home $G5/home_62 -B $D \
      $G5/isaac-sim_6.2.0.1.sif /isaac-sim/kit/python/bin/python3 $P 2>&1 | grep VK_PROBE | sed "s/^/$label: /" | cut -c1-70 )
}
probe plain X=1
probe tmpdir TMPDIR=$G5/tmp
probe hf HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 HF_HOME=$G5/hf_home
probe triton TRITON_CACHE_DIR=$G5/cache/triton TORCHINDUCTOR_CACHE_DIR=$G5/cache/inductor
probe conda_path PATH=/usr/licensed/anaconda3/2026.7/bin:$PATH
( module load anaconda3/2026.7 2>/dev/null; probe module_anaconda3 X=1 )
probe egl_only __EGL_VENDOR_LIBRARY_DIRS=/usr/licensed/anaconda3/2026.7.1/share/glvnd/egl_vendor.d
( module load anaconda3/2026.7 2>/dev/null; unset __EGL_VENDOR_LIBRARY_DIRS; probe module_minus_egl X=1 )
