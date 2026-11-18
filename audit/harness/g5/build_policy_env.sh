#!/bin/bash
# G5 prep: InternW0-Δ policy-server environment, as XPolicyLab/policy/InternW0_delta/install.sh builds it, under scratch.
# usage (on a Della node with internet, e.g. della-vis1): G5=<scratch>/work/g5 bash build_policy_env.sh
# Deviations from install.sh (declared in the run manifests):
# - the env is a prefix under $G5/conda (not a named env); conda packages come from conda-forge only (install.sh's
#   `conda create python=3.11` uses the default channel);
# - pip and conda caches live under $G5 (home is 50 GiB);
# - eval-only branch of install.sh (no full upstream checkout): `pip install -e wam_runtime`, then `pip install -e XPolicyLab`.
module load anaconda3/2026.7   # before set -u: the module init reads an unset PS1
set -euo pipefail
G5=${G5:?set G5 to the g5 scratch folder}
ENV=$G5/conda/internw0
XPL=$G5/XPolicyLab
export CONDA_PKGS_DIRS=$G5/conda_pkgs PIP_CACHE_DIR=$G5/pip_cache
[[ -x $ENV/bin/python ]] || conda create -y -p $ENV -c conda-forge --override-channels python=3.11
conda install -y -p $ENV -c conda-forge --override-channels gcc_linux-64 gxx_linux-64
PY=$ENV/bin/python
$PY -m pip install --upgrade pip
$PY -m pip install torch==2.10.0 torchvision==0.25.0 --index-url https://download.pytorch.org/whl/cu128
$PY -m pip install "modelscope==1.34.0" "ftfy==6.3.1" "flash-linear-attention==0.5.1"
# causal-conv1d is sdist-only; its setup.py picks a prebuilt wheel for the torch it is built against, else compiles with
# nvcc. install.sh installs it with pip's default build isolation, which built it against another torch: the import then
# fails (undefined symbol c10::cuda::CUDAErrorLogCapture), and transformers' Qwen3.5 code (RynnBrain) imports it whenever
# CUDA is available, so the policy server would crash. Deviation: install it without build isolation, against torch 2.10
# (the prebuilt wheel then chosen holds sm_75 to sm_120 kernels).
CUDA_HOME=/usr/local/cuda-12.8 PATH=/usr/local/cuda-12.8/bin:$PATH \
  $PY -m pip install --no-build-isolation --no-deps "causal-conv1d==1.6.2.post1"
$PY -m pip install -e $XPL/policy/InternW0_delta/wam_runtime
$PY -m pip install -e $XPL
# As install.sh runs them, the last step lets XPolicyLab's dependencies (opencv-python-headless 5) raise numpy to 2.x
# over wam_runtime's numpy==1.26.4 pin; kept as the official script leaves it (declared).
$PY -m pip freeze > $G5/logs/policy_env_freeze.txt
(cd /tmp && $PY -I -c "import torch, transformers, numpy, causal_conv1d_cuda; print('torch', torch.__version__, 'cuda', torch.version.cuda, 'transformers', transformers.__version__, 'numpy', numpy.__version__)")
echo BUILD_OK
