"""G5 prep: CPU-only load check of InternW0-Δ (no GPU, no forward pass).

Builds the model exactly as the adapter's model.py does (config/eval_model.yaml with model.py's overrides) on device
"cpu", loads robodojo.pt through the vendored runtime's own load_wam_checkpoint, and prints its compatibility report
(compatible, missing, shape-skipped and unexpected tensors per module), the parameter count and dtype. It checks that the
downloaded weights, Wan2.2 files and RynnBrain directory fit the code before any GPU time is spent.
usage (policy env): python -I cpu_load_check.py <XPolicyLab dir> <robodojo.pt> <Wan2.2 dir> <RynnBrain dir>
"""
import logging
import os
import sys
import time

xpl, ckpt, wan, rynn = sys.argv[1:5]
pol = os.path.join(xpl, "policy", "InternW0_delta")
sys.path[:0] = [os.path.join(pol, "wam_runtime", "src")]
os.environ["DIFFSYNTH_SKIP_DOWNLOAD"] = "true"
os.environ["DIFFSYNTH_MODEL_BASE_PATH"] = os.path.dirname(os.path.dirname(os.path.abspath(wan)))
logging.basicConfig(level=logging.INFO)

import torch  # noqa: E402
from hydra.utils import instantiate  # noqa: E402
from omegaconf import OmegaConf  # noqa: E402

from wam.runtime.checkpoint import load_wam_checkpoint  # noqa: E402

t0 = time.time()
saved = OmegaConf.load(os.path.join(pol, "config", "eval_model.yaml"))
cfg = OmegaConf.create(OmegaConf.to_container(saved.model, resolve=True))
cfg.model_id = wan
cfg.tokenizer_model_id = wan
cfg.redirect_common_files = False
cfg.skip_dit_load_from_pretrain = True
cfg.action_dit_pretrained_path = None
cfg.action_dit_use_interpolated_init = False
cfg.understanding.vlm_model_path = rynn
cfg.load_text_encoder = True
cfg.action_dit_config.physical_time_rope_enabled = False
cfg.action_dit_config.dimension_valid_mask = None
cfg.action_dit_config.valid_input_scale = 1.0
cfg.proprio_encoding = OmegaConf.create({"dimension_valid_mask": None, "valid_input_scale": 1.0})
model = instantiate(cfg, model_dtype=torch.bfloat16, device="cpu")
t1 = time.time()
payload = load_wam_checkpoint(model, ckpt)
t2 = time.time()
n = sum(p.numel() for p in model.parameters())
print(f"CPU_LOAD_CHECK params={n / 1e9:.3f}e9 dtypes={sorted({str(p.dtype) for p in model.parameters()})} "
      f"ckpt_step={payload.get('step')} ckpt_keys={sorted(k for k in payload if isinstance(payload[k], dict))} "
      f"build_s={t1 - t0:.0f} load_s={t2 - t1:.0f} torch={torch.__version__}", flush=True)
