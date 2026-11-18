#!/bin/bash
# G5 prep: InternW0-Δ weights under scratch. usage (on a Della node with internet): G5=<scratch>/work/g5 bash fetch_policy.sh ckpt|assets|verify
# ckpt:   robodojo.pt from RoboDojo's own Hugging Face dataset (ckpt/RoboDojo/InternW0_delta, pinned HF_REV); same SHA-256 as the
#         policy's stated source (InternRobotics/InternW0-Delta-RoboDojo) and its download_checkpoint.sh.
# assets: Wan2.2 VAE, UMT5 encoder and tokenizer, and RynnBrain1.1-2B from Hugging Face at pinned revisions (the adapter's
#         download_assets.sh takes them from ModelScope; see below).
# verify: SHA-256 of the four weight files against the adapter's config/artifacts.lock.json and of the eleven other files
#         against ModelScope's own listings.
set -euo pipefail
G5=${G5:?set G5 to the g5 scratch folder}
REV=${HF_REV:-1127bda94f172d757386945af3eb3a6754d1a4e6}
XPL=$G5/XPolicyLab
case "${1:?ckpt|assets|verify}" in
  ckpt)
    mkdir -p $G5/dl/ckpt
    curl --fail --location --continue-at - --output $G5/dl/ckpt/robodojo.pt \
      "https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/resolve/$REV/ckpt/RoboDojo/InternW0_delta/RoboDojo-sim-arx_x5-joint-0/checkpoints/robodojo.pt"
    ;;
  assets)
    # The adapter's download_assets.sh fetches these from ModelScope. From Della that ran at 0.5 to 7 MB/s, and it asks
    # for RynnBrain under its Hugging Face ID `Alibaba-DAMO-Academy/RynnBrain1.1-2B`, which ModelScope does not know
    # ("record not found"; ModelScope hosts it as `DAMO_Academy/RynnBrain1.1-2B`, #147). The same files are taken from the
    # same organisations' Hugging Face repositories at pinned revisions; `verify` checks them against the adapter's
    # artifacts.lock.json and against ModelScope's own SHA-256 listings.
    WAN_REV=921dbaf3f1674a56f47e83fb80a34bac8a8f203e; RB_REV=36069aa841fc58f500d307316cbfdf379777ce8a
    get() { mkdir -p "$(dirname "$3")"; curl --fail --location --continue-at - --silent --show-error --output "$3" \
              "https://huggingface.co/$1/resolve/$2/$4"; }
    W=$G5/dl/assets/Wan-AI/Wan2.2-TI2V-5B
    for f in Wan2.2_VAE.pth models_t5_umt5-xxl-enc-bf16.pth google/umt5-xxl/special_tokens_map.json \
             google/umt5-xxl/spiece.model google/umt5-xxl/tokenizer.json google/umt5-xxl/tokenizer_config.json; do
      get Wan-AI/Wan2.2-TI2V-5B $WAN_REV $W/$f $f
    done
    B=$G5/dl/assets/Alibaba-DAMO-Academy/RynnBrain1.1-2B
    for f in README.md chat_template.jinja config.json generation_config.json model.safetensors processor_config.json \
             tokenizer.json tokenizer_config.json; do
      get Alibaba-DAMO-Academy/RynnBrain1.1-2B $RB_REV $B/$f $f
    done
    ;;
  verify)
    python3 - "$G5" "$XPL/policy/InternW0_delta/config/artifacts.lock.json" <<'PY'
import hashlib, json, os, sys
g5, lock = sys.argv[1], json.load(open(sys.argv[2]))
def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 24), b""):
            h.update(b)
    return h.hexdigest()
rows = [("checkpoint", f"{g5}/dl/ckpt/robodojo.pt", lock["checkpoint"]["sha256"])]
for name, meta in lock["wan_base"]["files"].items():
    rows.append((name, f"{g5}/dl/assets/Wan-AI/Wan2.2-TI2V-5B/{name}", meta["sha256"]))
rows.append(("rynnbrain", f"{g5}/dl/assets/Alibaba-DAMO-Academy/RynnBrain1.1-2B/{lock['vlm']['required_file']}", lock["vlm"]["sha256"]))
# Wan tokenizer files: SHA-256 as ModelScope lists them (api/v1/models/Wan-AI/Wan2.2-TI2V-5B/repo/files, November 13, 2026)
for name, want in (("special_tokens_map.json", "7b8a9f5040adb67b5805abdfd42c1f8d0f3d0e711f10726580eb3789cd0ad61d"),
                   ("spiece.model", "e3909a67b780650b35cf529ac782ad2b6b26e6d1f849d3fbb6a872905f452458"),
                   ("tokenizer.json", "6e197b4d3dbd71da14b4eb255f4fa91c9c1f2068b20a2de2472967ca3d22602b"),
                   ("tokenizer_config.json", "ed9a3a8b0faa71a70a32847e0435fe036e6e112d4df4edb7bb48a921e344dc05")):
    rows.append((f"umt5 {name}", f"{g5}/dl/assets/Wan-AI/Wan2.2-TI2V-5B/google/umt5-xxl/{name}", want))
# RynnBrain's other files: SHA-256 as ModelScope lists them (api/v1/models/DAMO_Academy/RynnBrain1.1-2B/repo/files, November 13, 2026)
for name, want in (("README.md", "d19e40a4f84dbe1daeb907dab2eb8cb168c954e310ac5639393df265778d4822"),
                   ("chat_template.jinja", "273d8e0e683b885071fb17e08d71e5f2a5ddfb5309756181681de4f5a1822d80"),
                   ("config.json", "3db6717dea7333fd0a4f543e378c5e68e761c8f2907384c537bd8bbee4d18b81"),
                   ("generation_config.json", "943f9e0a394325880a8a7e38c8d2c5bc40e2bb44a9bdc689f50fc26a2212edcc"),
                   ("processor_config.json", "14932921ca485d458a04dafd8069fbb0a4505622a48208d19ed247115801385b"),
                   ("tokenizer.json", "87a7830d63fcf43bf241c3c5242e96e62dd3fdc29224ca26fed8ea333db72de4"),
                   ("tokenizer_config.json", "e98f1901ac6f0adff67b1d540bfa0c36ac1a0cf59eb72ed78146ef89aafa1182")):
    rows.append((f"rynnbrain {name}", f"{g5}/dl/assets/Alibaba-DAMO-Academy/RynnBrain1.1-2B/{name}", want))
ok = True
for name, path, want in rows:
    got = sha(path) if os.path.exists(path) else "missing"
    ok &= got == want
    print(f"{'OK ' if got == want else 'BAD'} {name} {got} {path}")
print("VERIFY_OK" if ok else "VERIFY_FAILED")
PY
    ;;
esac
