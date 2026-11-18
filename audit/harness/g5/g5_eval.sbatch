#!/bin/bash
# G5 prep on Della's RTX PRO 6000 node: policy server and Isaac Sim 6.2 share one GPU (96 GB).
# usage (login node): sbatch --export=ALL,G5=<scratch>/work/g5,TAG=<tag>,EVAL_NUM=3,EVALUATOR=v1,SMOKE=1 \
#   --output=<scratch>/work/g5/logs/%x-%j.out <scratch>/work/g5/robodojo-physics/audit/harness/g5/g5_eval.sbatch
# Account <account> comes from SBATCH_ACCOUNT (~/.bashrc). Keep jobs at or under one hour; a run longer than that continues
# in a new job with the same TAG (main.py resumes from its manifest; the episode in progress at the limit is rerun).
#SBATCH --job-name=g5prep
#SBATCH --partition=rtx6000
#SBATCH --gres=gpu:rtx_pro_6000:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=96G
#SBATCH --time=01:00:00
export SIM_TIMEOUT_S=${SIM_TIMEOUT_S:-2700}
export GPU=0 ACTIVE_GPU=0
bash ${G5:?}/robodojo-physics/audit/harness/g5/run_g5.sh
