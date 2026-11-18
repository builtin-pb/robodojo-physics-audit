"""Offline CPU unit tests for `run_batch.py --slurm` (stdlib only): `python3 -I audit/harness/test_run_batch_slurm.py`."""
from __future__ import annotations

import contextlib
import gzip
import io
import json
import os
import stat
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "../.."))
sys.path.insert(0, HERE)
import run_batch as rb  # noqa: E402
import speed_parity as sp  # noqa: E402


def _write_exec(path: str, body: str) -> None:
    with open(path, "w") as f:
        f.write(body)
    os.chmod(path, os.stat(path).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


class RunBatchSlurmTests(unittest.TestCase):
    def test_refusal_outside_allocation_and_on_login_hosts(self):
        good_env = {"SLURM_JOB_ID": "17500001", "CUDA_VISIBLE_DEVICES": "0"}
        for bad_host in (
            "della",
            "della7",
            "della8.princeton.edu",
            "della9",
            "della9.princeton.edu",
            "della9-h18u29",
            "della-milan",
            "della-milan.princeton.edu",
            "della-milan-i12u10",
            "della-gpu",
            "della-gpu.princeton.edu",
            "mydella",
            "login01",
        ):
            self.assertTrue(rb.is_login_host(bad_host), bad_host)
            with self.assertRaisesRegex(ValueError, "refuses login host"):
                rb.validate_slurm_env(host=bad_host, env=good_env)
            with self.assertRaises(SystemExit) as cm:
                rb.main(
                    ["--slurm", "--allow_any_host", "--name", "x", "--layouts", "0", "--dry_run"],
                    env=good_env,
                    hostname=bad_host,
                )
            self.assertIn("refuses login host", str(cm.exception))

        for ok_host in ("della-l02g1", "della-l09g7.princeton.edu", "della-i14g12", "della-h23g1"):
            self.assertFalse(rb.is_login_host(ok_host), ok_host)
            node, jid, gpus = rb.validate_slurm_env(host=ok_host, env=good_env)
            self.assertEqual((node, jid, gpus), (ok_host.split(".")[0], "17500001", [0]))

        for no_alloc_env in ({}, {"CUDA_VISIBLE_DEVICES": "0"}, {"SLURM_JOB_ID": "   ", "CUDA_VISIBLE_DEVICES": "0"}):
            with self.assertRaisesRegex(ValueError, "SLURM_JOB_ID"):
                rb.validate_slurm_env(host="della-l02g1", env=no_alloc_env)
            with self.assertRaises(SystemExit) as cm:
                rb.main(["--slurm", "--name", "x", "--layouts", "0", "--dry_run"], env=no_alloc_env, hostname="della-l02g1")
            self.assertIn("SLURM_JOB_ID", str(cm.exception))

        with self.assertRaises(SystemExit) as cm:
            rb.main(
                ["--slurm", "--nodes", "della-vis2:1:2", "--name", "x", "--layouts", "0", "--dry_run"],
                env=good_env,
                hostname="della-l02g1",
            )
        self.assertIn("omit --nodes", str(cm.exception))

        with self.assertRaises(SystemExit) as cm:
            rb.main(
                ["--nodes", "della-vis2:1:2", "--procs_per_gpu", "2", "--name", "x", "--layouts", "0", "--dry_run"],
                hostname="della-vis2",
            )
        self.assertIn("--procs_per_gpu is for --slurm", str(cm.exception))

    def test_gpu_index_mapping(self):
        # Device-remapped job (e.g. nvidia_spank): CUDA_VISIBLE_DEVICES=0 wins over physical SLURM_JOB_GPUS=2.
        self.assertEqual(rb.parse_slurm_gpus({"CUDA_VISIBLE_DEVICES": "0", "SLURM_JOB_GPUS": "2"}), [0])
        # ConstrainDevices=no without device remapping: CUDA_VISIBLE_DEVICES carries the job's node GPU ids.
        self.assertEqual(rb.parse_slurm_gpus({"CUDA_VISIBLE_DEVICES": "2,3", "SLURM_JOB_GPUS": "2,3"}), [2, 3])
        # Step override when CUDA_VISIBLE_DEVICES is absent.
        self.assertEqual(rb.parse_slurm_gpus({"SLURM_STEP_GPUS": "1", "SLURM_JOB_GPUS": "0,1"}), [1])
        # Job fallback with range syntax.
        self.assertEqual(rb.parse_slurm_gpus({"SLURM_JOB_GPUS": "0-3"}), [0, 1, 2, 3])
        self.assertEqual(rb.parse_slurm_gpus({"SLURM_JOB_GPUS": "0,2-3"}), [0, 2, 3])
        # UUID / MIG tokens.
        self.assertEqual(
            rb.parse_slurm_gpus({"CUDA_VISIBLE_DEVICES": "GPU-1111-aaaa,MIG-ef01-2222"}),
            ["GPU-1111-aaaa", "MIG-ef01-2222"],
        )
        # Invalid or empty specifications must fail loudly rather than touch unallocated GPUs.
        for bad in (
            {},
            {"CUDA_VISIBLE_DEVICES": "", "SLURM_JOB_GPUS": "0"},
            {"CUDA_VISIBLE_DEVICES": "NoDevFiles", "SLURM_JOB_GPUS": "0"},
            {"SLURM_JOB_GPUS": "3-1"},
            {"CUDA_VISIBLE_DEVICES": "1,1"},
            {"CUDA_VISIBLE_DEVICES": "gpu0"},
        ):
            with self.assertRaises(ValueError, msg=repr(bad)):
                rb.parse_slurm_gpus(bad)

    def test_slot_construction_and_dry_run_slurm(self):
        with tempfile.TemporaryDirectory() as td:
            bin_dir = os.path.join(td, "bin")
            os.makedirs(bin_dir)
            _write_exec(
                os.path.join(bin_dir, "nvidia-smi"),
                "#!/bin/bash\necho 'NVIDIA A100-SXM4-80GB'\n",
            )
            env = {
                "SLURM_JOB_ID": "17500123",
                "CUDA_VISIBLE_DEVICES": "2,3",
                "SLURM_JOB_GPUS": "2,3",
                "PATH": f"{bin_dir}:{os.environ.get('PATH', '')}",
            }
            buf = io.StringIO()
            old_path = os.environ.get("PATH")
            os.environ["PATH"] = env["PATH"]
            try:
                with contextlib.redirect_stdout(buf):
                    rc = rb.main(
                        [
                            "--slurm",
                            "--procs_per_gpu",
                            "3",
                            "--root",
                            "/scr/u",
                            "--repo",
                            "/scr/u/robodojo-physics",
                            "--sif",
                            "/scr/u/isaac-sim_5.1.0.sif",
                            "--name",
                            "slm_t",
                            "--seed",
                            "0",
                            "--layouts",
                            "0-5",
                            "--dry_run",
                            "--",
                            "--task",
                            "put_bottles_into_dustbin",
                            "--policy",
                            "slide",
                        ],
                        env=env,
                        hostname="della-l02g1.princeton.edu",
                    )
            finally:
                if old_path is None:
                    os.environ.pop("PATH", None)
                else:
                    os.environ["PATH"] = old_path
            self.assertEqual(rc, 0)
            lines = buf.getvalue().strip().splitlines()
            self.assertEqual(len(lines), 1 + 6)
            plan = json.loads(lines[0][len("BATCH plan ") :])
            self.assertEqual(
                plan["nodes"],
                [{"host": "della-l02g1", "gpu": 2, "procs": 3}, {"host": "della-l02g1", "gpu": 3, "procs": 3}],
            )
            self.assertEqual(plan["slurm"]["job_id"], "17500123")
            self.assertEqual(plan["slurm"]["node"], "della-l02g1")
            self.assertEqual(plan["slurm"]["gpu_type"], "NVIDIA A100-SXM4-80GB")
            self.assertEqual(plan["slurm"]["gpus"], [2, 3])
            cmd_lines = lines[1:]
            for idx, cmd in enumerate(cmd_lines):
                expected_gpu = 2 if idx < 3 else 3
                self.assertIn(f"slm_t_q0{idx}_l02g1", cmd)
                self.assertIn(f"HARNESS_GPU={expected_gpu}", cmd)
                self.assertIn("SLURM_JOB_ID=17500123", cmd)
                self.assertIn("HARNESS_NODE=della-l02g1", cmd)
                self.assertIn("HARNESS_GPU_TYPE='NVIDIA A100-SXM4-80GB'", cmd)
                self.assertIn("HARNESS_SIF=/scr/u/isaac-sim_5.1.0.sif", cmd)
                self.assertIn("--claim_dir /scr/u/harness_runs/slm_t_batch/claims", cmd)
                self.assertNotIn("ssh ", cmd)

    def test_summarize_gpu_hours_and_fields(self):
        slurm = {
            "job_id": "17500555",
            "node": "della-l09g7",
            "gpus": [0, 1],
            "gpu_type": "NVIDIA A100-SXM4-80GB",
            "gpu_names": {"0": "NVIDIA A100-SXM4-80GB", "1": "NVIDIA A100-SXM4-80GB"},
            "procs_per_gpu": 2,
            "cuda_visible_devices": "0,1",
            "slurm_job_gpus": "2,3",
            "slurm_step_gpus": None,
        }
        procs = [
            {
                "tag": "p0",
                "host": "della-l09g7",
                "node": "della-l09g7",
                "gpu": 0,
                "gpu_type": "NVIDIA A100-SXM4-80GB",
                "slurm_job_id": "17500555",
                "t_start": 100.0,
                "t_end": 1900.0,
                "rc": 0,
                "result": {"runs": [{"seed": 0, "layout": 0, "episode_score_pct": 100.0, "success": True, "take_action_cnt": 10, "wall_seconds": 30.0}]},
            }
        ]
        # 1800 s (0.5 h) wall time x 2 allocated GPUs = 1.0 GPU-hour.
        s = rb.summarize(procs, [(0, 0)], 100.0, 1900.0, slurm=slurm)
        self.assertEqual(s["slurm_job_id"], "17500555")
        self.assertEqual(s["node"], "della-l09g7")
        self.assertEqual(s["gpu_type"], "NVIDIA A100-SXM4-80GB")
        self.assertEqual(s["gpus"], [0, 1])
        self.assertEqual(s["gpu_hours"], 1.0)
        self.assertEqual(s["slurm"]["gpu_hours"], 1.0)
        self.assertEqual(s["per_host"]["della-l09g7"]["gpu_hours"], 1.0)
        self.assertEqual(s["per_host"]["della-l09g7"]["gpus"], 2)

    def test_end_to_end_slurm_execution_and_recorded_fields(self):
        with tempfile.TemporaryDirectory() as td:
            root = os.path.join(td, "root")
            fake_repo = os.path.join(td, "repo")
            bin_dir = os.path.join(td, "bin")
            os.makedirs(os.path.join(fake_repo, "audit/harness"))
            os.makedirs(bin_dir)
            ssh_marker = os.path.join(td, "ssh_called")

            # Fail loudly if `ssh` is ever called in --slurm mode.
            _write_exec(
                os.path.join(bin_dir, "ssh"),
                f"#!/bin/bash\necho called > {ssh_marker}\nexit 99\n",
            )
            _write_exec(
                os.path.join(bin_dir, "nvidia-smi"),
                "#!/bin/bash\n"
                "if [[ \"$*\" == *memory.free* ]]; then echo 81182; else echo 'NVIDIA A100-SXM4-80GB'; fi\n",
            )
            _write_exec(
                os.path.join(fake_repo, "audit/harness/run_harness.sh"),
                "#!/bin/bash\n"
                "set -euo pipefail\n"
                "TAG=$1; shift\n"
                "python3 -I - \"$TAG\" \"$@\" <<'PY'\n"
                "import json, os, sys\n"
                "tag = sys.argv[1]\n"
                "a = sys.argv[2:]\n"
                "lays = [int(x) for x in a[a.index('--layouts') + 1].split(',')]\n"
                "seed = int(a[a.index('--seed') + 1])\n"
                "claim_dir = a[a.index('--claim_dir') + 1]\n"
                "os.makedirs(claim_dir, exist_ok=True)\n"
                "runs = []\n"
                "for lay in lays:\n"
                "    cpath = os.path.join(claim_dir, f's{seed}_l{lay}_r0.claim')\n"
                "    try:\n"
                "        fd = os.open(cpath, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)\n"
                "        os.close(fd)\n"
                "    except FileExistsError:\n"
                "        continue\n"
                "    runs.append({'seed': seed, 'layout': lay, 'repeat': 0, 'policy': 'slide',\n"
                "                 'episode_score_pct': 100.0, 'success': True, 'take_action_cnt': 0,\n"
                "                 'wall_seconds': 1.2, 'final_state': {'sha256': f'sha_{lay:02d}'},\n"
                "                 'layout_files': [f'Eval_Layout/x/0/t_{lay}.json']})\n"
                "out_dir = os.path.join(os.environ['ISAAC_ROOT'], 'harness_runs', tag)\n"
                "os.makedirs(out_dir, exist_ok=True)\n"
                "res = {'versions': {'repo_head': 'abc', 'container': 'isaac-sim_5.1.0.sif'},\n"
                "       'deviations': 'd', 'speed': {'fast_ik': True}, 'runs': runs}\n"
                "with open(os.path.join(out_dir, 'result.json'), 'w') as f:\n"
                "    json.dump(res, f, indent=1)\n"
                "PY\n",
            )

            env = {
                "SLURM_JOB_ID": "17500999",
                "CUDA_VISIBLE_DEVICES": "0,1",
                "SLURM_JOB_GPUS": "2,3",
                "PATH": f"{bin_dir}:{os.environ.get('PATH', '')}",
            }
            old_path = os.environ.get("PATH")
            os.environ["PATH"] = env["PATH"]
            buf = io.StringIO()
            try:
                with contextlib.redirect_stdout(buf):
                    rc = rb.main(
                        [
                            "--slurm",
                            "--procs_per_gpu",
                            "2",
                            "--stagger_s",
                            "0",
                            "--root",
                            root,
                            "--repo",
                            fake_repo,
                            "--name",
                            "slm_e2e",
                            "--seed",
                            "0",
                            "--layouts",
                            "0-5",
                            "--state_digest",
                            "--",
                            "--task",
                            "put_bottles_into_dustbin",
                            "--policy",
                            "slide",
                        ],
                        env=env,
                        hostname="della-l09g7.princeton.edu",
                    )
            finally:
                if old_path is None:
                    os.environ.pop("PATH", None)
                else:
                    os.environ["PATH"] = old_path

            self.assertEqual(rc, 0, buf.getvalue())
            self.assertFalse(os.path.exists(ssh_marker), "ssh must never be invoked in --slurm mode")

            summ_path = os.path.join(root, "harness_runs", "slm_e2e_batch", "batch_summary.json")
            gz_path = summ_path + ".gz"
            with open(summ_path) as f:
                summ = json.load(f)
            with gzip.open(gz_path, "rt") as f:
                summ_gz = json.load(f)
            self.assertEqual(summ, summ_gz)
            self.assertEqual(summ["requested"], 6)
            self.assertEqual(summ["episodes"], 6)
            self.assertEqual(summ["missing"], [])
            self.assertEqual(summ["duplicates"], [])
            self.assertEqual(summ["slurm_job_id"], "17500999")
            self.assertEqual(summ["node"], "della-l09g7")
            self.assertEqual(summ["gpu_type"], "NVIDIA A100-SXM4-80GB")
            self.assertEqual(summ["gpus"], [0, 1])
            expected_gh = round(max(0.0, summ["t_end"] - summ["t_start"]) * 2 / 3600.0, 4)
            self.assertEqual(summ["gpu_hours"], expected_gh)
            self.assertEqual(summ["slurm"]["job_id"], "17500999")
            self.assertEqual(summ["slurm"]["slurm_job_gpus"], "2,3")
            self.assertEqual(len(summ["procs"]), 4)

            # Every per-process manifest (`result.json`) records node, gpu_type and slurm_job_id in `versions`,
            # and remains top-level compatible with `speed_parity.py` against an archived A40 manifest.
            a40_ref = os.path.join(td, "a40_ref.json")
            with open(a40_ref, "w") as f:
                json.dump(
                    {
                        "versions": {"repo_head": "abc", "container": "isaac-sim_5.1.0.sif"},
                        "deviations": "d",
                        "speed": {"fast_ik": False},
                        "runs": [
                            {
                                "seed": 0,
                                "layout": lay,
                                "repeat": 0,
                                "policy": "slide",
                                "episode_score_pct": 100.0,
                                "success": True,
                                "take_action_cnt": 0,
                                "wall_seconds": 2.5,
                                "final_state": {"sha256": f"sha_{lay:02d}"},
                                "layout_files": [f"Eval_Layout/x/0/t_{lay}.json"],
                            }
                            for lay in range(6)
                        ],
                    },
                    f,
                )
            cand_paths = []
            for p in summ["procs"]:
                self.assertEqual(p["node"], "della-l09g7")
                self.assertEqual(p["gpu_type"], "NVIDIA A100-SXM4-80GB")
                self.assertEqual(p["slurm_job_id"], "17500999")
                rpath = os.path.join(root, "harness_runs", p["tag"], "result.json")
                cand_paths.append(rpath)
                with open(rpath) as f:
                    manifest = json.load(f)
                self.assertEqual(manifest["versions"]["node"], "della-l09g7")
                self.assertEqual(manifest["versions"]["gpu_type"], "NVIDIA A100-SXM4-80GB")
                self.assertEqual(manifest["versions"]["slurm_job_id"], "17500999")

            parity = sp.compare([a40_ref], cand_paths)
            self.assertEqual(parity["all_runs_equal"], True, sp.summary_lines(parity))
            self.assertTrue(all(v["equal"] for v in parity["top_level"].values()), parity["top_level"])

    def test_job_tmp_and_sbatch_template_contracts(self):
        sbatch_path = os.path.join(HERE, "run_batch.sbatch")
        with open(sbatch_path) as f:
            sbatch_src = f.read()
        for line in sbatch_src.splitlines():
            if line.startswith("#SBATCH"):
                self.assertNotIn("--account=", line, "run_batch.sbatch must not bake in a default #SBATCH --account")
                self.assertNotIn("--partition=", line, "run_batch.sbatch must omit #SBATCH --partition")
        self.assertIn("#SBATCH --constraint=gpu80|sxm", sbatch_src)
        self.assertIn("APPTAINERENV_TMPDIR", sbatch_src)
        self.assertIn("ulimit -c 0", sbatch_src)
        self.assertIn("module load proxy/default", sbatch_src)

        with tempfile.TemporaryDirectory() as td:
            bin_dir = os.path.join(td, "bin")
            os.makedirs(bin_dir)
            _write_exec(os.path.join(bin_dir, "nvidia-smi"), "#!/bin/bash\necho 'NVIDIA A100-SXM4-80GB'\n")
            job_tmp = "/scratch/<user>/tmp/job/17500777"
            env = {
                "SLURM_JOB_ID": "17500777",
                "CUDA_VISIBLE_DEVICES": "0",
                "HARNESS_JOB_TMP": job_tmp,
                "PATH": f"{bin_dir}:{os.environ.get('PATH', '')}",
            }
            old_path = os.environ.get("PATH")
            os.environ["PATH"] = env["PATH"]
            buf = io.StringIO()
            try:
                with contextlib.redirect_stdout(buf):
                    rc = rb.main(
                        [
                            "--slurm",
                            "--procs_per_gpu",
                            "2",
                            "--root",
                            "/scratch/<user>/project/robodojo",
                            "--repo",
                            "/scratch/<user>/project/robodojo/wt-slurm",
                            "--name",
                            "slm_jtmp",
                            "--seed",
                            "2",
                            "--layouts",
                            "0,1",
                            "--dry_run",
                            "--",
                            "--task",
                            "swap_T",
                            "--policy",
                            "slide",
                        ],
                        env=env,
                        hostname="della-l02g4.princeton.edu",
                    )
            finally:
                if old_path is None:
                    os.environ.pop("PATH", None)
                else:
                    os.environ["PATH"] = old_path
            self.assertEqual(rc, 0)
            lines = buf.getvalue().strip().splitlines()
            plan = json.loads(lines[0][len("BATCH plan ") :])
            self.assertEqual(plan["slurm"]["job_tmp"], job_tmp)
            for idx, cmd in enumerate(lines[1:]):
                slot_tag = f"slm_jtmp_q0{idx}_l02g4"
                self.assertIn(f"TMPDIR={job_tmp}/slots/{slot_tag}/work", cmd)
                self.assertIn(f"HARNESS_JOB_TMP={job_tmp}", cmd)
                self.assertIn(f"APPTAINERENV_TMPDIR={job_tmp}/slots/{slot_tag}/tmp", cmd)

    def test_unchanged_vis_path_dry_run_matches_main(self):
        vis_cases = [
            [
                "--root",
                "/scr/w",
                "--repo",
                "/scr/w/robodojo-physics",
                "--name",
                "spd_b20",
                "--nodes",
                "della-vis1:0:2,della-vis2:1:4",
                "--seed",
                "0",
                "--layouts",
                "0-19",
                "--state_digest",
                "--dry_run",
                "--",
                "--task",
                "put_bottles_into_dustbin",
                "--policy",
                "script_bottles",
                "--evaluator",
                "v2",
            ],
            [
                "--root",
                "/scr/w",
                "--repo",
                "/scr/w/robodojo-physics",
                "--name",
                "spd_chunk",
                "--nodes",
                "della-vis2:1:2",
                "--chunk",
                "3",
                "--max_wall_s",
                "600",
                "--kit",
                "/scr/w/custom.kit",
                "--no_speed",
                "--seed",
                "2",
                "--layouts",
                "0-4,7",
                "--dry_run",
                "--",
                "--task",
                "cover_blocks",
                "--policy",
                "slide",
            ],
        ]
        ref_rev = "origin/main"
        git_show = subprocess.run(
            ["git", "-C", REPO, "show", f"{ref_rev}:audit/harness/run_batch.py"],
            capture_output=True,
            text=True,
        )
        if git_show.returncode != 0:
            git_show = subprocess.run(
                ["git", "-C", REPO, "show", "main:audit/harness/run_batch.py"],
                capture_output=True,
                text=True,
            )
        self.assertEqual(git_show.returncode, 0, git_show.stderr)
        with tempfile.TemporaryDirectory() as td:
            main_py = os.path.join(td, "run_batch_main.py")
            with open(main_py, "w") as f:
                f.write(git_show.stdout)
            helper = (
                "import socket, sys; sys.path.insert(0, sys.argv[1]); "
                "socket.gethostname = lambda: 'della-vis2.princeton.edu'; "
                "import run_batch_main as m; sys.exit(m.main(sys.argv[2:]))"
            )
            for case in vis_cases:
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    rc = rb.main(case, env={}, hostname="della-vis2.princeton.edu")
                self.assertEqual(rc, 0)
                branch_out = buf.getvalue()
                ref = subprocess.run(
                    [sys.executable, "-I", "-c", helper, td, *case],
                    cwd=td,
                    capture_output=True,
                    text=True,
                )
                self.assertEqual(ref.returncode, 0, ref.stderr)
                self.assertEqual(branch_out, ref.stdout)


if __name__ == "__main__":
    unittest.main()
