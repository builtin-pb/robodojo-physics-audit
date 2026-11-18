#!/usr/bin/env python3
"""Module-isolation helper and forward/reverse suite-order check for ``audit/tools`` unit tests."""

from __future__ import annotations

from collections.abc import Mapping
import contextlib
import os
from pathlib import Path
import re
import subprocess
import sys
import types
import unittest

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

ISOLATION_SUITES: tuple[str, ...] = (
    "audit.tools.test_evaluator_v2",
    "audit.tools.test_xylophone_strike_v2",
    "audit.tools.test_stillness_v2",
    "audit.tools.test_bottles_lift_v2",
    "audit.tools.test_predicates",
)

_REPO_TOP_PACKAGES: frozenset[str] = frozenset(
    {"client_server", "env", "scripts", "src", "task", "utils"}
)

_RAN_RE = re.compile(r"^Ran (\d+) tests? in [0-9.]+s$", re.MULTILINE)
_OK_RE = re.compile(r"^OK(?: \(skipped=(\d+)\))?$", re.MULTILINE)


def _is_tracked_module(name: str, extra_names: frozenset[str]) -> bool:
    top = name.split(".", 1)[0]
    return top in _REPO_TOP_PACKAGES or name in extra_names or top in extra_names


@contextlib.contextmanager
def preserve_repo_modules(stub_modules: Mapping[str, types.ModuleType] | None = None):
    """Run a block with optional module stubs, restoring repo/stub ``sys.modules`` and package attributes.

    Real third-party C extensions (``numpy``, ``torch``, ``scipy``, ``shapely``, ``yaml``) loaded as a
    side effect remain in ``sys.modules`` so CPython never unloads and re-initializes an extension.
    """
    stubs = dict(stub_modules or {})
    extra_names = frozenset(stubs)
    saved_entries = {
        name: mod for name, mod in sys.modules.items() if _is_tracked_module(name, extra_names)
    }
    saved_pkg_attrs = {
        name: dict(mod.__dict__)
        for name, mod in saved_entries.items()
        if isinstance(mod, types.ModuleType)
    }
    try:
        if stubs:
            sys.modules.update(stubs)
        yield
    finally:
        for name in [n for n in list(sys.modules) if _is_tracked_module(n, extra_names)]:
            if name not in saved_entries:
                sys.modules.pop(name, None)
        for name, mod in saved_entries.items():
            sys.modules[name] = mod
        for name, attrs in saved_pkg_attrs.items():
            mod = saved_entries[name]
            for k in list(mod.__dict__):
                if k not in attrs:
                    delattr(mod, k)
            for k, v in attrs.items():
                if mod.__dict__.get(k) is not v:
                    setattr(mod, k, v)


def _unittest_targets(suites: tuple[str, ...] | list[str]) -> list[str]:
    try:
        import importlib

        importlib.import_module("audit.tools")
        return list(suites)
    except ModuleNotFoundError:
        # On RHEL hosts where system ``audit.py`` (libaudit) shadows the ``audit/`` namespace directory,
        # load the bare module names from ``audit/tools`` on ``PYTHONPATH`` in one process in order.
        return [s.rsplit(".", 1)[-1] for s in suites]


def run_unittest_suites(suites: tuple[str, ...] | list[str]) -> tuple[int, int, str]:
    """Run ``suites`` in one fresh Python process and return ``(ran_count, skipped_count, stderr)``."""
    env = dict(os.environ)
    env.setdefault("OPENBLAS_THREAD_TIMEOUT", "4")
    existing_pp = env.get("PYTHONPATH", "")
    repo_str = str(REPO_ROOT)
    tools_str = str(REPO_ROOT / "audit/tools")
    pp_parts = [p for p in existing_pp.split(os.pathsep) if p]
    for needed in (tools_str, repo_str):
        if needed not in pp_parts:
            pp_parts.insert(0, needed)
    env["PYTHONPATH"] = os.pathsep.join(pp_parts)
    targets = _unittest_targets(suites)
    proc = subprocess.run(
        [sys.executable, "-m", "unittest", *targets],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=600,
    )
    if proc.returncode != 0:
        raise AssertionError(
            f"unittest failed (rc={proc.returncode}) for order {list(suites)}:\n"
            f"{proc.stderr[-4000:]}"
        )
    ran_m = _RAN_RE.search(proc.stderr)
    ok_m = _OK_RE.search(proc.stderr)
    if ran_m is None or ok_m is None:
        raise AssertionError(
            f"Could not parse clean OK summary for order {list(suites)}:\n{proc.stderr[-4000:]}"
        )
    ran = int(ran_m.group(1))
    skipped = int(ok_m.group(1) or 0)
    return ran, skipped, proc.stderr


class SuiteOrderIsolationTests(unittest.TestCase):
    def test_preserve_repo_modules_restores_seed_manager_and_pkg_attr(self) -> None:
        had_sm = "env.seed_manager.seed_manager" in sys.modules
        had_env = "env" in sys.modules
        had_env_attr = hasattr(sys.modules["env"], "seed_manager") if had_env else False

        stub_mods: dict[str, types.ModuleType] = {}
        try:
            import yaml as _yaml  # noqa: F401
        except ImportError:
            stub_mods["yaml"] = types.ModuleType("yaml")

        with preserve_repo_modules(stub_mods):
            import env.seed_manager.seed_manager as sm_mod

            self.assertTrue(hasattr(sm_mod, "SeedManager"))
            self.assertIn("env.seed_manager.seed_manager", sys.modules)

        self.assertEqual("env.seed_manager.seed_manager" in sys.modules, had_sm)
        if had_env:
            self.assertEqual(hasattr(sys.modules["env"], "seed_manager"), had_env_attr)
        else:
            self.assertNotIn("env", sys.modules)

    def test_preserve_repo_modules_restores_eval_client_pkg_attr(self) -> None:
        import src.eval_client as ec_pkg

        had_ee_mod = "src.eval_client.eval_env" in sys.modules
        had_ee_attr = hasattr(ec_pkg, "eval_env")
        fake_ee = types.ModuleType("src.eval_client.eval_env")
        with preserve_repo_modules():
            sys.modules["src.eval_client.eval_env"] = fake_ee
            setattr(ec_pkg, "eval_env", fake_ee)
            self.assertIs(getattr(ec_pkg, "eval_env"), fake_ee)

        self.assertEqual("src.eval_client.eval_env" in sys.modules, had_ee_mod)
        self.assertEqual(hasattr(ec_pkg, "eval_env"), had_ee_attr)

    def test_suites_pass_in_both_orders(self) -> None:
        fwd_ran, fwd_skipped, _ = run_unittest_suites(ISOLATION_SUITES)
        rev_ran, rev_skipped, _ = run_unittest_suites(tuple(reversed(ISOLATION_SUITES)))
        self.assertGreater(fwd_ran, 0)
        self.assertEqual((fwd_ran, fwd_skipped), (rev_ran, rev_skipped))


if __name__ == "__main__":
    unittest.main(verbosity=2)
