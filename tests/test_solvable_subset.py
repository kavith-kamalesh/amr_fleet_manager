import importlib.util
import pathlib

_p = pathlib.Path(__file__).resolve().parent.parent / "scripts" / "analyze_solvable_subset.py"
_spec = importlib.util.spec_from_file_location("analyze_solvable_subset", _p)
mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mod)


def test_stats_and_improvement_with_timeout_penalty():
    trials = {
        0: {"stop_and_wait": (True, None), "hybrid": (False, 20.0)},
        1: {"stop_and_wait": (False, 30.0), "hybrid": (False, 30.0)},
    }
    s = mod.stats(trials, [0, 1])
    assert s["stop_and_wait"] == (1, 45.0)
    assert s["hybrid"] == (2, 25.0)
    assert abs(mod.improvement(s) - 100.0 * 20.0 / 45.0) < 1e-9
