import importlib.util
import pathlib

_p = pathlib.Path(__file__).resolve().parent.parent / "scripts" / "smoke_odom.py"
_spec = importlib.util.spec_from_file_location("smoke_odom", _p)
smoke_odom = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(smoke_odom)


def test_no_breach_when_robots_stay_apart():
    rows = [(0.0, "a", 0.0, 0.0), (0.0, "b", 1.0, 0.0), (1.0, "a", 0.0, 0.0), (1.0, "b", 1.0, 0.0)]
    r = smoke_odom.analyze_rows(rows, 0.7)
    assert abs(r["min_d"] - 1.0) < 1e-9 and r["breach_s"] == 0.0 and r["first_breach"] is None


def test_overlap_is_reported_with_duration_and_path_length():
    rows = [(0.0, "a", 0.0, 0.0), (0.0, "b", 3.0, 0.0),
            (1.0, "a", 3.0, 0.0), (2.0, "b", 3.0, 0.0), (3.0, "a", 3.0, 0.0)]
    r = smoke_odom.analyze_rows(rows, 0.7)
    assert r["min_d"] == 0.0
    assert r["breach_s"] == 2.0 and r["first_breach"] == 1.0
    assert abs(r["travelled"]["a"] - 3.0) < 1e-9


def test_pair_breaches_counts_time_even_with_a_third_robot_interleaved():
    rows = []
    for k in range(21):
        t = k * 0.1
        rows += [(t, "a", 0.0, 0.0), (t, "b", 0.2, 0.0), (t, "c", 9.0, 9.0)]
    secs, mins, any_s = smoke_odom.pair_breaches(rows, 0.7)
    assert abs(secs[("a", "b")] - 2.1) < 0.15
    assert ("a", "c") not in secs
    assert abs(any_s - secs[("a", "b")]) < 1e-9
    assert abs(mins[("a", "b")] - 0.2) < 1e-9
