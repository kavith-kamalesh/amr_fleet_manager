import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "amr_fleet_manager"))

import fleet_pibt_bench as b  # noqa: E402

NBRS = b.build_graph()


def _run(policy, n, seeds=range(8)):
    return [b.run_trial(policy, b.make_scenario(n, s), nbrs=NBRS) for s in seeds]


def test_no_collisions_any_policy():
    for p in b.POLICIES:
        for r in _run(p, 12):
            assert r["collisions"] == 0, p


def test_decentral_completes_24_robots():
    assert all(r["success"] for r in _run("decentral", 24, range(15)))


def test_decentral_completes_48_robots_collision_free():
    rs = _run("decentral", 48, range(8))
    assert all(r["success"] for r in rs)
    assert sum(r["collisions"] for r in rs) == 0


def test_head_on_swap_deadlocks_stop_and_wait_but_not_decentral():
    sc = ([(0, 0), (1, 0)], [(1, 0), (0, 0)], [0.9, 0.1])
    assert not b.run_trial("stop_and_wait", sc, nbrs=NBRS)["success"]
    r = b.run_trial("decentral", sc, nbrs=NBRS)
    assert r["success"] and r["collisions"] == 0


def test_dropout_never_causes_collisions():
    for k in range(8):
        sc = b.make_scenario(24, k)
        r = b.run_trial("decentral", sc, dropout=0.3, seed=k, nbrs=NBRS)
        assert r["collisions"] == 0


def test_deterministic():
    a = b.run_trial("decentral", b.make_scenario(24, 3), nbrs=NBRS)
    c = b.run_trial("decentral", b.make_scenario(24, 3), nbrs=NBRS)
    assert a == c


# ---------------- breakdown / planner-outage experiment ----------------
def _fail_trial(case, n, seed):
    name, pol, brk, dead = next(c for c in b.FAIL_CASES if c[0] == case)
    sc = b.make_scenario(n, seed)
    return b.run_trial(pol, sc, nbrs=NBRS, events=b.make_events(sc, seed, brk, dead))


def test_breakdown_cases_never_collide():
    for name, *_ in b.FAIL_CASES:
        for s in range(6):
            assert _fail_trial(name, 12, s)["collisions"] == 0, name


def test_decentral_survives_breakdown_most_of_the_time():
    ok = sum(_fail_trial("decentral", 24, s)["success"] for s in range(20))
    assert ok >= 16


def test_planner_outage_hurts_central_more_than_decentral_at_scale():
    c = sum(_fail_trial("central_dead", 48, s)["success"] for s in range(10))
    d = sum(_fail_trial("decentral", 48, s)["success"] for s in range(10))
    assert d > c


def test_live_planner_replans_around_breakdown():
    ok = sum(_fail_trial("central_live", 24, s)["success"] for s in range(20))
    assert ok >= 16


def test_infeasible_task_is_excluded_not_counted_as_failure():
    # robot 1's goal is exactly where robot 0 freezes -> nobody can serve it
    sc = ([(0, 0), (0, 3), (6, 6)], [(5, 0), (0, 0), (6, 9)], [0.5, 0.4, 0.3])
    ev = dict(broken=[0], break_tick=0, planner_dead_tick=None)
    r = b.run_trial("decentral", sc, nbrs=NBRS, events=ev)
    assert r["success"] and r["unfinished"] == 0
