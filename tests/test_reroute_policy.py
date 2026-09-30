from amr_fleet_manager.reroute_policy import EstopTimer, plan_reroute
from amr_fleet_manager.benchmark_stop_and_wait_vs_hybrid import astar

PATH = [(0, 0), (1, 0), (2, 0)]


def test_timer_fires_only_after_threshold():
    t = EstopTimer(2.0)
    assert t.update(True, 0.0) is False
    assert t.update(True, 1.9) is False
    assert t.update(True, 2.1) is True


def test_timer_rearms_after_firing():
    t = EstopTimer(2.0)
    t.update(True, 0.0)
    assert t.update(True, 2.1) is True
    assert t.update(True, 3.0) is False
    assert t.update(True, 4.2) is True


def test_timer_resets_when_stop_clears():
    t = EstopTimer(2.0)
    t.update(True, 0.0)
    t.update(False, 1.5)
    assert t.update(True, 2.5) is False
    assert t.update(True, 4.0) is False
    assert t.update(True, 4.6) is True


def test_timer_disabled_when_threshold_is_zero():
    t = EstopTimer(0)
    assert t.update(True, 0.0) is False
    assert t.update(True, 100.0) is False


def test_plan_reroute_avoids_current_edge_and_keeps_endpoints():
    new_path, blocked, edge = plan_reroute(PATH, 0, {}, 10.0, 5.0, astar)
    assert edge == ((0, 0), (1, 0))
    assert blocked == {edge: 10.0}
    assert new_path[0] == (0, 0) and new_path[-1] == (2, 0)
    hops = list(zip(new_path, new_path[1:]))
    assert edge not in hops and (edge[1], edge[0]) not in hops


def test_plan_reroute_expires_old_blocks_and_does_not_mutate_input():
    original = {((3, 3), (3, 4)): 1.0, ((4, 4), (4, 3)): 8.0}
    snapshot = dict(original)
    _, blocked, edge = plan_reroute(PATH, 0, original, 10.0, 5.0, astar)
    assert ((3, 3), (3, 4)) not in blocked
    assert ((4, 4), (4, 3)) in blocked
    assert edge in blocked
    assert original == snapshot


def test_plan_reroute_at_end_of_path_does_nothing():
    new_path, blocked, edge = plan_reroute(PATH, 2, {}, 1.0, 5.0, astar)
    assert new_path is None and edge is None and blocked == {}


def test_plan_reroute_no_alternative_still_records_block():
    seen = {}

    def no_route(start, goal, blocked):
        seen["args"] = (start, goal, blocked)
        return None

    new_path, blocked, edge = plan_reroute(PATH, 1, {}, 1.0, 5.0, no_route)
    assert new_path is None
    assert edge == ((1, 0), (2, 0)) and edge in blocked
    assert seen["args"][0] == (1, 0) and seen["args"][1] == (2, 0)
    assert isinstance(seen["args"][2], frozenset)


def test_cooldown_blocks_a_second_reroute_until_it_expires():
    from amr_fleet_manager.reroute_policy import RerouteCooldown
    c = RerouteCooldown(2.0)
    assert c.ready(0.0) is True
    c.mark(0.0)
    assert c.ready(0.1) is False
    assert c.ready(1.9) is False
    assert c.ready(2.0) is True


def test_cooldown_is_ready_before_any_reroute():
    from amr_fleet_manager.reroute_policy import RerouteCooldown
    assert RerouteCooldown(5.0).ready(123.0) is True


def test_timer_rearm_interval_is_longer_than_threshold_when_requested():
    t = EstopTimer(2.0, rearm_sec=6.0)
    t.update(True, 0.0)
    assert t.update(True, 2.1) is True
    assert t.update(True, 5.0) is False
    assert t.update(True, 7.9) is False
    assert t.update(True, 8.2) is True


def test_turn_toward_direction_deadband_cap_and_wraparound():
    import math
    from amr_fleet_manager.reroute_policy import turn_toward
    assert turn_toward(0.0, (0, 0), (0, 1)) > 0
    assert turn_toward(0.0, (0, 0), (0, -1)) < 0
    assert turn_toward(0.0, (0, 0), (1, 0.01)) == 0.0
    assert abs(turn_toward(0.0, (0, 0), (-1, 0.001), max_w=0.5)) == 0.5
    assert turn_toward(math.pi - 0.1, (0, 0), (-1, -0.2)) > 0
