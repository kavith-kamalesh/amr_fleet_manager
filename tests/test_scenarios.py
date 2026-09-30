from amr_fleet_manager import nav_graph
from amr_fleet_manager.scenarios import CROSSING, SCENARIOS, SPAWN


def _paths(goals):
    return {r: nav_graph.astar(nav_graph.node_of(*SPAWN[r]), nav_graph.node_of(*g)) for r, g in goals.items()}


def test_both_scenarios_are_registered():
    assert set(SCENARIOS) == {"swap", "crossing"}


def test_crossing_goal_coordinates_snap_to_the_intended_nodes():
    assert [nav_graph.node_of(*CROSSING[r]) for r in ("robot1", "robot2", "robot3")] == [(3, 0), (2, 3), (4, 2)]


def test_crossing_goals_are_distinct_and_not_anyones_start():
    goals = [nav_graph.node_of(*g) for g in CROSSING.values()]
    starts = [nav_graph.node_of(*p) for p in SPAWN.values()]
    assert len(set(goals)) == len(goals)
    assert not set(goals) & set(starts)


def test_crossing_has_real_conflicts_and_nobody_parks_on_a_later_leg_of_another_path():
    paths = _paths(CROSSING)
    assert all(p is not None for p in paths.values())
    names = sorted(paths)
    shared = [(a, b) for i, a in enumerate(names) for b in names[i + 1:] if set(paths[a]) & set(paths[b])]
    assert len(shared) >= 2
    for r, p in paths.items():
        for o, q in paths.items():
            if r != o:
                assert p[-1] not in q[1:]
