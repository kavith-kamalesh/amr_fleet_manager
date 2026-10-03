import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "amr_fleet_manager"))

import fleet_pibt_bench as fb  # noqa: E402
import protocol_sim as ps  # noqa: E402
from fleet_protocol import FleetAgent  # noqa: E402
from pibt_core import grid_graph  # noqa: E402

NB = fb.build_graph()


def _trials(n, seeds, **kw):
    return [ps.run_protocol_trial(n, 42 * 100003 + n * 1009 + k, nbrs=NB,
                                  trial_seed=k, **kw) for k in seeds]


def test_ideal_radio_completes_without_collisions():
    rs = _trials(12, range(6))
    assert all(r["success"] for r in rs)
    assert sum(r["collisions"] for r in rs) == 0


def test_message_loss_never_causes_collisions():
    assert sum(r["collisions"] for r in _trials(12, range(8), loss=0.3)) == 0


def test_dead_radios_never_cause_collisions():
    assert sum(r["collisions"] for r in _trials(12, range(8), offline=0.2)) == 0


def test_parked_robots_are_not_driven_into():
    assert sum(r["collisions"] for r in _trials(12, range(6), dock="park")) == 0


def test_four_robot_rotation_resolves():
    # 2x2 grid, no free cell: the only legal move is a simultaneous rotation
    sc = ([(0, 0), (1, 0), (1, 1), (0, 1)], [(1, 0), (1, 1), (0, 1), (0, 0)],
          [0.9, 0.7, 0.5, 0.3])
    r = ps.run_protocol_trial(4, 0, nbrs=grid_graph(2), scenario=sc)
    assert r["success"] and r["collisions"] == 0


def test_robot_never_heard_is_still_an_obstacle():
    # robot 1 sits in the corridor and its radio never worked
    a = FleetAgent(0, grid_graph(3, 1), 0.9, (0, 0), (2, 0))
    a.seed_peer(1, (1, 0), None, 0.5)
    a.begin_tick(0)
    a.plan()
    assert a.finalize() == (0, 0)


def test_early_message_is_buffered_not_dropped():
    a = FleetAgent(0, grid_graph(3, 1), 0.9, (0, 0), (2, 0))
    a.begin_tick(0)
    a.receive({"id": 1, "tick": 1, "round": 0, "cell": [2, 0], "goal": None,
               "prio": 0.5, "docked": False, "target": None, "committed": False})
    assert 1 not in a.heard                      # not this tick yet
    a.begin_tick(1)
    assert 1 in a.heard                          # replayed at the tick boundary
