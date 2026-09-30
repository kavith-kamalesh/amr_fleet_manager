"""
tests/test_benchmark_allocation.py

Real import, not a mirror -- benchmark_fifo_vs_auction.py has zero
rclpy dependency (it's a standalone offline validation script, not a
ROS node), so simulate_allocation() can be imported and tested directly
the same way test_robot_common.py directly imports robot_common.

Run from the repo root:
    pytest tests/test_benchmark_allocation.py -v
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from amr_fleet_manager.benchmark_fifo_vs_auction import simulate_allocation


def test_single_robot_single_task_all_policies_agree():
    starts = [(0.0, 0.0)]
    tasks = [(3.0, 4.0)]
    for policy in ('fifo', 'auction', 'optimal'):
        result = simulate_allocation(1, tasks, policy, starts)
        assert result['total_distance'] == pytest.approx(5.0)
        assert result['makespan'] == pytest.approx(5.0)


def test_auction_never_worse_than_fifo_for_a_clear_nearest_case():
    starts = [(0.0, 0.0), (10.0, 10.0)]
    tasks = [(9.5, 9.5), (0.5, 0.5)]
    fifo = simulate_allocation(2, tasks, 'fifo', starts)
    auction = simulate_allocation(2, tasks, 'auction', starts)
    assert auction['total_distance'] < fifo['total_distance']


def test_optimal_matches_auction_on_a_trivial_case():
    starts = [(0.0, 0.0), (5.0, 5.0)]
    tasks = [(1.0, 0.0), (5.0, 6.0)]
    auction = simulate_allocation(2, tasks, 'auction', starts)
    optimal = simulate_allocation(2, tasks, 'optimal', starts)
    assert optimal['total_distance'] == pytest.approx(auction['total_distance'])


def test_optimal_never_worse_than_auction_within_a_single_round():
    starts = [(0.0, 0.0), (10.0, 0.0), (5.0, 10.0)]
    tasks = [(0.0, 1.0), (10.0, 1.0), (5.0, 9.0)]
    auction = simulate_allocation(3, tasks, 'auction', starts)
    optimal = simulate_allocation(3, tasks, 'optimal', starts)
    assert optimal['total_distance'] <= auction['total_distance'] + 1e-9


def test_no_tasks_returns_zero():
    result = simulate_allocation(2, [], 'auction', [(0.0, 0.0), (1.0, 1.0)])
    assert result['total_distance'] == 0.0
    assert result['makespan'] == 0.0


def test_unknown_policy_raises():
    with pytest.raises(ValueError):
        simulate_allocation(1, [(1.0, 1.0)], 'bogus_policy', [(0.0, 0.0)])
