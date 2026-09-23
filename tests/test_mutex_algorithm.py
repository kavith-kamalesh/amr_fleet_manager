"""
tests/test_mutex_algorithm.py

IMPORTANT -- what this file is and isn't:

These are REFERENCE-ALGORITHM tests, not direct imports of
spatial_mutex.py. SpatialMutex inherits from rclpy.node.Node, so it
cannot be instantiated without a working rclpy install -- which this
Mac doesn't have. The functions below are exact reproductions of the
formulas in spatial_mutex.py (peer_has_priority, effective_priority,
the sequence-number replay check in peer_intent_callback, and the
neighbor-radius filter in mutex_loop) -- copy-verify them against the
real file if you ever change one without updating the other.

Once ROS is available (on the WSL2/Gazebo machine), the stronger version
of this file would import SpatialMutex directly and call its real
methods instead of these mirrors. That's a follow-up, not done here --
flagging the gap rather than pretending this file closes it.

Run: pytest tests/test_mutex_algorithm.py -v
"""

import math


# ---------------- Mirrors of spatial_mutex.py's priority/aging formulas ----------------

PRIORITY_AGING_RATE = 0.05


def effective_priority(base_priority, wait_start_time, now):
    if wait_start_time is None:
        return base_priority
    return base_priority + PRIORITY_AGING_RATE * (now - wait_start_time)


def peer_has_priority(self_id, self_eff_priority, peer_id, peer_priority):
    if peer_priority > self_eff_priority:
        return True
    if peer_priority < self_eff_priority:
        return False
    return peer_id < self_id


# ---------------- Mirrors of the neighbor-radius filter ----------------

CELL = 2.0
NEIGHBOR_RADIUS_M = 4 * CELL


def node_pos(node):
    return (node[0] * CELL, node[1] * CELL)


def edge_midpoint(nodes):
    positions = [node_pos(n) for n in nodes]
    mx = sum(p[0] for p in positions) / len(positions)
    my = sum(p[1] for p in positions) / len(positions)
    return (mx, my)


def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


# ================================ Tests ================================

def test_equal_priority_tie_resolves_to_exactly_one_winner():
    """The bug this tie-break exists to prevent: two robots at equal
    priority both yielding to each other forever."""
    now = 100.0
    a_eff = effective_priority(0.5, None, now)
    b_eff = effective_priority(0.5, None, now)

    a_blocked_by_b = peer_has_priority(self_id=1, self_eff_priority=a_eff, peer_id=2, peer_priority=b_eff)
    b_blocked_by_a = peer_has_priority(self_id=2, self_eff_priority=b_eff, peer_id=1, peer_priority=a_eff)

    assert a_blocked_by_b != b_blocked_by_a, "both or neither blocked -- this is a deadlock"


def test_priority_aging_eventually_overrides_a_higher_static_priority():
    """A robot with low base priority that's been waiting long enough
    must eventually win against a robot with higher base priority that
    hasn't had to wait at all -- this is what prevents starvation."""
    now = 100.0
    low_priority_robot_wait_start = now - 30.0  # waited 30s
    low_eff = effective_priority(base_priority=0.1, wait_start_time=low_priority_robot_wait_start, now=now)
    high_eff = effective_priority(base_priority=0.9, wait_start_time=None, now=now)

    assert low_eff > high_eff, (
        f"after 30s of waiting, base 0.1 should have aged past base 0.9 "
        f"(got {low_eff:.3f} vs {high_eff:.3f}) -- check PRIORITY_AGING_RATE"
    )

    low_wins = not peer_has_priority(self_id=9, self_eff_priority=low_eff, peer_id=1, peer_priority=high_eff)
    assert low_wins, "aged-up low-priority robot should now win against the un-aged high-priority one"


def test_no_waiting_means_no_aging():
    """A robot that has never waited must report exactly its base
    priority, regardless of how much time has passed -- aging only
    applies to time spent actually blocked."""
    assert effective_priority(0.42, wait_start_time=None, now=99999.0) == 0.42


def test_replay_of_same_sequence_number_is_rejected():
    """Mirrors peer_intent_callback's replay guard: a sequence number
    that isn't strictly greater than the last one accepted from that
    peer must be dropped."""
    def accepts(last_seq, incoming_seq):
        return last_seq is None or incoming_seq > last_seq

    assert accepts(None, 1) is True          # first message from a new peer
    assert accepts(5, 6) is True              # normal increment
    assert accepts(5, 5) is False             # exact replay
    assert accepts(5, 3) is False             # stale/out-of-order replay
    assert accepts(5, 4) is False


def test_neighbor_filter_keeps_a_genuinely_adjacent_peer():
    """Two robots on edges that share a grid node must always be within
    NEIGHBOR_RADIUS_M of each other on this grid -- if this test ever
    fails, NEIGHBOR_RADIUS_M has been set too small and the filter would
    start hiding real conflicts."""
    self_edge = [(2, 2), (2, 3)]
    peer_edge_sharing_a_node = [(2, 3), (2, 4)]

    d = distance(edge_midpoint(self_edge), edge_midpoint(peer_edge_sharing_a_node))
    assert d <= NEIGHBOR_RADIUS_M, (
        f"adjacent edges are {d}m apart but NEIGHBOR_RADIUS_M is only "
        f"{NEIGHBOR_RADIUS_M}m -- filtering would incorrectly hide this peer"
    )


def test_neighbor_filter_excludes_a_genuinely_distant_peer():
    """A peer on the opposite side of a reasonably sized grid should be
    filtered out -- this is the actual computational saving the filter
    exists for."""
    self_edge = [(0, 0), (0, 1)]
    far_peer_edge = [(10, 10), (10, 11)]

    d = distance(edge_midpoint(self_edge), edge_midpoint(far_peer_edge))
    assert d > NEIGHBOR_RADIUS_M, "this peer should be far enough to be filtered"
