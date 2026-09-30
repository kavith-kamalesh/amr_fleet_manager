"""Pure reroute-decision logic for waypoint_nav_node (no rclpy, so it is unit-testable).

EstopTimer   - fires once the LiDAR emergency stop has been continuously active
               for threshold_sec, then re-arms. threshold_sec <= 0 disables it.
plan_reroute - blocks the edge the robot is currently on (entries expire after
               expiry_sec) and asks the supplied A* function for a new path.
"""


class EstopTimer:
    def __init__(self, threshold_sec, rearm_sec=None):
        self.threshold_sec = float(threshold_sec)
        # After firing, wait rearm_sec (default: same as threshold_sec) before firing again.
        self.rearm_sec = self.threshold_sec if rearm_sec is None else float(rearm_sec)
        self._since = None

    def update(self, stopped, now):
        if self.threshold_sec <= 0 or not stopped:
            self._since = None
            return False
        if self._since is None:
            self._since = now
            return False
        if now - self._since > self.threshold_sec:
            self._since = now + (self.rearm_sec - self.threshold_sec)
            return True
        return False


def prune_blocked(blocked_edges, now, expiry_sec):
    return {e: t for e, t in blocked_edges.items() if now - t < expiry_sec}


def plan_reroute(path, path_idx, blocked_edges, now, expiry_sec, astar):
    """Returns (new_path_or_None, blocked_edges, edge_or_None).

    edge is None when the robot has no current edge (nothing to reroute).
    The input dict is never mutated.
    """
    blocked = prune_blocked(blocked_edges, now, expiry_sec)
    if path is None or path_idx >= len(path) - 1:
        return None, blocked, None
    edge = (path[path_idx], path[path_idx + 1])
    blocked[edge] = now
    new_path = astar(path[path_idx], path[-1], frozenset(blocked))
    return new_path, blocked, edge


class RerouteCooldown:
    """Ignore further reroute requests for cooldown_sec after one was applied.

    After a reroute the node still holds the previous REROUTE_REQUESTED message until the
    mutex publishes a fresh clearance for the new edge; acting on that stale message blocks
    the new path's first edge too. The benchmark avoids this by resetting its wait timer, so
    this makes the node match it.
    """

    def __init__(self, cooldown_sec):
        self.cooldown_sec = float(cooldown_sec)
        self._last = None

    def ready(self, now):
        return self._last is None or now - self._last >= self.cooldown_sec

    def mark(self, now):
        self._last = now


def turn_toward(yaw, cur_xy, target_xy, max_w=0.5, gain=1.5, deadband=0.1):
    """Angular velocity for an in-place turn toward target_xy; 0.0 inside the deadband."""
    import math
    err = math.atan2(target_xy[1] - cur_xy[1], target_xy[0] - cur_xy[0]) - yaw
    err = math.atan2(math.sin(err), math.cos(err))
    if abs(err) < deadband:
        return 0.0
    return max(-max_w, min(max_w, gain * err))


class GoalGate:
    """Hold a goal that arrives before the first odometry message.

    Before the first odom the node's position is its 0.0 default (spawn offset not yet applied),
    so planning then starts from the wrong grid node -- or, for a goal within half a cell of the
    origin, from the goal itself, giving a one-node path that reports "Goal reached" instantly.
    """

    def __init__(self):
        self.ready = False
        self.pending = None

    def on_goal(self, goal_xy):
        """Returns goal_xy if it can be planned now, else stores it and returns None."""
        if self.ready:
            return goal_xy
        self.pending = goal_xy
        return None

    def on_odom(self):
        """Call on every odom message. Returns the stored goal exactly once, on the first odom."""
        if self.ready:
            return None
        self.ready = True
        goal, self.pending = self.pending, None
        return goal
