"""
Smoke test for pibt_fleet_node.py WITHOUT ROS: rclpy is replaced by a tiny
in-memory stub (shared topic bus + fake clock + point-mass robots that drive to
the published waypoints). It checks the node's wiring end to end -- tick
scheduling, negotiation rounds, waypoint publishing, odometry-based arrival.
It does NOT test DDS, Gazebo or real timing. Skipped when real rclpy exists.
"""
import importlib.util
import json
import math
import os
import sys
import types

import pytest

if importlib.util.find_spec("rclpy") is not None:
    pytest.skip("real rclpy installed; stub smoke test not needed", allow_module_level=True)

INNER = os.path.join(os.path.dirname(__file__), "..", "amr_fleet_manager")


# ----------------------------------------------------------- tiny ROS stub
class _Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Bus:
    subs, t = {}, 0.0


class _Param:
    def __init__(self, v):
        self.value = v


class _Pub:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        for cb in list(Bus.subs.get(self.topic, [])):
            cb(msg)


class _Log:
    def info(self, *a, **k): pass
    warn = error = debug = info


class _Clock:
    def now(self):
        return _Obj(nanoseconds=int(Bus.t * 1e9))


class Node:
    overrides, ns = {}, ""

    def __init__(self, name):
        self._p, self._timers = {}, []
        self._ns = Node.ns

    def declare_parameter(self, k, default):
        self._p[k] = _Param(Node.overrides.get(k, default))

    def get_parameter(self, k):
        return self._p[k]

    def _topic(self, t):
        return t if t.startswith("/") else f"{self._ns}/{t}"

    def create_subscription(self, typ, topic, cb, qos):
        Bus.subs.setdefault(self._topic(topic), []).append(cb)

    def create_publisher(self, typ, topic, qos):
        return _Pub(self._topic(topic))

    def create_timer(self, period, cb):
        self._timers.append(cb)

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Log()

    def destroy_node(self): pass


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    mod("rclpy", init=lambda args=None: None, spin=lambda n: None, shutdown=lambda: None)
    mod("rclpy.node", Node=Node)
    mod("rclpy.qos", QoSProfile=lambda **k: None,
        ReliabilityPolicy=_Obj(RELIABLE=1, BEST_EFFORT=2), HistoryPolicy=_Obj(KEEP_LAST=1))

    def pose():
        return _Obj(header=_Obj(frame_id=""),
                    pose=_Obj(position=_Obj(x=0.0, y=0.0, z=0.0),
                              orientation=_Obj(w=1.0)))
    mod("geometry_msgs", msg=None)
    mod("geometry_msgs.msg", PoseStamped=pose)
    mod("nav_msgs", msg=None)
    mod("nav_msgs.msg", Odometry=lambda: None)
    mod("std_msgs", msg=None)
    mod("std_msgs.msg", String=lambda data="": _Obj(data=data))
    pkg = types.ModuleType("amr_fleet_manager")
    pkg.__path__ = [INNER]
    sys.modules["amr_fleet_manager"] = pkg


def test_four_robots_cross_and_reach_goals():
    _install_stubs()
    Bus.subs.clear()
    Bus.t = 0.0
    from amr_fleet_manager.pibt_fleet_node import PibtFleetNode

    starts = {1: (0, 0), 2: (4, 0), 3: (0, 4), 4: (4, 4)}
    goals = {1: (4, 4), 2: (0, 4), 3: (4, 0), 4: (0, 0)}
    prios = {1: 0.9, 2: 0.6, 3: 0.4, 4: 0.2}
    roster = {str(i): {"cell": list(starts[i]), "prio": prios[i]} for i in starts}

    nodes, pos, wp = {}, {}, {}
    for i in starts:
        Node.ns = f"/robot{i}"
        Node.overrides = dict(robot_id=i, priority=prios[i], grid_w=5, grid_h=5,
                              roster_json=json.dumps(roster), tick_period=4.0,
                              round_period=0.15, rounds=6)
        nodes[i] = PibtFleetNode()
        pos[i] = [float(starts[i][0]), float(starts[i][1])]
        wp[i] = tuple(pos[i])
        Bus.subs.setdefault(f"/robot{i}/nav_waypoint", []).append(
            lambda m, i=i: wp.__setitem__(i, (m.pose.position.x, m.pose.position.y)))

    def odom(i):
        return _Obj(pose=_Obj(pose=_Obj(position=_Obj(x=pos[i][0], y=pos[i][1]))))

    for i in starts:                                     # send task goals
        g = _Obj(pose=_Obj(position=_Obj(x=float(goals[i][0]), y=float(goals[i][1]))))
        nodes[i].goal_cb(g)

    dt, speed, closest = 0.02, 0.8, 99.0
    for step in range(int(70 / dt)):
        Bus.t += dt
        for i in starts:                                 # point-mass kinematics
            dx, dy = wp[i][0] - pos[i][0], wp[i][1] - pos[i][1]
            d = math.hypot(dx, dy)
            if d > 1e-6:
                k = min(1.0, speed * dt / d)
                pos[i][0] += dx * k
                pos[i][1] += dy * k
            nodes[i].odom_cb(odom(i))
        for i in starts:
            for cb in nodes[i]._timers:
                cb()
        ids = list(starts)
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                closest = min(closest, math.dist(pos[ids[a]], pos[ids[b]]))

    for i in starts:
        assert math.dist(pos[i], goals[i]) < 0.3, f"robot {i} ended at {pos[i]}"
    assert closest > 0.45, f"robots came within {closest:.2f} m"
