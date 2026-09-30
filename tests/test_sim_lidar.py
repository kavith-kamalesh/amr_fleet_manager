import importlib.util
import pathlib

_p = pathlib.Path(__file__).resolve().parent.parent / "scripts" / "sim_lidar.py"
_spec = importlib.util.spec_from_file_location("sim_lidar", _p)
sim_lidar = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim_lidar)

FRONT, REAR = 180, 0   # ray 180 is angle 0 (straight ahead); ray 0 is angle -pi (behind)


def test_robot_ahead_is_seen_at_surface_distance_and_rear_is_clear():
    r = sim_lidar.scan_ranges((0.0, 0.0), 0.0, [(1.0, 0.0)])
    assert abs(r[FRONT] - 0.7) < 1e-9
    assert r[REAR] == sim_lidar.RANGE_MAX


def test_robot_behind_is_seen_only_at_the_rear():
    r = sim_lidar.scan_ranges((0.0, 0.0), 0.0, [(-1.0, 0.0)])
    assert r[FRONT] == sim_lidar.RANGE_MAX
    assert abs(r[REAR] - 0.7) < 1e-9


def test_heading_rotates_the_scan():
    r = sim_lidar.scan_ranges((0.0, 0.0), 3.141592653589793 / 2, [(0.0, 1.0)])
    assert abs(r[FRONT] - 0.7) < 1e-9


def test_far_robot_is_invisible():
    r = sim_lidar.scan_ranges((0.0, 0.0), 0.0, [(50.0, 0.0)])
    assert set(r) == {sim_lidar.RANGE_MAX}
