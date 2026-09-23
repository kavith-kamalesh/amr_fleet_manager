"""
tests/test_robot_common.py

Real tests against the actual amr_fleet_manager.robot_common module --
a direct import, not a simulation of it. robot_common.py has zero rclpy
dependency, so this runs anywhere with plain Python + pytest, no ROS
install required.

Run from the repo root:
    pip install pytest --break-system-packages   # if not already installed
    pytest tests/test_robot_common.py -v
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from amr_fleet_manager import robot_common


KEY = "test-fixed-key-for-pytest"


def test_sign_and_verify_roundtrip():
    payload = {'id': 1, 'nodes': [[0, 0], [0, 1]], 'window': [1.0, 2.0], 'priority': 0.5, 'seq': 3}
    sig = robot_common.sign_payload(payload, KEY)
    assert robot_common.verify_payload(payload, sig, KEY) is True


def test_wire_round_trip_via_json():
    """Signature must still verify after the exact JSON dumps/loads cycle
    the real spatial_mutex.py does when publishing/receiving over a ROS
    topic -- this catches serialization-order bugs that a plain-object
    test would miss."""
    import json
    payload = {'id': 7, 'nodes': [[2, 3], [2, 4]], 'window': [10.5, 12.5], 'priority': 0.72, 'seq': 15}
    sig = robot_common.sign_payload(payload, KEY)
    wire = json.dumps({**payload, 'sig': sig})
    received = json.loads(wire)
    recv_sig = received.pop('sig')
    assert robot_common.verify_payload(received, recv_sig, KEY) is True


def test_tampered_payload_is_rejected():
    payload = {'id': 1, 'priority': 0.5, 'seq': 1}
    sig = robot_common.sign_payload(payload, KEY)
    tampered = dict(payload)
    tampered['priority'] = 999.0  # attacker tries to claim max priority
    assert robot_common.verify_payload(tampered, sig, KEY) is False


def test_wrong_key_is_rejected():
    payload = {'id': 1, 'priority': 0.5, 'seq': 1}
    sig = robot_common.sign_payload(payload, KEY)
    assert robot_common.verify_payload(payload, sig, "a-different-key") is False


def test_missing_signature_is_rejected():
    payload = {'id': 1, 'priority': 0.5, 'seq': 1}
    assert robot_common.verify_payload(payload, None, KEY) is False
    assert robot_common.verify_payload(payload, "", KEY) is False


def test_unsigned_spoofed_packet_is_rejected():
    """Direct regression test for the exact attack verify_log_not_process.py
    demonstrates: an attacker publishes a plausible-looking packet with no
    valid signature at all."""
    forged_payload = {'id': 999, 'nodes': [[0, 0], [0, 1]], 'window': [0.0, 999.0], 'priority': 1.0, 'seq': 1}
    forged_sig = "0" * 64  # attacker has no key, can only guess/pad
    assert robot_common.verify_payload(forged_payload, forged_sig, KEY) is False


def test_mutex_clearance_constants_exist_and_are_distinct():
    """spatial_mutex.py, waypoint_nav_node.py, and edge_slm_node.py all
    import these from here rather than hardcoding string literals --
    the exact drift this file's constants were added to prevent. If
    these ever get renamed without every consumer being updated, this
    test catches it before a deploy does."""
    values = {
        robot_common.MUTEX_CLEAR,
        robot_common.MUTEX_WAIT,
        robot_common.MUTEX_REROUTE,
        robot_common.MUTEX_PARKED,
    }
    assert len(values) == 4, "one or more MUTEX_* constants collide or are missing"
