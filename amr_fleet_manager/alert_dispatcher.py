"""
Bridges /fleet/manager_alert (internal ROS2) to an external channel
(webhook / email / SMS).

DELIBERATELY SEPARATE from health_monitor.py: a failed HTTP call to a
notification service must never be able to affect robot behaviour or
block the ROS2 executor. This node can crash, hang, or have its
webhook fail entirely, and the fleet keeps running exactly as before --
that's the point of the split.

Ships with a console/log backend by default (works with zero config).
Real dispatch (webhook, email, SMS via Twilio, etc.) is a stub you
fill in with YOUR OWN credentials -- never hardcode API keys in this
file or commit them to git. Use an environment variable or a ROS2
parameter loaded from a gitignored yaml.
"""

import json
import os
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class AlertDispatcher(Node):
    def __init__(self):
        super().__init__('alert_dispatcher')

        self.declare_parameter('backend', 'console')   # console | webhook
        self.declare_parameter('webhook_url', '')       # e.g. Slack/Teams incoming webhook
        self.declare_parameter('min_seconds_between_repeats', 30.0)  # per-robot debounce

        self.backend = self.get_parameter('backend').value
        self.webhook_url = self.get_parameter('webhook_url').value
        self.debounce = self.get_parameter('min_seconds_between_repeats').value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(String, '/fleet/manager_alert', self.alert_cb, qos)
        self.create_subscription(String, '/fleet/emergency_replacement', self.emergency_cb, qos)

        self._last_sent = {}   # robot_id -> timestamp, for debounce
        self._log = deque(maxlen=200)   # in-memory alert history for a dashboard to read

        if self.backend == 'webhook' and not self.webhook_url:
            self.get_logger().warn(
                "backend=webhook but no webhook_url set -- falling back to console"
            )
            self.backend = 'console'

        self.get_logger().info(f"AlertDispatcher up | backend={self.backend}")

    def _debounced(self, robot_id):
        now = time.time()
        last = self._last_sent.get(robot_id, 0)
        if now - last < self.debounce:
            return True
        self._last_sent[robot_id] = now
        return False

    def alert_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        rid = data.get('robot_id')
        if self._debounced(rid):
            return

        self._log.append(data)
        self._dispatch(
            severity=data.get('severity', 'UNKNOWN'),
            text=data.get('message', f"Robot {rid} health alert"),
            raw=data,
        )

    def emergency_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        text = (f"EMERGENCY: Robot {data['failing_robot']} pulled from duty "
                f"(score={data.get('failing_score')}). "
                f"Suggested cover: robot{data.get('suggested_replacement', '?')}")
        self._log.append(data)
        self._dispatch(severity="CRITICAL", text=text, raw=data)

    def _dispatch(self, severity, text, raw):
        if self.backend == 'console':
            self.get_logger().error(f"[MANAGER ALERT / {severity}] {text}")
            return

        if self.backend == 'webhook':
            threading.Thread(target=self._send_webhook, args=(severity, text, raw), daemon=True).start()

    def _send_webhook(self, severity, text, raw):
        # Off the ROS2 executor thread deliberately -- a slow/hanging
        # HTTP call must never stall message processing for the rest
        # of the fleet. Never raises back into the caller.
        try:
            import urllib.request
            body = json.dumps({"text": f"[{severity}] {text}"}).encode()
            req = urllib.request.Request(
                self.webhook_url, data=body,
                headers={"Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=5)
        except Exception as e:
            self.get_logger().warn(f"webhook dispatch failed (fleet unaffected): {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AlertDispatcher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
py
