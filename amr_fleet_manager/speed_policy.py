"""Pure speed-and-separation logic (no rclpy, so it is unit-testable)."""

STOP, SLOW, CLEAR = "STOP", "SLOW", "CLEAR"


def safety_state(stopped, front_min, slow_distance):
    """STOP while the e-stop latch is set; SLOW inside slow_distance; otherwise CLEAR.

    `stopped` is the latched e-stop flag (with hysteresis) owned by safety_fallback.
    slow_distance <= 0 disables the slow zone.
    """
    if stopped:
        return STOP
    if slow_distance > 0 and front_min < slow_distance:
        return SLOW
    return CLEAR


def speed_cap(state, speed, slow_speed):
    """Commanded linear speed for a safety state. slow_speed <= 0 disables slowing."""
    if state == SLOW and slow_speed > 0:
        return min(speed, slow_speed)
    return speed


class SafetyWatchdog:
    """Fail-safe: a silent safety channel counts as STOP.

    safety_fallback publishes only when a LiDAR scan arrives, so silence means no scans (dead sensor,
    dead bridge, or a graph that has not finished discovery). timeout_sec <= 0 disables (old behavior).
    Silence is measured from construction if nothing has ever been heard.
    """

    def __init__(self, timeout_sec, now):
        self.timeout_sec = float(timeout_sec)
        self._last = now

    def heard(self, now):
        self._last = now

    def silent(self, now):
        return self.timeout_sec > 0 and (now - self._last) > self.timeout_sec
