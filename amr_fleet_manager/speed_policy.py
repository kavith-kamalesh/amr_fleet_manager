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
