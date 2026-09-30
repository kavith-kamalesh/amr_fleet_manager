from amr_fleet_manager.speed_policy import safety_state, speed_cap


def test_stop_wins_over_everything():
    assert safety_state(True, 0.2, 1.5) == "STOP"
    assert safety_state(True, 10.0, 1.5) == "STOP"


def test_slow_inside_zone_clear_outside():
    assert safety_state(False, 1.0, 1.5) == "SLOW"
    assert safety_state(False, 1.5, 1.5) == "CLEAR"
    assert safety_state(False, float("inf"), 1.5) == "CLEAR"


def test_slow_zone_can_be_disabled():
    assert safety_state(False, 0.6, 0.0) == "CLEAR"


def test_speed_cap_only_applies_in_slow_state():
    assert speed_cap("SLOW", 1.0, 0.25) == 0.25
    assert speed_cap("CLEAR", 1.0, 0.25) == 1.0
    assert speed_cap("STOP", 1.0, 0.25) == 1.0
    assert speed_cap("SLOW", 0.2, 0.25) == 0.2
    assert speed_cap("SLOW", 1.0, 0.0) == 1.0
