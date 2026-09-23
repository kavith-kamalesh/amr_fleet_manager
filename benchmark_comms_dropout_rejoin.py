"""
benchmark_comms_dropout_rejoin.py

Failure-injection test for the "communication-aware rejoin" gap: the
deployed spatial_mutex.py only treats a comms gap as dangerous when a
robot SELF-REPORTS CHARGING/SHIFT_CHANGE. A real Wi-Fi dropout while a
robot is actively negotiating is invisible to that logic -- it just
prunes the silent peer's stale data after peer_timeout and reports
CLEAR to itself based on absence of evidence, not confirmed clearance.

Scenario: Robot A holds a real, ongoing reservation on a contested edge
the entire time (it never moves, never releases it -- this is the
ground truth). Robot B is negotiating the same edge and correctly
starts out blocked by A's higher priority. At t=1.0s, B's radio drops
out -- it simply stops RECEIVING any message from A (A keeps
broadcasting normally; this is a receive-side failure, the realistic
case for a Wi-Fi dead zone). At t=4.0s, B's radio recovers.

OLD (charging-only) logic: B has no dropout awareness at all. Once
A's last-seen peer_intent entry exceeds peer_timeout (1.5s), B's
prune_stale_peers() simply deletes it, and mutex_loop finds no
recorded conflict -- B reports CLEAR to itself, meaning it would begin
moving into a still-actually-occupied edge, based purely on having
stopped hearing from A. This is the dangerous window being measured.

NEW (generalized) logic: B tracks time-since-last-ANY-peer-message
independently of self-reported state. Once that gap exceeds
COMMS_DROPOUT_THRESHOLD_SEC (set shorter than peer_timeout, so the
dropout is caught before stale data would even be pruned), B refuses
to report CLEAR regardless of what prune_stale_peers has or hasn't
removed -- it holds. When messages resume, B holds for an additional
REJOIN_GRACE_SEC before trusting fresh data, mirroring the existing
charging-recovery behavior but now triggered by ANY dropout.

Metric: "unsafe CLEAR ticks" -- how many simulation ticks does B report
CLEAR while A's ground-truth reservation on the shared edge is still
actually active. Under OLD logic this should be large (the entire
post-prune blackout duration). Under NEW logic it should be zero.

Run: python3 benchmark_comms_dropout_rejoin.py
"""

DT = 0.1
PEER_TIMEOUT_SEC = 1.5
COMMS_DROPOUT_THRESHOLD_SEC = 1.0   # shorter than PEER_TIMEOUT_SEC -- catches the gap before stale data is even pruned
REJOIN_GRACE_SEC = 1.0
SIM_DURATION_SEC = 6.0

BLACKOUT_START = 1.0
BLACKOUT_END = 4.0

MUTEX_CLEAR = "CLEAR"
MUTEX_WAIT = "WAIT"


class RobotBOldLogic:
    """Mirrors the CURRENTLY DEPLOYED spatial_mutex.py: comms-gap
    awareness only exists inside enter_low_power_mode, which only fires
    on self-reported CHARGING/SHIFT_CHANGE. This robot is never told
    it's charging (it's actively negotiating), so none of that logic
    ever engages -- it behaves exactly as today's code does for a real
    Wi-Fi dropout: nothing special happens at all."""

    def __init__(self):
        self.last_seen_a = None  # last time a message from A was actually received

    def receive_from_a(self, now, message_arrives: bool):
        if message_arrives:
            self.last_seen_a = now

    def decide_clearance(self, now):
        if self.last_seen_a is None:
            return MUTEX_WAIT  # no data yet at all -- startup, not the case under test
        if now - self.last_seen_a > PEER_TIMEOUT_SEC:
            # prune_stale_peers() has deleted A's entry -- no known
            # conflict on record -- old logic reports CLEAR here.
            return MUTEX_CLEAR
        return MUTEX_WAIT  # still holds valid (unpruned) data showing A has the edge


class RobotBNewLogic:
    """Generalized rejoin: comms-health tracked independently of
    self-reported state. ANY gap since the last message from ANY peer
    triggers a hold, and recovery triggers a grace period before
    trusting fresh data -- not just the charging case."""

    def __init__(self):
        self.last_seen_a = None
        self.last_seen_any_peer = None
        self.in_dropout = False
        self.rejoin_until = None

    def receive_from_a(self, now, message_arrives: bool):
        if message_arrives:
            was_in_dropout = self.in_dropout
            self.last_seen_a = now
            self.last_seen_any_peer = now
            self.in_dropout = False
            if was_in_dropout:
                # Recovery: mirrors exit_low_power_mode's
                # request_state_sync(), generalized to fire on ANY
                # dropout ending, not just CHARGING ending.
                self.rejoin_until = now + REJOIN_GRACE_SEC

    def decide_clearance(self, now):
        if self.last_seen_any_peer is None:
            # Never having heard from ANY peer is NOT the same as having
            # lost contact with one -- this is startup / genuinely no
            # peers nearby, which correctly defaults to CLEAR (matches
            # mutex_loop's actual default when current_edge_nodes has no
            # recorded conflict). Only a peer we'd previously heard from
            # going silent is the dangerous case this fix targets.
            return MUTEX_CLEAR

        gap = now - self.last_seen_any_peer
        if gap > COMMS_DROPOUT_THRESHOLD_SEC:
            self.in_dropout = True
            return MUTEX_WAIT  # don't trust silence as clearance

        if self.rejoin_until is not None and now < self.rejoin_until:
            return MUTEX_WAIT  # just recovered -- hold through the grace period

        if now - self.last_seen_a > PEER_TIMEOUT_SEC:
            return MUTEX_CLEAR  # genuinely stale AND comms are healthy -- A must have actually released

        return MUTEX_WAIT


def run_comparison():
    t = 0.0
    old_robot = RobotBOldLogic()
    new_robot = RobotBNewLogic()

    old_unsafe_ticks = 0
    new_unsafe_ticks = 0
    total_ticks = 0

    rows = []

    while t < SIM_DURATION_SEC:
        t += DT
        total_ticks += 1

        # Ground truth: A holds the edge for the entire simulation --
        # it never moves, never releases. A keeps broadcasting every
        # tick regardless of B's radio state (A's own radio is fine).
        a_actually_holds_edge = True
        message_arrives_at_b = not (BLACKOUT_START <= t < BLACKOUT_END)

        old_robot.receive_from_a(t, message_arrives_at_b)
        new_robot.receive_from_a(t, message_arrives_at_b)

        old_clearance = old_robot.decide_clearance(t)
        new_clearance = new_robot.decide_clearance(t)

        old_unsafe = a_actually_holds_edge and old_clearance == MUTEX_CLEAR
        new_unsafe = a_actually_holds_edge and new_clearance == MUTEX_CLEAR

        old_unsafe_ticks += int(old_unsafe)
        new_unsafe_ticks += int(new_unsafe)

        rows.append((round(t, 1), message_arrives_at_b, old_clearance, new_clearance, old_unsafe, new_unsafe))

    return rows, old_unsafe_ticks, new_unsafe_ticks, total_ticks


def run_isolated_robot_check():
    """Sanity check for the fix I just made: a robot that has NEVER
    heard from any peer (fleet startup, or genuinely alone) must still
    default to CLEAR immediately -- it must not be held hostage waiting
    for a peer that may not exist. This is what distinguishes 'never
    connected' from 'lost a connection'."""
    robot = RobotBNewLogic()
    decision_at_t0 = robot.decide_clearance(0.1)
    return decision_at_t0


def main():
    isolated_decision = run_isolated_robot_check()
    print(f"Isolated-robot sanity check (never heard from any peer): {isolated_decision}")
    assert isolated_decision == MUTEX_CLEAR, "REGRESSION: an isolated robot must default to CLEAR, not hold forever"
    print("(correct -- an isolated/startup robot is not held hostage waiting for a peer that may not exist)\n")

    rows, old_unsafe, new_unsafe, total = run_comparison()

    print(f"Scenario: Robot A holds a contested edge for the entire {SIM_DURATION_SEC}s run.")
    print(f"Robot B's radio drops out from t={BLACKOUT_START}s to t={BLACKOUT_END}s "
          f"({BLACKOUT_END - BLACKOUT_START}s blackout, A's messages simply never arrive at B).\n")

    print(f"{'t':>5} | {'msg_arrives':>11} | {'OLD decision':>13} | {'NEW decision':>13} | {'OLD unsafe':>10} | {'NEW unsafe':>10}")
    print("-" * 80)
    for t, arrives, old_c, new_c, old_u, new_u in rows:
        if BLACKOUT_START - 0.2 <= t <= BLACKOUT_END + 1.5 or t < 0.3 or t > SIM_DURATION_SEC - 0.3:
            print(f"{t:>5} | {str(arrives):>11} | {old_c:>13} | {new_c:>13} | {str(old_u):>10} | {str(new_u):>10}")

    print(f"\n=== Result ===")
    print(f"OLD (charging-only rejoin): {old_unsafe}/{total} ticks reported CLEAR while A actually held the edge "
          f"({100*old_unsafe/total:.0f}% of the run) -- B would have believed it was safe to move for "
          f"{old_unsafe*DT:.1f}s while A was genuinely still there.")
    print(f"NEW (generalized rejoin):   {new_unsafe}/{total} ticks unsafe.")

    if old_unsafe > 0 and new_unsafe == 0:
        print(f"\nGeneralized rejoin eliminates the unsafe window entirely for this scenario.")
    elif new_unsafe > 0:
        print(f"\nWARNING: new logic still has {new_unsafe} unsafe ticks -- investigate before trusting this fix.")


if __name__ == '__main__':
    main()
