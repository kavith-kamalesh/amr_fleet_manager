# Deployment and Update Path

This is the honest, current-state answer to "how would this actually
reach a real warehouse and stay maintained" — not a roadmap slide, a
plan grounded in what's already in this repo.

## Target hardware

Each AMR runs its ROS 2 nodes on a Raspberry Pi 5. `spatial_mutex.py`,
`waypoint_nav_node.py`, and `safety_fallback.py` run per-robot, fully
decentralized (see the architecture note in `launch/hybrid_fleet_bringup.launch.py`);
`mission_controller.py` is the one node that runs off-robot, as the
task-allocation/WMS-facing layer.

## Containerization

**Not yet built. This section states the plan, not a claim that it exists.**

Each robot's node set builds into one Docker image per ROS distro/arch
target (`arm64` for the Pi 5), tagged by git commit SHA:

```
ghcr.io/codecircuit/amr-fleet-manager:<git-sha>
```

`docker-compose.yml` (per-robot, not yet in this repo) pins each robot to
a specific image tag. A robot's exact running code is always identifiable
from its container tag — no "which version is actually on robot 3"
ambiguity, which is the failure mode a fleet without this has.

## Versioned parameters

This matters more than it sounds like for a negotiation system: `hmac_key`,
`priority_aging_rate`, `neighbor_radius_m`, `reservation_buffer_sec`, and
`reroute_wait_threshold_sec` are already ROS 2 declared parameters (see
`spatial_mutex.py`), not hardcoded constants — that was a deliberate
choice, not an accident, specifically so a fleet's negotiation behavior
can be tuned and versioned without a code change.

Each deployed configuration is a `params/<version>.yaml` file, checked
into this repo alongside the image tag it's paired with. A robot's
behavior is fully reproducible from two facts: image tag + params
version. Rolling back behavior (e.g., reverting `priority_aging_rate`
after a bad tune) is a params-file change, not a redeploy.

## OTA update path

1. New image + params version tested in simulation first
   (`benchmark_stop_and_wait_vs_hybrid.py`, `benchmark_fifo_vs_auction.py`,
   `benchmark_scalability.py` — all three already exist and are exactly
   the regression suite a real update process would gate on before
   anything touches hardware).
2. Roll out to **one robot** during its own charging/`PARKED` window —
   `spatial_mutex.py` already exempts parked robots from active
   negotiation (`enter_low_power_mode`), so an update landing during that
   window can't destabilize live traffic.
3. `mission_controller.py`'s existing heartbeat watchdog
   (`watchdog_loop`, 5s timeout) is the automatic health check: if the
   updated robot doesn't resume reporting odometry after its update, it's
   marked `OFFLINE` and its task is re-queued — the same mechanism that
   already handles a robot silently failing handles a bad update failing
   the same way, for free.
4. Only after one robot runs clean for a defined soak period does the
   update roll to the rest of the fleet, one at a time, each during its
   own parked window — never all robots simultaneously.

## Key rotation

`hmac_key` is currently a single pre-shared key (see the roadmap note in
`spatial_mutex.py` and slide 3's "X.509 / SROS2: roadmap" line — this is
the same open item, not a new one). Rotation path once that lands:
SROS2 keystores generated per-robot, distributed at provisioning time,
rotated by re-provisioning during a parked window using the same
one-robot-at-a-time rollout above — key rotation and code updates use
the identical safe-rollout mechanism, not two separate processes.

## What this deliberately does not claim

No fleet-wide atomic rollback, no automated canary analysis, no
zero-downtime guarantee. Those are real gaps for a production system and
are not solved by anything above — this is a scoped, honest "how a
6-robot pilot fleet gets updated without someone SSHing into each Pi",
not a claim of enterprise fleet-ops maturity.
