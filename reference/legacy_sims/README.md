# Legacy simulations and reference code (not in the deployed path)

Kept for comparison and history. None of this is installed by setup.py or used by the launch file.

- sim2d.py: force-based avoidance demo (matplotlib)
- sim2d_hybrid.py: early grid/graph reservation demo, superseded by benchmark_stop_and_wait_vs_hybrid.py
- sim2d_lanes.py: lane-based reservation demo
- orca_node_peer_aware.py: ORCA (rvo2) node with peer state and priority weighting. Needs the rvo2 library. Not benchmarked.

Authoritative results: results/ and scripts/reproduce_headline_numbers.sh
