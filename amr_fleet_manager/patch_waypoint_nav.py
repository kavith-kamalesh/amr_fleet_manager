path = "waypoint_nav_node.py"
with open(path, encoding='utf-8') as f:
    content = f.read()

# Change 1: blocked_edges becomes a dict (edge -> block time) with an expiry constant
old1 = """        self.path = None
        self.path_idx = 0
        self.blocked_edges = set()
        self.mutex_state = MUTEX_CLEAR"""
new1 = """        self.path = None
        self.path_idx = 0
        self.blocked_edges = {}  # edge -> time.time() when blocked; expires after BLOCK_EXPIRY_SEC
        self.BLOCK_EXPIRY_SEC = 5.0
        self.mutex_state = MUTEX_CLEAR"""

count1 = content.count(old1)
if count1 != 1:
    raise SystemExit(f"FATAL: change 1 pattern found {count1} times, expected 1 -- aborting, no changes written")

# Change 2: expire stale entries before adding the new one, in the reroute branch
old2 = """        if self.mutex_state == MUTEX_REROUTE:
            current_node = self.path[self.path_idx]
            goal_node = self.path[-1]
            self.blocked_edges.add(edge)
            new_path = nav_graph.astar(current_node, goal_node, frozenset(self.blocked_edges))"""
new2 = """        if self.mutex_state == MUTEX_REROUTE:
            now = time.time()
            self.blocked_edges = {
                e: t for e, t in self.blocked_edges.items()
                if now - t < self.BLOCK_EXPIRY_SEC
            }
            current_node = self.path[self.path_idx]
            goal_node = self.path[-1]
            self.blocked_edges[edge] = now
            new_path = nav_graph.astar(current_node, goal_node, frozenset(self.blocked_edges))"""

count2 = content.count(old2)
if count2 != 1:
    raise SystemExit(f"FATAL: change 2 pattern found {count2} times, expected 1 -- aborting, no changes written")

content = content.replace(old1, new1, 1)
content = content.replace(old2, new2, 1)

with open(path, "w", encoding='utf-8') as f:
    f.write(content)

print("Patched successfully: blocked_edges now expires after 5.0s instead of growing forever.")
