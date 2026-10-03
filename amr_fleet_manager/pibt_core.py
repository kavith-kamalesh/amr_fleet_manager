"""
pibt_core.py -- pure-Python, dependency-free building blocks shared by the
benchmark (fleet_pibt_bench.py), the message protocol (fleet_protocol.py) and
the ROS 2 node (pibt_fleet_node.py). Keeping ONE implementation means the
benchmark numbers and the robot behaviour cannot silently drift apart.

  grid_graph : 4-connected grid
  bfs_dist   : distance-to-goal map, optionally around blocked cells
  pibt_step  : one tick of Priority Inheritance with Backtracking
               (Okumura et al., 2019)
"""
from collections import deque

DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1)]


def grid_graph(w, h=None, blocked=frozenset()):
    h = w if h is None else h
    return {
        (x, y): [(x + dx, y + dy) for dx, dy in DIRS
                 if 0 <= x + dx < w and 0 <= y + dy < h
                 and (x + dx, y + dy) not in blocked]
        for x in range(w) for y in range(h) if (x, y) not in blocked
    }


def bfs_dist(nbrs, goal, blocked=frozenset(), fill=None):
    dist = {goal: 0}
    q = deque([goal])
    while q:
        v = q.popleft()
        for u in nbrs[v]:
            if u not in dist and u not in blocked:
                dist[u] = dist[v] + 1
                q.append(u)
    if fill is not None:                      # unreachable cells get a big cost
        for v in nbrs:
            dist.setdefault(v, fill)
    return dist


def pibt_step(nbrs, cur, dist, eff, active, frozen=()):
    """Return {robot: next_cell} for every robot in `active`.

    cur[i]      current cell          dist[i][cell]  cost-to-goal map
    eff[i]      priority (higher wins) frozen        robots that must stay put
    """
    at = {cur[i]: i for i in active}
    nxt, taken = {}, {}
    for i in frozen:
        if i in active:
            nxt[i] = cur[i]
            taken[cur[i]] = i

    def push(i, parent):
        c = cur[i]
        cands = sorted(nbrs[c] + [c], key=lambda u: (dist[i][u], u != c, u))
        for u in cands:
            if u in taken:
                continue
            if parent is not None and u == cur[parent]:
                continue                      # would swap with the requester
            taken[u] = i
            nxt[i] = u
            j = at.get(u)
            if j is not None and j != i and j not in nxt:
                if not push(j, i):
                    del nxt[i]                # j failed and now holds u
                    continue
            return True
        nxt[i] = c
        taken[c] = i
        return False

    for i in sorted(active, key=lambda k: (-eff[k], k)):
        if i not in nxt:
            push(i, None)
    return nxt
