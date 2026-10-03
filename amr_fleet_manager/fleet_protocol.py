"""
fleet_protocol.py -- the per-robot, message-driven coordination logic.
Pure Python (no ROS imports) so it is unit-testable and shared by the lossy
network simulator (protocol_sim.py) and the ROS 2 node (pibt_fleet_node.py).

One TICK = every robot moves at most one grid cell. Per tick:

  round 0   every robot broadcasts its STATE (cell, goal, priority)
  plan      each robot runs the SAME deterministic PIBT step on what it heard
            and learns its own proposed next cell
  round 1.. every robot broadcasts its INTENT (target cell, committed?)
            A robot COMMITS to a move only when
              (a) it has heard a current intent from every peer it planned with,
              (b) no higher-priority / already-committed peer wants the same cell,
              (c) the cell is free, or its occupant has COMMITTED to leave.
            Staying is always committed. A commit is never retracted.
  finalize  anything not committed stays put  -> failure mode is "wait", not "crash"

Peers that stop talking are treated as frozen obstacles for MEMORY_TICKS, and
a robot will not move within SILENT_RADIUS cells of such a peer.
"""
try:
    from pibt_core import bfs_dist, pibt_step
except ImportError:
    from amr_fleet_manager.pibt_core import bfs_dist, pibt_step

BIG = 10 ** 6


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class FleetAgent:
    def __init__(self, rid, nbrs, prio, cell, goal, rounds=6,
                 memory_ticks=None, silent_radius=2, vanish_docked=False,
                 rotations=True):
        self.id, self.nbrs, self.prio = rid, nbrs, prio
        self.cell, self.goal = tuple(cell), tuple(goal) if goal else None
        self.rounds = rounds
        self.memory_ticks = memory_ticks
        self.silent_radius = silent_radius
        self.vanish_docked = vanish_docked
        self.rotations = rotations          # allow >=3-robot rotations (needs consensus)
        self._round = 0
        self.docked = self.goal is not None and self.cell == self.goal
        self.known = {}            # pid -> last seen info
        self.tick = -1
        self.heard, self.intents, self.expected = set(), {}, set()
        self.target, self.committed = self.cell, True
        self.ready = self.armed = False
        self._dist_cache = {}
        self._future = []          # messages that arrived before our tick began

    # ------------------------------------------------------------ messages
    def _msg(self, rnd):
        return {"id": self.id, "tick": self.tick, "round": rnd,
                "cell": list(self.cell),
                "goal": list(self.goal) if self.goal else None,
                "prio": self.prio, "docked": self.docked,
                "target": list(self.target) if rnd > 0 else None,
                "committed": bool(self.committed) if rnd > 0 else False,
                "ready": bool(self.ready) if rnd > 0 else False,
                "armed": bool(self.armed) if rnd > 0 else False}

    def begin_tick(self, tick):
        self.tick = tick
        self.heard, self.intents, self.expected = set(), {}, set()
        self.target, self.committed = self.cell, True
        self.ready = self.armed = False
        self._round = 0
        early, self._future = self._future, []
        for m in sorted(early, key=lambda x: x["round"]):
            self.receive(m)                     # replay what arrived early
        return self._msg(0)

    def round_msg(self, rnd):
        self._round = rnd
        return self._msg(rnd)

    def seed_peer(self, pid, cell, goal=None, prio=0.0):
        """Fleet roster: every robot's id and start cell are known from the
        launch config, so a robot whose radio is dead from the start is still
        on everyone's map."""
        if pid != self.id:
            self.known[pid] = {"last_tick": -1, "cell": tuple(cell),
                               "goal": tuple(goal) if goal else None,
                               "prio": prio, "docked": False,
                               "maybe": {tuple(cell)}}

    def forget(self, pid):
        self.known.pop(pid, None)

    def receive(self, m):
        if m["id"] == self.id or m["tick"] < self.tick:
            return                              # stale
        if m["tick"] > self.tick:
            self._future.append(m)              # peer's clock is slightly ahead
            return
        pid = m["id"]
        self.known[pid] = {"last_tick": self.tick, "cell": tuple(m["cell"]),
                           "goal": tuple(m["goal"]) if m["goal"] else None,
                           "prio": m["prio"], "docked": m["docked"],
                           "maybe": {tuple(m["cell"])}}
        self.heard.add(pid)
        if m["round"] >= 1 and m["target"] is not None:
            tgt, com = tuple(m["target"]), bool(m["committed"])
            # where could it be at the END of this tick? (matters if it falls silent)
            self.known[pid]["maybe"] = {tgt} if com else {tuple(m["cell"]), tgt}
            rdy, arm = bool(m.get("ready", False)), bool(m.get("armed", False))
            old = self.intents.get(pid)
            if old and old[2] > m["round"]:
                return                                   # stale, out of order
            if old and old[0] == tgt:
                com = com or old[1]                      # commits are monotone
                rdy, arm = rdy or old[3], arm or old[4]  # so are ready / armed
            self.intents[pid] = (tgt, com, m["round"], rdy, arm)

    # ---------------------------------------------------------------- plan
    def _fresh(self, info):
        # None = never forget: a silent robot stays on the map as an obstacle
        # (it is probably stopped); only an explicit hand-off removes it.
        return self.memory_ticks is None or \
            self.tick - info["last_tick"] <= self.memory_ticks

    def plan(self):
        """Run the shared PIBT step on this robot's view; set own proposal."""
        cur, goals, eff, frozen = {self.id: self.cell}, {}, {self.id: self.prio}, set()
        goals[self.id] = self.goal
        for pid, info in self.known.items():
            if not self._fresh(info):
                continue
            if info["docked"] and self.vanish_docked:
                continue
            cur[pid], eff[pid] = info["cell"], info["prio"]
            if pid in self.heard and not info["docked"]:
                goals[pid] = info["goal"]
            else:
                frozen.add(pid)                           # silent or parked
                if pid not in self.heard:                 # silent: could be anywhere
                    ghosts = sorted(info["maybe"] - {info["cell"]})
                    for k, c in enumerate(ghosts):
                        gid = -(pid * 16 + k) - 1         # phantom obstacle id
                        cur[gid], eff[gid] = c, info["prio"]
                        frozen.add(gid)
        if self.docked or self.goal is None:
            frozen.add(self.id)
        obstacles = frozenset(cur[p] for p in frozen)
        dist = {}
        for pid, g in goals.items():
            if pid in frozen or g is None:
                dist[pid] = {}
                continue
            key = (g, obstacles)
            if key not in self._dist_cache:
                self._dist_cache[key] = bfs_dist(self.nbrs, g, obstacles, fill=BIG)
            dist[pid] = self._dist_cache[key]
        for pid in frozen:
            dist.setdefault(pid, {})
        for pid in cur:                                   # robots w/o goal: stay
            if pid not in frozen and goals.get(pid) is None:
                frozen.add(pid)
        nxt = pibt_step(self.nbrs, cur, dist, eff, set(cur), frozen)
        self.proposals = nxt
        self.target = nxt.get(self.id, self.cell)
        self.expected = {p for p in self.heard
                         if p in cur and p not in frozen}
        # never move near a peer that went silent (it might still be moving)
        if self.target != self.cell:
            for pid, info in self.known.items():
                if (pid not in self.heard and self._fresh(info)
                        and not (info["docked"] and self.vanish_docked)
                        and min(manhattan(c, self.target)
                                for c in info["maybe"] | {info["cell"]})
                        <= self.silent_radius):
                    self.target = self.cell
                    break
        self.committed = (self.target == self.cell)
        return self.target

    # -------------------------------------------------------------- commit
    def step_commit(self):
        if self.committed:
            return
        t = self.target
        if any(p not in self.intents for p in self.expected):
            return                                        # wait for more info
        me = (self.prio, -self.id)
        for pid, (tg, com, _, _r, _a) in self.intents.items():
            info = self.known[pid]
            if tg == t and (com or (info["prio"], -pid) > me):
                return self._abort()                      # someone else owns it
            if tg == self.cell and info["cell"] == t:
                return self._abort()                      # head-on swap
        for pid, info in self.known.items():              # who is standing there?
            cells = {info["cell"]} if pid in self.heard else info["maybe"] | {info["cell"]}
            if t in cells and self._fresh(info) and not (
                    info["docked"] and self.vanish_docked):
                it = self.intents.get(pid)
                if it and it[1] and it[0] not in (t, info["cell"]):
                    continue                              # committed to leave
                return self._try_cycle()                  # blocked: rotation?
        self.committed = True

    def _find_cycle(self):
        """Members of a rotation me -> o1 -> o2 -> ... -> me, or None."""
        owner = {info["cell"]: pid for pid, info in self.known.items()
                 if self._fresh(info) and not (info["docked"] and self.vanish_docked)}
        members, t = [self.id], self.target
        while True:
            if t == self.cell:
                return members
            pid = owner.get(t)
            if pid is None or pid in members:
                return None
            it = self.intents.get(pid)
            if it is None or it[0] == self.known[pid]["cell"]:
                return None                               # stays: really blocked
            members.append(pid)
            t = it[0]

    def _try_cycle(self):
        """A rotation of >= 3 robots can only start together, so the members run
        a small agreement:  ready  (I see the cycle)  ->  armed  (everyone is
        ready)  ->  commit (everyone is armed). Over a lossy radio this can be
        made rare-to-fail but never impossible, hence: nobody commits in the
        last round (so the commit is re-broadcast at least once), and a robot
        that sees a member already committed joins immediately."""
        if not self.rotations:
            return
        cyc = self._find_cycle()
        if not cyc or len(cyc) < 3:
            return
        targets = {self.target} | {self.intents[p][0] for p in cyc[1:]}
        if any(pid not in cyc and it[0] in targets
               for pid, it in self.intents.items()):
            return
        others = cyc[1:]
        if any(self.intents[p][1] for p in others):       # a member is already moving
            self.committed = True
            return
        self.ready = True
        if all(self.intents[p][3] for p in others):
            self.armed = True
        if (self.armed and all(self.intents[p][4] for p in others)
                and self._round <= self.rounds - 1):
            self.committed = True

    def _abort(self):
        self.target, self.committed = self.cell, True

    def finalize(self):
        if not self.committed:
            self.target, self.committed = self.cell, True
        return self.target

    def apply(self):
        self.cell = self.target
        if self.goal is not None and self.cell == self.goal:
            self.docked = True
