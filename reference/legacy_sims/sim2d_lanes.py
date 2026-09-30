import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

MAX_SPEED = 1.2
DT = 0.05
ROBOT_RADIUS = 0.3
STOP_DISTANCE = 0.9
WORLD_SIZE = 10.0

STATE_CRUISE = "CRUISE"
STATE_WAIT = "WAIT"


class LaneRobot:
    def __init__(self, rid, start, end, priority, speed=MAX_SPEED):
        self.id = rid
        self.start = np.array(start, dtype=float)
        self.end = np.array(end, dtype=float)
        self.pos = self.start.copy()
        direction = self.end - self.start
        self.length = np.linalg.norm(direction)
        self.dir = direction / self.length
        self.priority = priority
        self.speed = speed
        self.progress = 0.0
        self.state = STATE_CRUISE
        self.trail = [self.pos.copy()]

    def advance(self, dist):
        self.progress = min(self.length, self.progress + dist)
        self.pos = self.start + self.dir * self.progress
        self.trail.append(self.pos.copy())

    def done(self):
        return self.progress >= self.length


def find_conflict_points(robots):
    conflicts = []
    for i in range(len(robots)):
        for j in range(i + 1, len(robots)):
            a, b = robots[i], robots[j]
            p, r = a.start, a.dir * a.length
            q, s = b.start, b.dir * b.length
            rxs = r[0] * s[1] - r[1] * s[0]
            if abs(rxs) < 1e-9:
                continue
            t = ((q - p)[0] * s[1] - (q - p)[1] * s[0]) / rxs
            u = ((q - p)[0] * r[1] - (q - p)[1] * r[0]) / rxs
            if 0 <= t <= 1 and 0 <= u <= 1:
                point = p + t * r
                conflicts.append((a, b, point))
    return conflicts


def dist_along_path_to_point(robot, point):
    return np.dot(point - robot.start, robot.dir)


def update_robot_states(robots, conflicts, reservations):
    for robot in robots:
        if robot.done():
            continue

        blocking_conflict = None
        for (a, b, point) in conflicts:
            if robot not in (a, b):
                continue
            other = b if robot is a else a
            conflict_id = (min(a.id, b.id), max(a.id, b.id))

            dist_to_point_self = dist_along_path_to_point(robot, point) - robot.progress
            dist_to_point_other = dist_along_path_to_point(other, point) - other.progress

            if dist_to_point_self < -0.05:
                continue

            # FIX: if the other robot has already passed this point, it is
            # no longer a threat -- skip it so `self` can proceed. Without
            # this, a stopped robot kept re-yielding to a robot that had
            # already crossed and moved on, freezing forever.
            if dist_to_point_other < -0.05:
                continue

            reserved_by = reservations.get(conflict_id)

            if reserved_by is not None and reserved_by != robot.id:
                if dist_to_point_self < STOP_DISTANCE:
                    blocking_conflict = conflict_id
                    break
                continue

            if reserved_by == robot.id:
                continue

            both_near = (dist_to_point_self < STOP_DISTANCE * 1.5 and
                         dist_to_point_other < STOP_DISTANCE * 1.5)
            if both_near:
                other_has_priority = other.priority > robot.priority
                if other_has_priority and dist_to_point_self < STOP_DISTANCE:
                    blocking_conflict = conflict_id
                    break
                elif not other_has_priority and dist_to_point_self < 0.15:
                    reservations[conflict_id] = robot.id
            elif dist_to_point_self < 0.15:
                reservations[conflict_id] = robot.id

        robot.state = STATE_WAIT if blocking_conflict else STATE_CRUISE

    for conflict_id, owner_id in list(reservations.items()):
        owner = next((r for r in robots if r.id == owner_id), None)
        match = next((c for c in conflicts
                      if (min(c[0].id, c[1].id), max(c[0].id, c[1].id)) == conflict_id), None)
        if owner is None or match is None:
            del reservations[conflict_id]
            continue
        point = match[2]
        dist_past = owner.progress - dist_along_path_to_point(owner, point)
        if owner.done() or dist_past > ROBOT_RADIUS * 2.2:
            del reservations[conflict_id]


def make_robots():
    configs = [
        ((0.5, 2.0), (9.5, 2.0), 0.9),
        ((0.5, 5.0), (9.5, 5.0), 0.5),
        ((0.5, 8.0), (9.5, 8.0), 0.3),
        ((2.0, 0.5), (2.0, 9.5), 0.7),
        ((5.0, 0.5), (5.0, 9.5), 0.6),
        ((8.0, 0.5), (8.0, 9.5), 0.4),
    ]
    return [LaneRobot(i, s, e, p) for i, (s, e, p) in enumerate(configs)]


robots = make_robots()
reservations = {}

fig, ax = plt.subplots(figsize=(7.5, 7.5))
ax.set_xlim(0, WORLD_SIZE)
ax.set_ylim(0, WORLD_SIZE)
ax.set_aspect('equal')

for r in robots:
    ax.plot([r.start[0], r.end[0]], [r.start[1], r.end[1]], '--', color='lightgray', linewidth=1, zorder=1)

colors = plt.cm.tab10(np.linspace(0, 1, len(robots)))
circles = [plt.Circle(r.pos, ROBOT_RADIUS, color=colors[i], zorder=3) for i, r in enumerate(robots)]
for c in circles:
    ax.add_patch(c)
labels = [ax.text(r.pos[0], r.pos[1], f"{r.id}", fontsize=8, ha='center', va='center', zorder=4, color='white') for r in robots]
state_labels = [ax.text(r.pos[0], r.pos[1] + 0.5, "", fontsize=7, ha='center', zorder=4) for r in robots]

sim_time = [0.0]
collision_count = [0]


def update(frame):
    sim_time[0] += DT
    conflicts = find_conflict_points(robots)
    update_robot_states(robots, conflicts, reservations)

    for r in robots:
        if r.done():
            continue
        if r.state == STATE_WAIT:
            continue
        r.advance(r.speed * DT)

    for i in range(len(robots)):
        for j in range(i + 1, len(robots)):
            d = np.linalg.norm(robots[i].pos - robots[j].pos)
            if d < ROBOT_RADIUS * 2:
                collision_count[0] += 1

    for r, c, lbl, st in zip(robots, circles, labels, state_labels):
        c.center = r.pos
        lbl.set_position(r.pos)
        st.set_position((r.pos[0], r.pos[1] + 0.5))
        st.set_text("STOP" if r.state == STATE_WAIT else "")
        st.set_color('red' if r.state == STATE_WAIT else 'black')

    n_waiting = sum(1 for r in robots if r.state == STATE_WAIT)
    ax.set_title(f"Lane-based AMR | t={sim_time[0]:.1f}s | collisions={collision_count[0]} | waiting={n_waiting}")

    return circles + labels + state_labels


ani = animation.FuncAnimation(fig, update, frames=800, interval=DT * 1000, blit=False)
plt.show()
