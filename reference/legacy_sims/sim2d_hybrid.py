import numpy as np
import heapq
import matplotlib.pyplot as plt
import matplotlib.animation as animation

GRID_SIZE = 5
CELL = 2.0
ROBOT_RADIUS = 0.3
SPEED = 1.0
DT = 0.05
REROUTE_WAIT_THRESHOLD = 1.5
RESERVATION_BUFFER = 0.3


def node_pos(n):
    x, y = n
    return np.array([x * CELL, y * CELL])


def neighbors(n):
    x, y = n
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
            yield (nx, ny)


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(start, goal, blocked_edges=frozenset()):
    open_set = [(heuristic(start, goal), 0, start, [start])]
    visited = {}
    while open_set:
        f, g, current, path = heapq.heappop(open_set)
        if current == goal:
            return path
        if current in visited and visited[current] <= g:
            continue
        visited[current] = g
        for nxt in neighbors(current):
            edge = (current, nxt)
            if edge in blocked_edges or (nxt, current) in blocked_edges:
                continue
            ng = g + 1
            heapq.heappush(open_set, (ng + heuristic(nxt, goal), ng, nxt, path + [nxt]))
    return None


class Robot:
    def __init__(self, rid, start_node, goal_node, priority, color):
        self.id = rid
        self.priority = priority
        self.color = color
        self.path = astar(start_node, goal_node)
        self.path_idx = 0
        self.pos = node_pos(start_node).astype(float)
        self.state = "MOVE"
        self.wait_time = 0.0
        self.edge_progress = 0.0
        self.reroute_count = 0
        self.trail = [self.pos.copy()]
        self.arrived = False

    def current_edge(self):
        if self.path is None or self.path_idx >= len(self.path) - 1:
            return None
        return (self.path[self.path_idx], self.path[self.path_idx + 1])

    def edge_time_window(self, t_now):
        edge_duration = CELL / SPEED
        t_start = t_now - self.edge_progress * edge_duration
        t_end = t_start + edge_duration
        return (t_start - RESERVATION_BUFFER, t_end + RESERVATION_BUFFER)


def windows_conflict(window_a, window_b):
    return not (window_a[1] < window_b[0] or window_b[1] < window_a[0])


def step_robots(robots, t_now):
    for robot in robots:
        if robot.arrived or robot.path is None:
            continue

        edge = robot.current_edge()
        if edge is None:
            robot.arrived = True
            continue

        my_nodes = set(edge)
        my_window = robot.edge_time_window(t_now)

        # FIX: check conflicts at the NODE level, not just exact-edge match.
        # Two robots on *different* edges that share an endpoint node with
        # overlapping time windows must still be treated as a conflict --
        # this catches robots converging on the same intersection from
        # opposite directions (the bug seen in the screenshot), as well as
        # classic head-on edge swaps.
        blocked_by = None
        for other in robots:
            if other.id == robot.id or other.arrived or other.path is None:
                continue
            other_edge = other.current_edge()
            if other_edge is None:
                continue
            other_nodes = set(other_edge)
            if not (my_nodes & other_nodes):
                continue  # no shared node -> no possible conflict
            other_window = other.edge_time_window(t_now)
            if windows_conflict(my_window, other_window):
                blocked_by = other
                break

        if blocked_by is not None:
            if blocked_by.priority >= robot.priority:
                robot.state = "WAIT"
                robot.wait_time += DT
                if robot.wait_time > REROUTE_WAIT_THRESHOLD:
                    current_node = robot.path[robot.path_idx]
                    goal_node = robot.path[-1]
                    new_path = astar(current_node, goal_node, blocked_edges=frozenset({edge}))
                    if new_path:
                        robot.path = new_path
                        robot.path_idx = 0
                        robot.edge_progress = 0.0
                        robot.wait_time = 0.0
                        robot.reroute_count += 1
                continue
            # else: we have strictly higher priority, proceed

        robot.state = "MOVE"
        robot.wait_time = 0.0

        robot.edge_progress += (SPEED * DT) / CELL
        n1, n2 = edge
        p1, p2 = node_pos(n1), node_pos(n2)
        robot.pos = p1 + (p2 - p1) * min(1.0, robot.edge_progress)
        robot.trail.append(robot.pos.copy())

        if robot.edge_progress >= 1.0:
            robot.path_idx += 1
            robot.edge_progress = 0.0
            if robot.path_idx >= len(robot.path) - 1:
                robot.arrived = True


def make_robots():
    configs = [
        ((0, 0), (4, 4), 0.9, 'tab:red'),
        ((4, 0), (0, 4), 0.6, 'tab:blue'),
        ((0, 4), (4, 0), 0.3, 'tab:green'),
    ]
    return [Robot(i, s, g, p, c) for i, (s, g, p, c) in enumerate(configs)]


robots = make_robots()

fig, ax = plt.subplots(figsize=(7, 7))
ax.set_xlim(-1, GRID_SIZE * CELL)
ax.set_ylim(-1, GRID_SIZE * CELL)
ax.set_aspect('equal')

for x in range(GRID_SIZE):
    for y in range(GRID_SIZE):
        if x < GRID_SIZE - 1:
            p1, p2 = node_pos((x, y)), node_pos((x + 1, y))
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], '-', color='lightgray', zorder=1, linewidth=0.8)
        if y < GRID_SIZE - 1:
            p1, p2 = node_pos((x, y)), node_pos((x, y + 1))
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], '-', color='lightgray', zorder=1, linewidth=0.8)

for r in robots:
    goal_pos = node_pos(r.path[-1])
    ax.plot(goal_pos[0], goal_pos[1], 'x', color=r.color, markersize=10, zorder=2)

circles = [plt.Circle(r.pos, ROBOT_RADIUS, color=r.color, zorder=3) for r in robots]
for c in circles:
    ax.add_patch(c)
labels = [ax.text(r.pos[0], r.pos[1], f"{r.id}\np{r.priority}", fontsize=7, ha='center', color='white', zorder=4) for r in robots]
state_labels = [ax.text(r.pos[0], r.pos[1] + 0.5, "", fontsize=7, ha='center', zorder=4) for r in robots]

sim_time = [0.0]
collision_count = [0]


def update(frame):
    sim_time[0] += DT
    step_robots(robots, sim_time[0])

    for i in range(len(robots)):
        for j in range(i + 1, len(robots)):
            d = np.linalg.norm(robots[i].pos - robots[j].pos)
            if d < ROBOT_RADIUS * 2:
                collision_count[0] += 1

    for r, c, lbl, st in zip(robots, circles, labels, state_labels):
        c.center = r.pos
        lbl.set_position(r.pos)
        st.set_position((r.pos[0], r.pos[1] + 0.5))
        if r.arrived:
            st.set_text("DONE")
            st.set_color('gray')
        elif r.state == "WAIT":
            st.set_text("WAIT")
            st.set_color('red')
        else:
            st.set_text("")

    total_reroutes = sum(r.reroute_count for r in robots)
    n_waiting = sum(1 for r in robots if r.state == "WAIT" and not r.arrived)
    ax.set_title(f"Hybrid Grid/Graph AMR | t={sim_time[0]:.1f}s | "
                 f"collisions={collision_count[0]} | waiting={n_waiting} | reroutes={total_reroutes}")

    return circles + labels + state_labels


ani = animation.FuncAnimation(fig, update, frames=1000, interval=DT * 1000, blit=False)
plt.show()
