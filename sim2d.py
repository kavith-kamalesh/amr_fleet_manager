import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

NUM_AGENTS = 6
RADIUS = 0.35
MAX_SPEED = 1.0
DT = 0.05
NEIGHBOR_DIST = 3.0
DEADLOCK_SPEED_THRESH = 0.03
WORLD_SIZE = 8.0
MIN_SAFE = RADIUS * 2.3
REACTION_MARGIN = RADIUS * 5.0

rng = np.random.default_rng(42)

class Agent:
    def __init__(self, aid, pos, goal, priority):
        self.id = aid
        self.pos = np.array(pos, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.vel = np.zeros(2)
        self.priority = priority
        self.stalled_since = None
        self.deadlock_time_thresh = 0.6 + 0.15 * aid
        self.trail = [self.pos.copy()]

    def pref_velocity(self):
        to_goal = self.goal - self.pos
        dist = np.linalg.norm(to_goal)
        if dist < 0.05:
            return np.zeros(2)
        direction = to_goal / dist
        speed = min(MAX_SPEED, dist / DT)
        return direction * speed

def avoid(agent, others, t):
    pref = agent.pref_velocity()
    desired = pref.copy()

    for other in others:
        if other.id == agent.id:
            continue
        offset = agent.pos - other.pos
        dist = np.linalg.norm(offset)
        if dist < 1e-6 or dist > REACTION_MARGIN:
            continue
        direction = offset / dist
        relative_vel = agent.vel - other.vel
        closing_speed = -np.dot(relative_vel, direction)
        relative_priority = other.priority - agent.priority
        yield_factor = 0.5 + 0.5 * np.tanh(relative_priority * 2)

        if closing_speed > 0:
            urgency = max(0.0, (REACTION_MARGIN - dist) / REACTION_MARGIN)
            cancel = direction * closing_speed * urgency * (0.8 + 0.6 * yield_factor)
            desired += cancel

        if dist < MIN_SAFE:
            penetration = (MIN_SAFE - dist)
            hard_push = direction * (penetration / MIN_SAFE) * MAX_SPEED * 1.5
            desired += hard_push

    speed = np.linalg.norm(desired)
    if speed > MAX_SPEED:
        desired = desired / speed * MAX_SPEED

    actual_speed = np.linalg.norm(desired)
    wants_to_move = np.linalg.norm(pref) > DEADLOCK_SPEED_THRESH

    if actual_speed < DEADLOCK_SPEED_THRESH and wants_to_move:
        if agent.stalled_since is None:
            agent.stalled_since = t
        elif t - agent.stalled_since > agent.deadlock_time_thresh:
            nudge_sign = 1.0 if (agent.id % 2 == 0) else -1.0
            perp = np.array([-pref[1], pref[0]])
            norm = np.linalg.norm(perp) or 1.0
            desired += (perp / norm) * 0.4 * nudge_sign
    else:
        agent.stalled_since = None

    return desired

def make_agents():
    agents = []
    for i in range(NUM_AGENTS):
        angle = 2 * np.pi * i / NUM_AGENTS
        jitter = rng.uniform(-0.15, 0.15, size=2)
        start = np.array([WORLD_SIZE / 2 + 3.2 * np.cos(angle),
                           WORLD_SIZE / 2 + 3.2 * np.sin(angle)]) + jitter
        goal = np.array([WORLD_SIZE / 2 - 3.2 * np.cos(angle),
                          WORLD_SIZE / 2 - 3.2 * np.sin(angle)])
        priority = rng.uniform(0.0, 1.0)
        agents.append(Agent(i, start, goal, priority))
    return agents

agents = make_agents()

fig, ax = plt.subplots(figsize=(7, 7))
ax.set_xlim(0, WORLD_SIZE)
ax.set_ylim(0, WORLD_SIZE)
ax.set_aspect('equal')

circles = [plt.Circle(a.pos, RADIUS, color=plt.cm.tab10(a.id % 10)) for a in agents]
for c in circles:
    ax.add_patch(c)
goal_markers = [ax.plot(a.goal[0], a.goal[1], 'x', color=plt.cm.tab10(a.id % 10))[0] for a in agents]
trails = [ax.plot([], [], '-', alpha=0.3, color=plt.cm.tab10(a.id % 10))[0] for a in agents]
labels = [ax.text(a.pos[0], a.pos[1], f"{a.id}\np{a.priority:.1f}", fontsize=7, ha='center') for a in agents]

sim_time = [0.0]
collision_count = [0]

def update(frame):
    sim_time[0] += DT
    t = sim_time[0]
    new_vels = [avoid(agent, agents, t) for agent in agents]

    for agent, v in zip(agents, new_vels):
        agent.vel = v
        agent.pos = agent.pos + v * DT
        agent.trail.append(agent.pos.copy())

    for i in range(len(agents)):
        for j in range(i + 1, len(agents)):
            d = np.linalg.norm(agents[i].pos - agents[j].pos)
            if d < RADIUS * 2:
                collision_count[0] += 1

    for agent, circle, label, trail in zip(agents, circles, labels, trails):
        circle.center = agent.pos
        label.set_position(agent.pos + np.array([0, RADIUS + 0.15]))
        pts = np.array(agent.trail[-100:])
        trail.set_data(pts[:, 0], pts[:, 1])

    ax.set_title(f"2D Fleet Avoidance | t={t:.1f}s | collisions={collision_count[0]}")
    return circles + labels + trails

ani = animation.FuncAnimation(fig, update, frames=600, interval=DT * 1000, blit=False)
plt.show()
