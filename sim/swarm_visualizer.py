#!/usr/bin/env python3
"""
Autonomous Swarm UAV Formation Simulator & APF Vector Visualizer
Simulates 1 Leader + 2 Follower drones navigating in V-formation
with Artificial Potential Field (APF) obstacle avoidance.
"""

import math
import time
import argparse

class Vector2D:
    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vector2D(self.x * scalar, self.y * scalar)

    def magnitude(self):
        return math.hypot(self.x, self.y)

    def normalize(self):
        mag = self.magnitude()
        if mag == 0:
            return Vector2D(0, 0)
        return Vector2D(self.x / mag, self.y / mag)

class UAV:
    def __init__(self, uav_id, x, y, role="follower"):
        self.id = uav_id
        self.pos = Vector2D(x, y)
        self.vel = Vector2D(0, 0)
        self.role = role
        self.max_speed = 3.0 # m/s

    def update(self, force, dt=0.1):
        self.vel = force
        if self.vel.magnitude() > self.max_speed:
            self.vel = self.vel.normalize() * self.max_speed
        self.pos = self.pos + (self.vel * dt)

def calculate_apf_force(drone_pos, target_pos, obstacles, k_att=1.2, k_rep=45.0, d_influence=8.0):
    """
    Computes resultant steering force using Artificial Potential Field (APF):
    F_total = F_attractive + sum(F_repulsive)
    """
    # 1. Attractive force towards desired target formation position
    diff = target_pos - drone_pos
    dist_to_target = diff.magnitude()
    f_att = diff.normalize() * (k_att * dist_to_target)

    # 2. Repulsive force from nearby obstacles
    f_rep = Vector2D(0, 0)
    for obs_pos, obs_radius in obstacles:
        to_drone = drone_pos - obs_pos
        dist_to_obs = to_drone.magnitude() - obs_radius

        if 0 < dist_to_obs < d_influence:
            rep_mag = k_rep * ((1.0 / dist_to_obs) - (1.0 / d_influence)) * (1.0 / (dist_to_obs ** 2))
            f_rep = f_rep + (to_drone.normalize() * rep_mag)

    return f_att + f_rep

def run_simulation(steps=100, headless=True):
    print("=" * 65)
    print("  SWARM UAV V-FORMATION & APF COLLISION AVOIDANCE SIMULATOR")
    print("=" * 65)
    print("  Leader   : Drone 0 (Waypoint Navigator)")
    print("  Follower : Drone 1 (Port Wing Offset: -4m, -3m)")
    print("  Follower : Drone 2 (Starboard Wing Offset: -4m, +3m)")
    print("=" * 65 + "\n")

    # Initial drone positions
    leader = UAV(0, x=0.0, y=0.0, role="leader")
    follower1 = UAV(1, x=-5.0, y=-4.0, role="follower")
    follower2 = UAV(2, x=-5.0, y=+4.0, role="follower")

    # Virtual Obstacle directly along flight corridor
    obstacles = [
        (Vector2D(35.0, 0.5), 2.5),  # Cylinder obstacle at (35, 0.5) with radius 2.5m
    ]

    leader_waypoints = [Vector2D(70.0, 0.0)]
    current_wp = leader_waypoints[0]

    dt = 0.1
    for step in range(steps):
        # Update Leader towards waypoint with obstacle avoidance
        f_leader = calculate_apf_force(leader.pos, current_wp, obstacles, k_att=0.8, k_rep=60.0)
        leader.update(f_leader, dt)

        # Compute desired formation slots relative to Leader position
        # V-Formation offsets: (-4m forward/back, +/- 3m lateral)
        desired_slot1 = leader.pos + Vector2D(-4.0, -3.0)
        desired_slot2 = leader.pos + Vector2D(-4.0, +3.0)

        # Apply APF to followers (Slot attraction + Obstacle repulsion)
        f_fol1 = calculate_apf_force(follower1.pos, desired_slot1, obstacles)
        f_fol2 = calculate_apf_force(follower2.pos, desired_slot2, obstacles)

        follower1.update(f_fol1, dt)
        follower2.update(f_fol2, dt)

        if step % 10 == 0:
            print(f"Step {step:03d} | Leader: ({leader.pos.x:5.1f}, {leader.pos.y:5.1f}) | "
                  f"F1: ({follower1.pos.x:5.1f}, {follower1.pos.y:5.1f}) | "
                  f"F2: ({follower2.pos.x:5.1f}, {follower2.pos.y:5.1f})")

        if (current_wp - leader.pos).magnitude() < 1.0:
            print(f"\n[+] Waypoint achieved at step {step}! Swarm V-formation preserved.")
            break

        time.sleep(0.02)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Swarm UAV Formation Simulation")
    parser.add_argument("--steps", type=int, default=120, help="Simulation steps")
    args = parser.parse_args()

    run_simulation(steps=args.steps)
