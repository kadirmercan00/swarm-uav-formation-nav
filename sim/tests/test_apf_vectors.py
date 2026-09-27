"""Unit tests for Swarm UAV Artificial Potential Field (APF) vector kinematics."""
import unittest
import math
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from swarm_visualizer import Vector2D, calculate_apf_force

class TestSwarmAPF(unittest.TestCase):
    def test_vector_operations(self):
        v1 = Vector2D(3, 4)
        self.assertEqual(v1.magnitude(), 5.0)

        norm = v1.normalize()
        self.assertAlmostEqual(norm.magnitude(), 1.0)
        self.assertAlmostEqual(norm.x, 0.6)
        self.assertAlmostEqual(norm.y, 0.8)

    def test_attractive_force_direction(self):
        drone_pos = Vector2D(0, 0)
        target_pos = Vector2D(10, 0)
        obstacles = []

        force = calculate_apf_force(drone_pos, target_pos, obstacles, k_att=1.0)
        self.assertGreater(force.x, 0) # Pulls right
        self.assertEqual(force.y, 0)

    def test_repulsive_force_pushes_away(self):
        drone_pos = Vector2D(10, 0)
        target_pos = Vector2D(20, 0)
        # Place obstacle at x=12 (2 meters ahead)
        obstacles = [(Vector2D(12, 0), 0.5)]

        force = calculate_apf_force(drone_pos, target_pos, obstacles, k_att=1.0, k_rep=50.0)
        # Repulsive force from obstacle at x=12 must push in -x direction
        self.assertLess(force.x, 0)

if __name__ == '__main__':
    unittest.main()
