# simulator/simulator.py

import numpy as np
from visualizer import Visualizer
from robot import Robot

class Simulator:
    """Simulates robot motion along a path represented by 3D edge segments."""

    def __init__(self, visualizer=None):
        self.visualizer = visualizer or Visualizer()
        self.robot = Robot(None, None, 5, simulation=True)

    def compute_edge_centers(self, segments):
        """Compute the center point of each edge segment."""
        return np.array([(
            (seg[0][0] + seg[1][0]) / 2,
            (seg[0][1] + seg[1][1]) / 2,
            (seg[0][2] + seg[1][2]) / 2
        ) for seg in segments])

    def nearest_neighbor_order(self, segments):
        """Compute a drawing order using a greedy nearest-neighbor approach."""
        centers = self.compute_edge_centers(segments)
        n = len(segments)
        visited = np.zeros(n, dtype=bool)
        order = [0]
        visited[0] = True

        for _ in range(n - 1):
            dists = np.linalg.norm(centers - centers[order[-1]], axis=1)
            dists[visited] = np.inf
            next_idx = np.argmin(dists)
            visited[next_idx] = True
            order.append(next_idx)
        return order

    def simulate(self, segments, pause_time=0.05):
        """Run the visualization loop simulating path following."""
        orders = self.nearest_neighbor_order(segments)
        self.visualizer.animate_segments(segments, orders, pause_time)

    def simulate_with_robot(self, segments, pause_time=0.05):
        """
        Run the visualization with robot configuration updates.
        """
        orders = self.nearest_neighbor_order(segments)
        print("[Simulator] Computing robot configurations for each segment...")
        theta_0s = []
        angles_sequence = []
        for order in orders:
            start_pt = segments[order][0]
            end_pt = segments[order][1]

            start_angles = self.robot.inverse_kinematics(start_pt, 0)
            end_angles = self.robot.inverse_kinematics(end_pt, 0)

            angles_sequence.append([start_angles, end_angles])
            theta_0s.append(start_angles[0])
            # theta_0s.append(end_angles[0])

        print("[Simulator] Robot configuration computation complete.")
        positions_sequence = []
        for angles_pair in angles_sequence:            
            positions, _ = self.robot.forward_kinematics(angles_pair[0])
            positions_sequence.append(positions)
            # positions, _ = self.robot.forward_kinematics(angles_pair[1])
            # positions_sequence.append(positions)

        print("[Simulator] Starting visualization with robot...")
        self.visualizer.animate_segments_with_robot(segments, orders, positions_sequence, theta_0s, pause_time)