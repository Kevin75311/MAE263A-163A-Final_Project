# simulator/simulator.py

import numpy as np
from visualizer import Visualizer

class Simulator:
    """Simulates robot motion along a path represented by 3D edge segments."""

    def __init__(self, visualizer=None):
        self.visualizer = visualizer or Visualizer()

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
        order = self.nearest_neighbor_order(segments)
        self.visualizer.animate_segments(segments, order, pause_time)
