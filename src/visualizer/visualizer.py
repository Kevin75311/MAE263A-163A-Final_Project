# simulator/visualizer.py

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from settings import constants
matplotlib.use('TkAgg')

class Visualizer:
    """Handles 3D path visualization and live simulation display."""

    def __init__(self):
        self.fig = None
        self.ax = None

    def _setup_plot(self, segments):
        """Initialize 3D plot with proper scaling."""
        self.fig = plt.figure(figsize=(9, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')

        all_pts = np.vstack([seg for seg in segments])
        x_min, y_min, z_min = np.min(all_pts, axis=0)
        x_max, y_max, z_max = np.max(all_pts, axis=0)
        max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
        mid_x, mid_y, mid_z = (x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2

        self.ax.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
        self.ax.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
        self.ax.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)
        self.ax.set_box_aspect([1, 1, 1])

        return max_range, (mid_x, mid_y, mid_z)

    def animate_segments(self, segments, order, pause_time=0.05):
        """Animate path drawing according to the given segment order."""
        max_range, (mid_x, mid_y, mid_z) = self._setup_plot(segments)

        drawn_segments = []
        travel_points = []
        current_pos = np.mean(segments[order[0]], axis=0)
        travel_points.append(current_pos)

        for idx in order:
            seg = segments[idx]
            drawn_segments.append(seg)
            edge_center = np.mean(seg, axis=0)
            travel_points.append(edge_center)
            current_pos = edge_center

            # Clear and redraw
            self.ax.cla()
            lc = Line3DCollection(drawn_segments, colors='b', linewidths=1)
            self.ax.add_collection3d(lc)
            travel_arr = np.array(travel_points)
            self.ax.plot(travel_arr[:, 0], travel_arr[:, 1], travel_arr[:, 2], 'r--', linewidth=0.5)
            for point in travel_points:
                # label each travel point
                self.ax.text(point[0], point[1], point[2], f'({point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f})', size=6, zorder=1, color='k')

            # # Reset view and limits
            # self.ax.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
            # self.ax.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
            # self.ax.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)
            # self.ax.set_box_aspect([1, 1, 1])
            # self.ax.set_axis_off()

            plt.pause(pause_time)

        plt.show()

    def animate_segments_with_robot(self, segments, orders, positions_sequence, theta_0s, pause_time=0.05):
        max_range, (mid_x, mid_y, mid_z) = self._setup_plot(segments)

        print(len(segments))
        print(len(orders))
        print(len(positions_sequence))
        print(len(theta_0s))

        drawn_segments = []
        travel_points = []
        current_pos = np.mean(segments[orders[0]], axis=0)
        travel_points.append(current_pos)

        for i, order in enumerate(orders):
            seg = segments[order]
            drawn_segments.append(seg)
            edge_center = np.mean(seg, axis=0)
            travel_points.append(edge_center)
            current_pos = edge_center
            positions = positions_sequence[i]

            # Clear and redraw
            self.ax.cla()
            lc = Line3DCollection(drawn_segments, colors='b', linewidths=1)
            self.ax.add_collection3d(lc)
            travel_arr = np.array(travel_points)
            self.ax.plot(travel_arr[:, 0], travel_arr[:, 1], travel_arr[:, 2], 'r--', linewidth=0.5)
            # label axes
            self.ax.set_xlabel('X [m]')
            self.ax.set_ylabel('Y [m]')
            self.ax.set_zlabel('Z [m]')

            self.ax.quiver(*positions[0], *(positions[3] - positions[0]), arrow_length_ratio=0.1, color="red", linestyle="--")
            self.ax.quiver(*positions[1], *positions[8], arrow_length_ratio=0.1, color="red")
            self.ax.quiver(*positions[2], *positions[9], arrow_length_ratio=0.1, color="red")
            self.ax.quiver(*positions[3], *(positions[4] - positions[3]), arrow_length_ratio=0.1, color="blue")
            self.ax.quiver(*positions[4], *(positions[5] - positions[4]), arrow_length_ratio=0.1, color="green")
            self.ax.quiver(*positions[5], *(positions[6] - positions[5]), arrow_length_ratio=0.1, color="orange")
            self.ax.quiver(*positions[6], *(positions[7] - positions[6]), arrow_length_ratio=0.1, color="black")

            self.ax.set_xlim(mid_x - max_range / 2 * 20, mid_x + max_range / 2 * 20)
            self.ax.set_ylim(mid_y - max_range / 2 * 20, mid_y + max_range / 2 * 20)
            self.ax.set_zlim(mid_z - max_range / 2 * 25, mid_z + max_range / 2 * 15)
            self.ax.set_box_aspect([1, 1, 1])
            # self.ax.set_axis_off()

            plt.pause(pause_time)

        plt.show()
