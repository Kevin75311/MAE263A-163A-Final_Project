import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection

def compute_edge_centers(segments):
    """Compute the center point of each edge segment."""
    centers = np.array([( (seg[0][0]+seg[1][0])/2,
                          (seg[0][1]+seg[1][1])/2,
                          (seg[0][2]+seg[1][2])/2 ) for seg in segments])
    return centers

def nearest_neighbor_order(segments):
    """Compute a drawing order using a greedy nearest-neighbor approach."""
    centers = compute_edge_centers(segments)
    n = len(segments)
    visited = np.zeros(n, dtype=bool)
    order = []
    
    # Start from the first segment
    idx = 0
    visited[idx] = True
    order.append(idx)

    for _ in range(n-1):
        # Compute distances to unvisited edges
        dists = np.linalg.norm(centers - centers[idx], axis=1)
        dists[visited] = np.inf  # ignore already visited
        next_idx = np.argmin(dists)
        visited[next_idx] = True
        order.append(next_idx)
        idx = next_idx
    return order

def simulate_edge_drawing(segments, pause_time=0.05):
    """Simulate edge drawing in 3D with optimized travel path."""
    # Compute order
    order = nearest_neighbor_order(segments)
    
    fig = plt.figure(figsize=(9,7))
    ax = fig.add_subplot(111, projection='3d')
    
    # Collect all points for scaling
    all_pts = np.vstack([seg for seg in segments])
    x_min, y_min, z_min = np.min(all_pts, axis=0)
    x_max, y_max, z_max = np.max(all_pts, axis=0)
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
    mid_x, mid_y, mid_z = (x_max+x_min)/2, (y_max+y_min)/2, (z_max+z_min)/2
    ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)
    ax.set_box_aspect([1,1,1])
    ax.set_axis_off()

    # Plot drawn edges and travel route
    drawn_segments = []
    travel_points = []
    
    current_pos = np.mean(segments[order[0]], axis=0)  # start at first segment center
    travel_points.append(current_pos)

    for idx in order:
        seg = segments[idx]
        drawn_segments.append(seg)
        # Update travel path: from current_pos to start of this edge
        edge_center = np.mean(seg, axis=0)
        travel_points.append(edge_center)
        current_pos = edge_center

        # Clear and redraw
        ax.cla()
        # Draw edges so far
        lc = Line3DCollection(drawn_segments, colors='b', linewidths=1)
        ax.add_collection3d(lc)
        # Draw travel path
        travel_arr = np.array(travel_points)
        ax.plot(travel_arr[:,0], travel_arr[:,1], travel_arr[:,2], 'r--', linewidth=0.5)
        
        # Keep axes limits fixed
        ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
        ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
        ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)
        ax.set_box_aspect([1,1,1])
        ax.set_axis_off()
        
        plt.pause(pause_time)
    
    plt.show()

if __name__ == "__main__":
    # Example usage: load segments from file (exported from your previous code)
    segments = []
    with open("./data/segments/UCLAMAE_segments.txt") as f:
        for line in f:
            x0, y0, z0, x1, y1, z1 = map(float, line.strip().split(","))
            segments.append([[x0, y0, z0], [x1, y1, z1]])
    
    simulate_edge_drawing(segments, pause_time=0.005)
