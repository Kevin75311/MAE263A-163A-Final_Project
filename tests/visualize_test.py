import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # registers 3D projection

def plot_3d_path(x, y, z, title="3D Path Visualization"):
    """
    Plots a 3D path given X, Y, Z coordinates.
    """
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot3D(x, y, z, 'b-', linewidth=2)
    ax.scatter3D(x[0], y[0], z[0], color='green', s=60, label='Start')
    ax.scatter3D(x[-1], y[-1], z[-1], color='red', s=60, label='End')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)
    ax.legend()
    ax.grid(True)
    plt.show()

if __name__ == "__main__":
    # Example: Generate a spiral path
    t = np.linspace(0, 4*np.pi, 200)
    x = np.cos(t)
    y = np.sin(t)
    z = np.linspace(0, 2, 200)

    plot_3d_path(x, y, z, title="Helical (Spiral) 3D Path")
