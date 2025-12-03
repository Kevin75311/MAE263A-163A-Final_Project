#!/usr/bin/env python3
import time
import numpy as np
import matplotlib.pyplot as plt
from robot import Robot  # Your custom robot class

# ===============================================================
# Configuration
# ===============================================================
BASE_ANGLES = [178, 181, 170, 180, 180]   # Home position [j1, j2, j3, j4, j5]
JOINT_LIMITS = (-170, 170)                # Safe joint limits for j3 & j5
LINK_LENGTH = 1.0                         # For visualization (scaled)
DT = 0.02                                 # Delay between commands (seconds)


# ===============================================================
# Pattern Generators (j3, j5 offsets in DEGREES)
# ===============================================================
def pattern_circle(N: int = 500, amp: float = 60.0) -> np.ndarray:
    """Simple circle centered at base position."""
    t = np.linspace(0, 2 * np.pi, N, endpoint=True)
    j3 = amp * np.sin(t)
    j5 = amp * np.cos(t)
    return np.column_stack((j3, j5))


def pattern_flower(N: int = 600, amp: float = 60.0, petals: int = 6) -> np.ndarray:
    """Rose/rhodonea curve."""
    t = np.linspace(0, 2 * np.pi, N, endpoint=True)
    r = amp * np.cos(petals * t)
    j3 = r * np.sin(t)
    j5 = r * np.cos(t)
    return np.column_stack((j3, j5))


def pattern_heart(N: int = 600, amp: float = 70.0) -> np.ndarray:
    """Classic parametric heart – scaled and centered."""
    t = np.linspace(0, 2 * np.pi, N, endpoint=True)
    x = 16 * np.sin(t)**3
    y = 13 * np.cos(t) - 5 * np.cos(2*t) - 2 * np.cos(3*t) - np.cos(4*t)
    scale = amp / 18.0
    return np.column_stack((x * scale, y * scale))


def pattern_lemniscate(N: int = 600, amp: float = 80.0) -> np.ndarray:
    """Figure-8 infinity symbol (proper lemniscate of Bernoulli)."""
    t = np.linspace(0, 2 * np.pi, N, endpoint=True)
    a = amp / np.sqrt(2)
    denom = np.sin(2*t)**2 + 1
    r = a * np.sqrt(2) * np.cos(2*t) / denom
    j3 = r * np.cos(t)
    j5 = r * np.sin(t)
    return np.column_stack((j3, j5))


def pattern_star(N: int = 1000, amp: float = 70.0, points: int = 5) -> np.ndarray:
    """Smooth 5-pointed star using sinusoidal modulation."""
    t = np.linspace(0, 2 * np.pi, N, endpoint=False)
    inner_ratio = 0.4
    r = amp * (1 + inner_ratio * np.sin(points * t))
    j3 = r * np.sin(t)
    j5 = r * np.cos(t)
    return np.column_stack((j3, j5))


def pattern_square(N: int = 400, amp: float = 70.0) -> np.ndarray:
    """Clean axis-aligned square with smooth corners (optional)."""
    side = N // 4
    pts = []
    half = amp
    corners = [
        ( half,  half), ( half, -half),
        (-half, -half), (-half,  half),
        ( half,  half)   # close the loop
    ]
    for i in range(4):
        x0, y0 = corners[i]
        x1, y1 = corners[i+1]
        for k in range(side):
            alpha = k / side
            pts.append([x0 + alpha*(x1-x0), y0 + alpha*(y1-y0)])
    return np.array(pts[:-1])  # remove duplicate last point


# ===============================================================
# Assemble full 5-joint trajectory
# ===============================================================
def assemble_motion(pattern: np.ndarray) -> list[list[float]]:
    """Convert (j3_offset, j5_offset) pattern → full 5-joint angles."""
    sequence = []
    j3_base, j5_base = BASE_ANGLES[2], BASE_ANGLES[4]
    for dj3, dj5 in pattern:
        target = BASE_ANGLES.copy()
        target[2] = np.clip(j3_base + dj3, *JOINT_LIMITS)
        target[4] = np.clip(j5_base + dj5, *JOINT_LIMITS)
        sequence.append(target)
    return sequence


# ===============================================================
# Live RR Simulation (no artificial rotation!)
# ===============================================================
def animate_rr(pattern: np.ndarray, L: float = LINK_LENGTH) -> None:
    j3_offsets = np.deg2rad(pattern[:, 0])
    j5_offsets = np.deg2rad(pattern[:, 1])

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-2.2*L, 2.2*L)
    ax.set_ylim(-2.2*L, 2.2*L)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title("RR Robot Simulation (Real Orientation)")

    arm_line, = ax.plot([], [], 'o-', lw=3, markersize=8)
    trace_line, = ax.plot([], [], 'r-', lw=2)

    trace_x, trace_y = [], []

    for i in range(len(j3_offsets)):
        th1 = j3_offsets[i]          # absolute angle of first joint
        th2 = th1 + j5_offsets[i]    # second joint relative → absolute

        # Forward kinematics
        x1 = L * np.cos(th1)
        y1 = L * np.sin(th1)
        x2 = x1 + L * np.cos(th2)
        y2 = y1 + L * np.sin(th2)

        trace_x.append(x2)
        trace_y.append(y2)

        arm_line.set_data([0, x1, x2], [0, y1, y2])
        trace_line.set_data(trace_x, trace_y)

        plt.pause(0.005)

    plt.ioff()
    plt.show()


# ===============================================================
# Execute on real robot
# ===============================================================
def execute_on_robot(plotter: Robot, sequence: list[list[float]]) -> None:
    for target in sequence:
        plotter.set_angles_with_feedback(target)
        time.sleep(DT)
    print("Pattern finished!")


# ===============================================================
# Main Menu
# ===============================================================
def main() -> None:
    patterns = {
        "1": ("Circle", pattern_circle),
        "2": ("Flower (6 petals)", pattern_flower),
        "3": ("Heart", pattern_heart),
        "4": ("Lemniscate (∞)", pattern_lemniscate),
        "5": ("Star (5-point)", pattern_star),
        "6": ("Square", pattern_square),
    }

    print("\n" + "="*40)
    print("   RR Robot Pattern Drawer")
    print("="*40)
    for k, (name, _) in patterns.items():
        print(f"{k}. {name}")
    print("="*40)

    while True:
        choice = input("\nChoose pattern (1-6): ").strip()
        if choice not in patterns:
            print("Invalid choice!")
            continue

        name, func = patterns[choice]
        print(f"\nGenerating {name}...")
        pat = func()

        print("Running simulation...")
        animate_rr(pat)

        motion = assemble_motion(pat)
        run = input("\nSend to robot? (y/n): ").strip().lower()
        if run != "y":
            print("Cancelled.")
            continue

        try:
            plotter = Robot("COM5", 57600, 5)
            time.sleep(2)
            plotter.set_gains(None)
            plotter.torque_enable()
            print("Executing on robot...")
            execute_on_robot(plotter, motion)
        except Exception as e:
            print(f"Robot error: {e}")
        finally:
            if 'plotter' in locals():
                plotter.torque_disable()

        again = input("\nAnother pattern? (y/n): ").strip().lower()
        if again != "y":
            break


if __name__ == "__main__":
    main()