import time
import serial
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from robot import Robot
matplotlib.use('TkAgg')

# ===================================================================
# Robot parameters
# ===================================================================
L1 = 1.0
L2 = 1.0

# Safe center (adjust once and forget about singularities!)
DEFAULT_CENTER_X = 0.0
DEFAULT_CENTER_Y = 0.8

def set_led(ser, led_number, state):
    cmd = f"LED {led_number} {state}\n"
    ser.write(cmd.encode())

def clear_leds(ser):
    for i in range(1, 6):
        set_led(ser, i, "OFF")

def set_all_leds_on(ser):
    for i in range(1, 6):
        set_led(ser, i, "ON")

def ik(y, x):
    cos_th2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_th2 = np.clip(cos_th2, -1.0, 1.0)
    th2 = np.arccos(cos_th2)
    k1 = L1 + L2 * np.cos(th2)
    k2 = L2 * np.sin(th2)
    th1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    return np.rad2deg(th1), np.rad2deg(th1 + th2)

def pattern_circle(N=30, r=0.5, cx=DEFAULT_CENTER_X, cy=DEFAULT_CENTER_Y):
    t = np.linspace(0, 2*np.pi, N)
    return r * np.cos(t) + cx, r * np.sin(t) + cy

def pattern_flower(N=50, r=0.5, petals=6, cx=DEFAULT_CENTER_X, cy=DEFAULT_CENTER_Y):
    t = np.linspace(0, 2*np.pi, N)
    rho = r * (0.5 + 0.5 * np.cos(petals * t))
    return rho * np.cos(t) + cx, rho * np.sin(t) + cy

def pattern_heart(N=40, scale=0.5, cx=DEFAULT_CENTER_X, cy=DEFAULT_CENTER_Y):
    t = np.linspace(0, 2*np.pi, N)
    x = scale * 16 * np.sin(t)**3 / 20
    y = scale * (13*np.cos(t) - 5*np.cos(2*t) - 2*np.cos(3*t) - np.cos(4*t)) / 20 + 0.4
    return x + cx, y + cy

def pattern_spirograph(R=1.0, r=0.2, p=0.6, N=100, cx=DEFAULT_CENTER_X, cy=DEFAULT_CENTER_Y):
    t = np.linspace(0, 30*np.pi, N)
    x = (R - r) * np.cos(t) + p * np.cos((R - r)/r * t)
    y = (R - r) * np.sin(t) - p * np.sin((R - r)/r * t)
    return x*0.6 + cx, y*0.6 + cy

def pattern_lissajous(a=3, b=4, delta=np.pi/2, scale=0.4, N=100, cx=DEFAULT_CENTER_X, cy=DEFAULT_CENTER_Y):
    t = np.linspace(0, 2*np.pi, N)
    x = scale * np.sin(a * t + delta)
    y = scale * np.cos(b * t)
    return x + cx, y + cy

def pattern_hypocycloid(petals=5, scale=0.4, N=80, cx=DEFAULT_CENTER_X, cy=DEFAULT_CENTER_Y):
    t = np.linspace(0, 2*np.pi*petals, N)
    k = petals
    x = scale * (k-1)/k * np.cos(t) + scale/k * np.cos((k-1)*t)
    y = scale * (k-1)/k * np.sin(t) - scale/k * np.sin((k-1)*t)
    return x + cx, y + cy

def pattern_butterfly(N=60, scale=0.1, cx=DEFAULT_CENTER_X, cy=DEFAULT_CENTER_Y):
    t = np.linspace(0, 24*np.pi, N)
    r = scale * (np.exp(np.cos(t)) - 2*np.cos(4*t) + np.sin(t/12)**5)
    return r * np.sin(t) + cx, r * np.cos(t) + cy / 2

def pattern_clover(N=50, knots=4, scale=0.4, cx=DEFAULT_CENTER_X, cy=DEFAULT_CENTER_Y):
    t = np.linspace(0, 2*np.pi, N)
    r = scale * np.sin(knots * t)
    return r * np.cos(t) + cx, r * np.sin(t) + cy

def pattern_spiral_archimedes(N=80, turns=5, scale=0.02, cx=DEFAULT_CENTER_X, cy=DEFAULT_CENTER_Y):
    t = np.linspace(0, turns*2*np.pi, N)
    r = scale * t
    return r * np.cos(t) + cx, r * np.sin(t) + cy

# ===================================================================
# Helper functions for clean lines and arcs
# ===================================================================
def segment(x0, y0, x1, y1, steps=40):
    """Straight line from (x0,y0) → (x1,y1)"""
    t = np.linspace(0, 1, steps)
    x = x0 + t * (x1 - x0)
    y = y0 + t * (y1 - y0)
    return x, y

def arc(cx, cy, r, angle_start_deg, angle_end_deg, steps=60):
    """Circular arc centered at (cx,cy)"""
    angles = np.linspace(np.deg2rad(angle_start_deg), np.deg2rad(angle_end_deg), steps)
    x = cx + r * np.cos(angles)
    y = cy + r * np.sin(angles)
    return x, y

def pattern_ucla(scale=0.9, cx=DEFAULT_CENTER_X, cy=DEFAULT_CENTER_Y):
    """
    Generates a 2D trajectory for the text 'UCLA'.
    Returns an N x 2 numpy array of (x, y) coordinates.
    """
    s = scale * 0.35
    pts = []
    
    # Separation parameters
    letter_width = 0.5
    gap = 0.2
    
    # Cursor to track X position
    cursor_x = -1.5 
    
    # --- Letter U ---
    # Path: Top-Left -> Bottom-Left -> Bottom-Right -> Top-Right
    # Note: We use [NaN, NaN] to break lines if plotting, but for a continuous 
    # robot path, we just extend. The robot will 'drag' between letters.
    u_h = 0.6
    pts.extend(np.linspace([cursor_x, u_h], [cursor_x, 0.0], 20))       # Down
    pts.extend(np.linspace([cursor_x, 0.0], [cursor_x + letter_width, 0.0], 15)) # Bottom
    pts.extend(np.linspace([cursor_x + letter_width, 0.0], [cursor_x + letter_width, u_h], 20)) # Up
    
    cursor_x += letter_width + gap

    # --- Letter C ---
    # Path: Elliptical Arc from Top-Right opening counter-clockwise to Bottom-Right
    # Angles: 45 degrees (pi/4) to 315 degrees (7pi/4)
    theta = np.linspace(np.pi/4, 7*np.pi/4, 40)
    c_center_x = cursor_x + letter_width / 2.0
    c_center_y = 0.3 # Mid-height
    
    # Width radius and height radius
    rx = letter_width / 2.0
    ry = 0.3
    
    c_x = c_center_x + rx * np.cos(theta)
    c_y = c_center_y + ry * np.sin(theta)
    pts.extend(np.column_stack((c_x, c_y)))
    
    cursor_x += letter_width + gap

    # --- Letter L ---
    # Path: Top-Left -> Bottom-Left -> Bottom-Right
    pts.extend(np.linspace([cursor_x, 0.6], [cursor_x, 0.0], 25))
    pts.extend(np.linspace([cursor_x, 0.0], [cursor_x + letter_width, 0.0], 20))
    
    cursor_x += letter_width + gap

    # --- Letter A ---
    # Path: Bottom-Left -> Top-Peak -> Bottom-Right
    pts.extend(np.linspace([cursor_x, 0.0], [cursor_x + letter_width/2.0, 0.6], 25))
    pts.extend(np.linspace([cursor_x + letter_width/2.0, 0.6], [cursor_x + letter_width, 0.0], 25))
    
    # Crossbar (The robot must "drag" from bottom-right leg to the crossbar start)
    # Crossbar goes Left -> Right
    bar_y = 0.25
    # Calculate x positions based on the A slopes to make the bar connect perfectly
    # Left leg x at bar_y: starts at cursor_x, ends at cursor_x+0.25. 
    # Interpolation approx:
    bar_start_x = cursor_x + 0.12 
    bar_end_x = cursor_x + letter_width - 0.12
    
    pts.extend(np.linspace([bar_start_x, bar_y], [bar_end_x, bar_y], 15))

    # --- Compilation ---
    arr = np.vstack(pts) * s
    
    # Apply global offset
    arr[:, 0] += cx
    arr[:, 1] += cy
    
    return arr

def pack(x, y):
    return np.column_stack((x, y))

def cartesian_to_joints(cartesian_pattern):
    j3, j5 = [], []
    for x, y in cartesian_pattern:
        th1, th2 = ik(x, y)
        j3.append(th1); j5.append(th2)
    return np.array(j3), np.array(j5)

def assemble_motion(j3_offsets, j5_offsets, base=None):
    if base is None: base = [178, 181, 170, 180, 180]
    return [[base[0], base[1], np.clip(base[2]-j3,0,360), base[3], np.clip(base[4]+(j5-j3),0,360)]
            for j3, j5 in zip(j3_offsets, j5_offsets)]

def animate_rr(j3_deg, j5_deg):
    fig, ax = plt.subplots(figsize=(8,8))
    ax.set_xlim(-2.5,2.5); ax.set_ylim(-0.5,2.8); ax.set_aspect('equal'); ax.grid(True)
    
    arm, = ax.plot([], [], 'o-', lw=4, color='steelblue', markersize=8)
    trace, = ax.plot([], [], '-', lw=2, color='crimson')
    
    # Text annotation box
    angle_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=12,
                         verticalalignment='top', 
                         bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    tx, ty = [], []
    for i in range(len(j3_deg)):
        # Current J3 (Shoulder)
        val_j3 = j3_deg[i]
        
        # The "angle between linkages" is the difference between the absolute angle of link 2 (J5)
        # and the absolute angle of link 1 (J3). 
        val_elbow = j5_deg[i] - j3_deg[i]
        
        # Geometry math (for plotting the arm)
        th1 = np.deg2rad(val_j3)
        th2 = np.deg2rad(val_elbow)
        
        x1 = L1 * np.cos(th1)
        y1 = L1 * np.sin(th1)
        x2 = x1 + L2 * np.cos(th1 + th2)
        y2 = y1 + L2 * np.sin(th1 + th2)
        
        tx.append(x2); ty.append(y2)
        arm.set_data([0,x1,x2], [0,y1,y2])
        trace.set_data(tx, ty)
        
        # --- Update text to show J3 and the Elbow Angle ---
        angle_text.set_text(f"J3 (Shoulder): {val_j3:.2f}°\nAngle Between: {val_elbow:.2f}°")
        
        plt.pause(0.2)
    plt.show()

def execute_on_robot(plotter, seq):
    print(f"Drawing {len(seq)} points...")
    for target in seq:
        plotter.set_angles_with_feedback(target)
        # time.sleep(1)

# ===================================================================
# MAIN with new menu
# ===================================================================
def main():
    cx, cy = DEFAULT_CENTER_X, DEFAULT_CENTER_Y

    patterns = {
        "1": ("Circle", lambda: pack(*pattern_circle(cx=cx, cy=cy))),
        "2": ("Rose (6 petals)", lambda: pack(*pattern_flower(cx=cx, cy=cy))),
        "3": ("Heart", lambda: pack(*pattern_heart(cx=cx, cy=cy))),
        "4": ("Spirograph", lambda: pack(*pattern_spirograph(cx=cx, cy=cy))),
        "5": ("Lissajous (3:4)", lambda: pack(*pattern_lissajous(cx=cx, cy=cy))),
        "6": ("Hypocycloid (5)", lambda: pack(*pattern_hypocycloid(cx=cx, cy=cy))),
        "7": ("Butterfly Curve", lambda: pack(*pattern_butterfly(cx=cx, cy=cy))),
        "8": ("Four-leaf Clover", lambda: pack(*pattern_clover(cx=cx, cy=cy))),
        "9": ("Archimedean Spiral", lambda: pack(*pattern_spiral_archimedes(cx=cx, cy=cy))),
        "10": ("UCLA", lambda: pattern_ucla(cx=cx, cy=cy)),
    }

    plotter = Robot("COM5", 57600, 2)
    arduino_UNO = serial.Serial("COM7", 9600, timeout=1)
    time.sleep(2)
    # LED test
    print("===============================")
    print("[Main] Connected to Arduino. Starting LED test...")
    for i in range(1, 6):
        clear_leds(arduino_UNO)
        set_led(arduino_UNO, i, "ON")
        time.sleep(1)
    clear_leds(arduino_UNO)
    print("[Main] LED test complete.")
    print("===============================")
    time.sleep(2)

    # homing
    print("===============================")
    print("[Main] Starting robot homing...")
    steps = 6
    current_position = plotter.get_angles()
    homing_position = [178, 181, 170, 180, 180]
    plotter.torque_enable()
    for step in range(1, steps + 1):
        intermediate_position = [
            current_position[i] + (homing_position[i] - current_position[i]) * step / steps
            for i in range(5)
        ]
        plotter.set_angles_with_feedback(intermediate_position)
        time.sleep(0.5)
    print("[Main] Robot homing complete.")
    print("===============================")
    time.sleep(2)

    while True:
        print("\n" + "="*50)
        for k, (name, _) in patterns.items():
            print(f"{k.rjust(2)}. {name}")
        print(" q. Quit")
        choice = input("\nChoose (1-10): ").strip()
        color = input("Enter LED color during drawing (e.g., '1' for LED 1 ON): ").strip()

        if choice == 'q': break
        if choice not in patterns:
            print("Invalid!")
            continue

        name, func = patterns[choice]
        print(f"\nGenerating {name}...")
        cart = func()

        print("Inverse kinematics...")
        j3, j5 = cartesian_to_joints(cart)

        print("Simulation (close plot to continue)...")
        # animate_rr(j3, j5)

        time.sleep(2)
        plotter.set_gains(None)
        plotter.torque_enable()
        motion = assemble_motion(j3, j5)

        steps = 6
        current_position = plotter.get_angles()
        homing_position = motion[0]
        plotter.torque_enable()
        for step in range(1, steps + 1):
            intermediate_position = [
                current_position[i] + (homing_position[i] - current_position[i]) * step / steps
                for i in range(5)
            ]
            plotter.set_angles_with_feedback(intermediate_position)
            time.sleep(0.5)

        set_led(arduino_UNO, int(color), "ON")
        execute_on_robot(plotter, motion)
        clear_leds(arduino_UNO)
        
if __name__ == "__main__":
    main()