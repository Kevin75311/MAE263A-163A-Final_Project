import time
from robot import Robot
from planner import PathGenerator
from simulator import Simulator

def main():
    robot = Robot("COM5", 57600, 4)
    pathgen = PathGenerator()
    simulator = Simulator()
    
    pathgen.set_path("./data/steps/cube.step")
    pathgen.set_resolution(5)
    segments = pathgen.generate_from_step()

    # test the motor first
    print("===============================")
    print("[Main] Starting motor test...")
    print("===============================")
    robot.torque_enable()
    robot.set_angles_with_feedback([0,0,0,0])
    robot.set_angles_with_feedback([90,90,90,90])
    robot.set_angles_with_feedback([0,0,0,0])
    print("[Main] Motor test complete.")
    print("===============================")
    time.sleep(5)

    print("[Main] Starting simulation...")
    print("===============================")
    simulator.simulate(segments, pause_time=0.005)
    print("[Main] Simulation complete.")
    print("===============================")

    print("[Main] Starting real-world execution...")
    print("===============================")
    for i, segment in enumerate(segments):
        start_pt = segment[0]
        end_pt = segment[1]
        
        start_angles = robot.inverse_kinematics(start_pt, 0, degrees=True)
        end_angles = robot.inverse_kinematics(end_pt, 0, degrees=True)
        
        print(f"Segment {i+1}/{len(segments)}: Moving to start angles {start_angles}", end='\r', flush=True)

        robot.set_angles_with_feedback(start_angles)
        robot.set_angles_with_feedback(end_angles)

    print("\n[Main] Real-world execution complete.")
    print("===============================")


if __name__ == "__main__":
    main()
