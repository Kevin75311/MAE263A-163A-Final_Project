import time
import tqdm
import serial
from robot import Robot
from planner import PathGenerator
from simulator import Simulator

def set_led(ser, led_number, state):
    cmd = f"LED {led_number} {state}\n"
    ser.write(cmd.encode())

def clear_leds(ser):
    for i in range(1, 6):
        set_led(ser, i, "OFF")

def set_all_leds_on(ser):
    for i in range(1, 6):
        set_led(ser, i, "ON")

def main():
    # robot = Robot("COM5", 57600, 5)
    # robot.set_gains(None) # default gains
    # arduino_UNO = serial.Serial("COM7", 9600, timeout=1)
    pathgen = PathGenerator()
    simulator = Simulator()
    time.sleep(2)

    # # LED test
    # print("===============================")
    # print("[Main] Connected to Arduino. Starting LED test...")
    # for i in range(1, 6):
    #     clear_leds(arduino_UNO)
    #     set_led(arduino_UNO, i, "ON")
    #     time.sleep(1)
    # clear_leds(arduino_UNO)
    # print("[Main] LED test complete.")
    # print("===============================")
    # time.sleep(2)


    # # homing
    # print("===============================")
    # print("[Main] Starting robot homing...")
    # steps = 6
    # current_position = robot.get_angles()
    # homing_position = [178, 181, 170, 180, 180]
    # robot.torque_enable()
    # for step in range(1, steps + 1):
    #     intermediate_position = [
    #         current_position[i] + (homing_position[i] - current_position[i]) * step / steps
    #         for i in range(5)
    #     ]
    #     robot.set_angles_with_feedback(intermediate_position)
    #     time.sleep(0.5)
    # print("[Main] Robot homing complete.")
    # print("===============================")
    # time.sleep(2)
    
    # path generation
    print("===============================")
    print("[Main] Starting path generation...")
    pathgen.set_path("./data/steps/cube.step")
    pathgen.set_resolution(6)
    segments = pathgen.generate_from_step()
    print(f"[Main] Path generation complete. Generated {len(segments)} segments.")
    print("===============================")
    time.sleep(2)

    # simulation
    print("===============================")
    print("[Main] Starting simulation...")
    # simulator.simulate(segments, pause_time=0.1)
    simulator.simulate_with_robot(segments, pause_time=0.01)
    print("[Main] Simulation complete.")
    print("===============================")


    # command = input("[Main] Type Y to continue to real-world execution, or type anything else to exit: ")
    # if command.lower() != 'y':
    #     print("[Main] Exiting program.")
    #     return
    
    # real-world execution
    # print("===============================")
    # print("[Main] Starting real-world execution...")
    # for i in tqdm.tqdm(range(len(segments)), desc="Executing segments"):
    #     start_pt = segments[i][0]
    #     end_pt = segments[i][1]
        
    #     start_angles = robot.inverse_kinematics(start_pt, 0, degrees=True)
    #     end_angles = robot.inverse_kinematics(end_pt, 0, degrees=True)
    #     start_angles = [358 - start_angles[0] + homing_position[0], start_angles[0] + homing_position[1], start_angles[1] + homing_position[2], start_angles[2] + homing_position[3], start_angles[3] + homing_position[4] + homing_position[4]]
    #     end_angles = [358 - end_angles[0] + homing_position[0], end_angles[0] + homing_position[1], end_angles[1] + homing_position[2], end_angles[2] + homing_position[3], end_angles[3] + homing_position[4]]
        
    #     if i == 0:
    #         # move slowly to first position
    #         print(start_angles)
    #         current_position = robot.get_angles()
    #         steps = 10
    #         for step in range(1, steps + 1):
    #             intermediate_position = [
    #                 current_position[j] + (start_angles[j] - current_position[j]) * step / steps
    #                 for j in range(5)
    #             ]
    #             robot.set_angles_with_feedback(intermediate_position)
    #             time.sleep(0.5)
            
        
    #     print(f"Segment {i+1}/{len(segments)}: Moving to start angles {start_angles}", end='\r', flush=True)
    #     clear_leds(arduino_UNO)
    #     # robot.set_angles_with_feedback(start_angles)
    #     set_led(arduino_UNO, 1, "ON")
    #     # robot.set_angles_with_feedback(end_angles)
    # print("[Main] Real-world execution complete.")
    # print("===============================")
    # return

if __name__ == "__main__":
    main()