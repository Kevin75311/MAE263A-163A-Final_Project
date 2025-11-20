from dynio import *
from motor_utils import MotorArr
import numpy as np
from time import sleep

port = 'COM7' # COM5 is where the u2d2 appears for me, but check device manager to see where it is for you

motors = MotorArr(port, 57600, [1,2,3,4])
motors.torque_enable()
target_positions = np.array([[0,0,0,0,0,0], # array of target positions for a basic demo
                             [2e3,0,0,0,0,0],
                             [2e3,2e3,0,0,0,0],
                             [2e3,2e3,2e3,0,0,0],
                             [2e3,2e3,2e3,2e3,0,0],
                             [2e3,2e3,2e3,2e3,2e3,0],
                             [2e3,2e3,2e3,2e3,2e3,2e3],
                             [0,2e3,0,2e3,0,2e3],
                             [2e3,0,2e3,0,2e3,0],
                             [0,0,0,0,0,0],
                             [4e3,4e3,4e3,4e3,4e3,4e3],
                             [5e2,3e3,5e2,3e3,5e2,3e3],
                             [0,2e3,0,2e3,0,2e3],
                             [2e3,0,2e3,0,2e3,0],])

# for row in target_positions:
#     motors.set_positions(row) # write the positions to the motors
#     while True:
#         positions = motors.get_positions() # see how close we've gotten
#         # print(positions)
#         if np.linalg.norm(positions - row) < 5:
#             print("Reached target.")
#             break
#         sleep(0.01)



# generate a test for set_angle
for angle in range(0, 351, 30):
    motors.set_angles([angle]*4)
    while True:
        angles = motors.get_angles()
        if all(abs(angles[i] - angle) < 2 for i in range(4)):
            print("Reached angle:", angle)
            break
        sleep(0.01)