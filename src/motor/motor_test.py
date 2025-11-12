from dynio import *
from motor_utils import MotorArr
import numpy as np
from time import sleep

port = 'COM5' # COM5 is where the u2d2 appears for me, but check device manager to see where it is for you

motors = MotorArr(port, 57600, [1,2,3,4,5,6])
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

for row in target_positions:
    motors.set_positions(row) # write the positions to the motors
    while True:
        positions = motors.get_positions() # see how close we've gotten
        print(positions)
        if np.linalg.norm(positions - row) < 5:
            break
        sleep(0.01)

