from dynio import dxl
import numpy as np

def set_PID_gains(motor:dxl.DynamixelMotor, kP=256, kI=0, kD=0):
    '''
    write PID gains to a servo. if no gains are passed will set to default values
    '''
    if kP != None:
        kP_scaled = kP / 8
        if kP_scaled > 254 or kP_scaled < 0: raise ArithmeticError('kP outside of acceptable bounds (0-2032)')
        motor.write_control_table('P_Gain',int(kP_scaled))
    if kI != None:
        kI_scaled = kI * 1000/2048
        if kI_scaled > 254 or kI_scaled < 0: raise ArithmeticError('kI outside of acceptable bounds (0-520)')
        motor.write_control_table('I_Gain',int(kI_scaled))
    if kD != None:
        kD_scaled = kD * 4/1000
        if kD_scaled > 254 or kD_scaled < 0: raise ArithmeticError('kD outside of acceptable bounds (0-63500)')
        motor.write_control_table('D_Gain',int(kD_scaled))

class MotorArr:
    def __init__(self, device_name:str, baud_rate:int, device_ids:list):
        self.dxl_comm = dxl.DynamixelIO(device_name, baud_rate) # setup a single communication object to handle serial data
        self.motor_arr = np.array([self.dxl_comm.new_mx28(dev_id, 2) for dev_id in device_ids])

    def set_positions(self, positions): # position setpoints
        for i, motor in enumerate(self.motor_arr):
            if positions[i] == None: continue
            motor.set_position(int(positions[i])) # int casting because otherwise the bitwise math gets angry

    def set_angles(self, angles): # angle setpoints
        for i, motor in enumerate(self.motor_arr):
            if angles[i] == None: continue
            motor.set_angle(angles[i])

    def set_gains(self, gains):
        if gains == None: # if a single None val is passed, reset all gains
            for motor in self.motor_arr:
                set_PID_gains(motor)
        else:
            for i, motor in enumerate(self.motor_arr):
                set_PID_gains(motor, *gains[i,:])

    def torque_enable(self): # need to call this before anything will move
        for motor in self.motor_arr:
            motor.torque_enable()

    def torque_disable(self):
        for motor in self.motor_arr:
            motor.torque_disable()

    def get_positions(self):
        return np.array([motor.get_position() for motor in self.motor_arr])
    
    def get_angles(self):
        return np.array([motor.get_angle() for motor in self.motor_arr])
    
    def apply_write_table(self, data_name, vals): # more general function to write to any value in the control table
        for i, motor in enumerate(self.motor_arr):
            if vals[i] == None: continue
            motor.write_control_table(data_name, int(vals[i]))

    def apply_read_table(self, data_name):
        return np.array([motor.read_control_table(data_name) for motor in self.motor_arr])