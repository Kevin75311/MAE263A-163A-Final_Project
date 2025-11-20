class MX28AR():
    def __init__(self, dxl_comm, id):
        self.model = "MX28AR"
        self.device = dxl_comm.new_mx28(id, 2)

    def set_PID_gains(self, kP=256, kI=0, kD=0):
        if kP != None:
            kP_scaled = kP / 8
            if kP_scaled > 254 or kP_scaled < 0: raise ArithmeticError('kP outside of acceptable bounds (0-2032)')
            self.device.write_control_table('P_Gain',int(kP_scaled))
        if kI != None:
            kI_scaled = kI * 1000/2048
            if kI_scaled > 254 or kI_scaled < 0: raise ArithmeticError('kI outside of acceptable bounds (0-520)')
            self.device.write_control_table('I_Gain',int(kI_scaled))
        if kD != None:
            kD_scaled = kD * 4/1000
            if kD_scaled > 254 or kD_scaled < 0: raise ArithmeticError('kD outside of acceptable bounds (0-63500)')
            self.device.write_control_table('D_Gain',int(kD_scaled))
    
    # torque functions #

    def torque_enable(self):
        self.device.torque_enable()

    def torque_disable(self):
        self.device.torque_disable()

    # angle functions #

    def set_angle(self, angle):
        self.device.set_angle(angle)

    def get_angle(self):
        return self.device.get_angle()
    
    # position functions #

    def set_position_mode(self):
        self.device.set_position_mode()

    def set_extended_position_mode(self):
        self.device.set_extended_position_mode()

    def set_position(self, position):
        self.device.set_position(position)

    def get_position(self):
        return self.device.get_position()

    # velocity functions #

    def set_velocity_mode(self):
        self.device.set_velocity_mode()

    def set_velocity(self, velocity):
        self.device.set_velocity(velocity)

    # acceleration functions #
    
    def set_acceleration(self, acceleration):
        self.device.set_acceleration(acceleration)

    # others #

    def write_control_table(self, data_name, value):
        self.device.write_control_table(data_name, value)

    def read_control_table(self, data_name):
        return self.device.read_control_table(data_name)
    
    def get_current(self):
        return self.device.get_current()