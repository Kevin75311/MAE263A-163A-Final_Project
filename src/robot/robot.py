# from typing import Tuple
import numpy as np
from numpy.linalg import LinAlgError
import matplotlib.pyplot as plt
from dynio import dxl
from motor import MX28AR
from settings import constants


class Robot:
    def __init__(self, port:str, baud_rate:int, num_motors:int):
        self.num_motors = num_motors
        self.dxl_comm = dxl.DynamixelIO(port, baud_rate)
        self.motors = [MX28AR(self.dxl_comm, id+1) for id in range(num_motors)]
        self.color = "red"

    def set_positions(self, positions):
        for i, motor in enumerate(self.motors):
            if positions[i, 1] == None: continue
            motor.set_position(int(positions[i, 1]))

    def get_positions(self):
        return np.array([motor.get_position() for motor in self.motors])
    
    def set_angles(self, angles):
        for i, motor in enumerate(self.motors):
            # check if angle is in the range
            angles[i] = angles[i] % 360
            if angles[i] < 0:
                angles[i] += 360

            if angles[i] == None: continue
            motor.set_angle(angles[i])

    def set_angles_with_feedback(self, angles):
        for i, motor in enumerate(self.motors):
            # check if angle is in the range
            angles[i] = angles[i] % 360
            if angles[i] < 0:
                angles[i] += 360

            if angles[i] == None: continue
            motor.set_angle(angles[i])
        
        while True:
            current_positions = self.get_angles()
            if all(abs(current_positions[i] - angles[i]) < constants.tolerance for i in range(len(angles))):
                break

    def get_angles(self):
        return np.array([motor.get_angle() for motor in self.motors])

    def set_gains(self, gains):
        if gains == None:
            for motor in self.motors:
                motor.set_PID_gains() # default gains
        else:
            for i, motor in enumerate(self.motors):
                motor.set_PID_gains(*gains[i,:])

    def torque_enable(self):
        for motor in self.motors:
            motor.torque_enable()

    def torque_disable(self):
        for motor in self.motors:
            motor.torque_disable()

    def apply_write_table(self, data_name, vals):
        for i, motor in enumerate(self.motors):
            if vals[i] == None: continue
            motor.write_control_table(data_name, int(vals[i]))

    def apply_read_table(self, data_name):
        return np.array([motor.read_control_table(data_name) for motor in self.motors])
    
        # ---- low-level transforms ----
    
    def _get_T_01(self, theta_0: float, degrees: bool = False) -> np.ndarray:
        if degrees:
            theta_0 = np.radians(theta_0)
        c0 = np.cos(theta_0)
        s0 = np.sin(theta_0)
        # prismatic displacement along z of frame 1 relative to 0
        d_0 = (
            constants.GEOM[0]["l_crank"] * c0
            + np.sqrt(
                constants.GEOM[0]["l_crod"] ** 2
                - (constants.GEOM[0]["ht_diff"] + constants.GEOM[0]["l_crank"] * s0) ** 2
            )
        )
        T_01 = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, d_0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        return T_01

    def _get_T_ij_revolute(self, theta: float, link_idx: int, degrees: bool = False) -> np.ndarray:
        """
        Generic revolute transform for the joints 1,2,3 using the same pattern.
        link_idx is 1,2,3 mapping into constants.GEOM
        """
        if degrees:
            theta = np.radians(theta)
        c = np.cos(theta)
        s = np.sin(theta)
        if link_idx == 1:
            disp = constants.GEOM[1]["h_tower"]
            off = constants.GEOM[1]["offset"]
        elif link_idx == 2:
            disp = constants.GEOM[2]["l_link"]
            off = constants.GEOM[2]["offset"]
        elif link_idx == 3:
            disp = constants.GEOM[3]["l_link"]
            off = constants.GEOM[3]["offset"]
        else:
            raise ValueError("link_idx must be 1,2 or 3")
        T = np.array(
            [
                [c, -s, 0.0, disp],
                [s, c, 0.0, 0.0],
                [0.0, 0.0, 1.0, off],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        return T

    # ---- forward kinematics ----
    def forward_kinematics(self, joint_angles, degrees: bool = False) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute full transform of end-effector tip in base frame {B}.
        Returns (position_vector [3x1], T_Bf [4x4]).
        """

        theta_0 = joint_angles[0]
        theta_1 = joint_angles[1]
        theta_2 = joint_angles[2]
        theta_3 = joint_angles[3]

        T_01 = self._get_T_01(theta_0, degrees=degrees)
        T_12 = self._get_T_ij_revolute(theta_1, 1, degrees=degrees)
        T_23 = self._get_T_ij_revolute(theta_2, 2, degrees=degrees)
        T_34 = self._get_T_ij_revolute(theta_3, 3, degrees=degrees)

        T_Bf = constants.T_B0 @ T_01 @ T_12 @ T_23 @ T_34 @ self.T_4f
        pos = T_Bf[:3, 3].copy()
        return pos, T_Bf

    # ---- jacobian ----
    def jacobian(self, theta_0: float, theta_1: float, theta_2: float, theta_3: float, degrees: bool = False) -> np.ndarray:
        """
        calculates the space jacobian of a manipulator given its configuration, which relates the task space (x,y,z,t) to joint space (t0,t1,t2,t3)
        
                   [[dx/dt0  dx/dt1  dx/dt2  dx/dt3]
        Jacobian =  [dy/dt0  dy/dt1  dy/dt2  dy/dt3]
                    [dz/dt0  dz/dt1  dz/dt2  dx/dt3]
                    [dt/dt0  dt/dt1  dt/dt2  dt/dt3]]
        """
        z_ax = np.array([0.0, 0.0, 1.0])
        T_01 = self._get_T_01(theta_0, degrees)
        T_12 = self._get_T_ij_revolute(theta_1, 1, degrees)
        T_23 = self._get_T_ij_revolute(theta_2, 2, degrees)
        T_34 = self._get_T_ij_revolute(theta_3, 3, degrees)

        T_B2 = constants.T_B0 @ T_01 @ T_12
        T_B3 = T_B2 @ T_23
        T_B4 = T_B3 @ T_34
        T_Bf = T_B4 @ self.T_4f

        # convert theta_0 to radians for analytic derivative if input degrees=True
        if degrees:
            t0 = np.radians(theta_0)
        else:
            t0 = theta_0
        s0 = np.sin(t0)
        c0 = np.cos(t0)

        J = np.zeros((4, 4))
        dz_dt0 = -constants.GEOM[0]["l_crank"] * s0 - constants.GEOM[0]["l_crank"] * c0 * (
            (constants.GEOM[0]["ht_diff"] + constants.GEOM[0]["l_crank"] * s0)
            / np.sqrt(constants.GEOM[0]["l_crod"] ** 2 - (constants.GEOM[0]["ht_diff"] + constants.GEOM[0]["l_crank"] * s0) ** 2)
        )

        # accidentally packed the jacobian transposed the first time around lol
        J[:,0] = np.array([0.0, 0.0, dz_dt0, 0.0])
        J[:,1] = np.append(np.cross(z_ax, T_Bf[:3, 3] - T_B2[:3, 3]), 1.0)
        J[:,2] = np.append(np.cross(z_ax, T_Bf[:3, 3] - T_B3[:3, 3]), 1.0)
        J[:,3] = np.append(np.cross(z_ax, T_Bf[:3, 3] - T_B4[:3, 3]), 1.0)
        return J

    # ---- workspace check (internal) ----
    def check_workspace(self, x: float, y: float, z: float, theta: float, degrees: bool = False) -> None:
        """
        Raises ValueError if target is outside workspace (z or radial).
        """
        # if z < constants.LIMS["z_min"] or z > constants.LIMS["z_max"]:
        #     raise ValueError("Target z-coordinate outside of workspace.")
            

        if degrees:
            theta = np.radians(theta)
        # rotate target angle -90 deg to match your link frame orientation
        theta_local = theta - np.pi / 2.0

        x_wrist = y - constants.GEOM[4]["l_eff"] * np.cos(theta_local)
        y_wrist = -x - constants.GEOM[4]["l_eff"] * np.sin(theta_local)
        r_wrist = np.sqrt(x_wrist ** 2 + y_wrist ** 2)
        # if r_wrist > constants.LIMS["r_max"]:
        #     raise ValueError("Target x, y, theta combination outside of workspace.")

    # ---- inverse kinematics (analytical) ----
    def inverse_kinematics(self, target, theta: float, degrees: bool = False) -> np.ndarray:
        """
        Analytical inverse kinematics. Returns sols array shape (4,2):
          sols[0] = [theta0_elbup, theta0_elbdn]
          sols[1] = [theta1_elbup, theta1_elbdn]
          sols[2] = [theta2_elbup, theta2_elbdn]
          sols[3] = [theta3_elbup, theta3_elbdn]
        Angles returned in radians (unless degrees=True).
        """

        x = target[0]
        y = target[1]
        z = target[2]

        # check workspace first (raises ValueError on bad target)
        self.check_workspace(x, y, z, theta, degrees)

        # convert to radians if requested
        if degrees:
            theta = np.radians(theta)

        # rotate target angle -90 deg to match the x/y-axis orientation of link frames
        theta_local = theta - (np.pi / 2)

        # theta_0 (prismatic joint's driving crank angle) computation
        d_0 = z - constants.T_B0[2, 3] - (constants.GEOM[1]["offset"] + constants.GEOM[2]["offset"] + constants.GEOM[3]["offset"])
        r_plinkage = np.sqrt(d_0 ** 2 + constants.GEOM[0]["ht_diff"] ** 2)
        beta_pris = np.arctan2(-constants.GEOM[0]["ht_diff"], d_0)
        # law of cosines to get psi
        psi_pris = np.arccos(
            (constants.GEOM[0]["l_crank"] ** 2 + r_plinkage ** 2 - constants.GEOM[0]["l_crod"] ** 2)
            / (2.0 * r_plinkage * constants.GEOM[0]["l_crank"])
        )

        theta_0_elbup = beta_pris + psi_pris
        theta_0_elbdn = beta_pris - psi_pris

        # wrist (planar part) coordinates in tower frame (match your original)
        x_wrist = y - constants.GEOM[4]["l_eff"] * np.cos(theta_local)
        y_wrist = -x - constants.GEOM[4]["l_eff"] * np.sin(theta_local)
        r_wrist = np.sqrt(x_wrist ** 2 + y_wrist ** 2)

        # theta_1
        beta = np.arctan2(y_wrist, x_wrist)
        psi = np.arccos((constants.GEOM[2]["l_link"] ** 2 + r_wrist ** 2 - constants.GEOM[3]["l_link"] ** 2) / (2.0 * constants.GEOM[2]["l_link"] * r_wrist))
        theta_1_elbup = beta + psi
        theta_1_elbdn = beta - psi

        # theta_2 (elbow)
        theta_2_elbdn = np.arccos((r_wrist ** 2 - constants.GEOM[2]["l_link"] ** 2 - constants.GEOM[3]["l_link"] ** 2) / (2.0 * constants.GEOM[2]["l_link"] * constants.GEOM[3]["l_link"]))
        theta_2_elbup = 2.0 * np.pi - theta_2_elbdn

        # theta_3 to satisfy orientation
        theta_3_elbup = theta_local - theta_1_elbup - theta_2_elbup + constants.GEOM[4][self.color]
        theta_3_elbdn = theta_local - theta_1_elbdn - theta_2_elbdn + constants.GEOM[4][self.color]

        # sols = np.array(
        #     [
        #         [theta_0_elbup, theta_0_elbdn],
        #         [theta_1_elbup, theta_1_elbdn],
        #         [theta_2_elbup, theta_2_elbdn],
        #         [theta_3_elbup, theta_3_elbdn],
        #     ]
        # )

        # need a better solution to select elbow up vs down here; for now just return elbow-up
        sols = np.array([theta_0_elbup, theta_1_elbup, theta_2_elbup, theta_3_elbup])

        if degrees:
            return np.degrees(sols)
        return sols
    
    # ---- inverse kinematics (numerical) ----
    def inverse_kinematics_numerical(self, target, theta: float, guess, degrees: bool = False, tol=1e-3, maxiters:int = 100) -> np.ndarray:
        '''
        Uses numerical inverse kinematics to find a set of joint positions that accomplishes a desired end effector position, starting from a guess vector of joint angles and using newton-raphson iteration to converge toward a solution
        '''

        self.check_workspace(*target,theta,degrees)
        x_goal = np.array([*target,theta])
        joint_angles = guess.copy()

        for i in range(maxiters):   # newton-raphson iteration to find a solution
            
            pos, T_Bf = self.forward_kinematics(*joint_angles, degrees) # fk to find end position given guess
            theta_end = np.arctan2(T_Bf[1,0],T_Bf[0,0])

            x_curr = np.append(pos,theta_end) # vector of end effector coordinates with angle appended
            err = x_curr - x_goal # positional error

            J = self.jacobian(*joint_angles, degrees) # space jacobian of manipulator in current configuration

            try:
                dtheta = 0.2 * np.linalg.solve(J,err) # scale solution to mitigate overshooting
            except LinAlgError:
                raise LinAlgError(f'Singularity encountered while solving at iteration {i}')

            joint_angles -= dtheta # adjust guess with correction factor

            if np.linalg.norm(err) < tol: # solution converged to within tolerance
                degrees[-1] += constants.GEOM[4][self.color]
                if degrees:
                    joint_angles = np.degrees(joint_angles)
                return joint_angles
            
        raise RuntimeError('solution failed to converge :(') # max iters reached, complain

    # ---- plotting helper (optional) ----
    def plot_manipulator(self, theta_0, theta_1, theta_2, theta_3, ax=None, degrees: bool = False):
        """
        Thin wrapper around matplotlib quiver plot from your original code.
        Returns (end_effector_pos, list_of_quiver_objects)
        """
        if ax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection="3d")

        T_01 = self._get_T_01(theta_0, degrees)
        T_12 = self._get_T_ij_revolute(theta_1, 1, degrees)
        T_23 = self._get_T_ij_revolute(theta_2, 2, degrees)
        T_34 = self._get_T_ij_revolute(theta_3, 3, degrees)

        T_B1 = constants.T_B0 @ T_01
        T_B2 = T_B1 @ T_12
        T_B3 = T_B2 @ T_23
        T_B4 = T_B3 @ T_34
        T_Bf = T_B4 @ self.T_4f

        # for crank visualization, convert theta_0 to radians for trig (if input degrees)
        theta0_rad = np.radians(theta_0) if degrees else theta_0

        # quiver arrows (exact same visual parts as original)
        q0 = ax.quiver(*constants.T_B0[:3, 3], *(T_B1[:3, 3] - constants.T_B0[:3, 3]), arrow_length_ratio=0.1, color="red", linestyle="--")
        vec_ht_diff = np.array([0.0, constants.GEOM[0]["ht_diff"], 0.0])
        vec_crank = np.array([0.0, constants.GEOM[0]["l_crank"] * np.sin(theta0_rad), constants.GEOM[0]["l_crank"] * np.cos(theta0_rad)])
        q0c = ax.quiver(*(constants.T_B0[:3, 3] + vec_ht_diff), *vec_crank, arrow_length_ratio=0.1, color="red")
        q0r = ax.quiver(*(constants.T_B0[:3, 3] + vec_ht_diff + vec_crank), *(T_B1[:3, 3] - constants.T_B0[:3, 3] - vec_crank - vec_ht_diff), arrow_length_ratio=0.1, color="red")
        q1 = ax.quiver(*T_B1[:3, 3], *(T_B2[:3, 3] - T_B1[:3, 3]), arrow_length_ratio=0.1, color="blue")
        q2 = ax.quiver(*T_B2[:3, 3], *(T_B3[:3, 3] - T_B2[:3, 3]), arrow_length_ratio=0.1, color="green")
        q3 = ax.quiver(*T_B3[:3, 3], *(T_B4[:3, 3] - T_B3[:3, 3]), arrow_length_ratio=0.1, color="orange")
        q4 = ax.quiver(*T_B4[:3, 3], *(T_Bf[:3, 3] - T_B4[:3, 3]), arrow_length_ratio=0.1, color="black")

        return T_Bf[:3, 3], [q0, q0c, q0r, q1, q2, q3, q4]