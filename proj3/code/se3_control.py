import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        # STUDENT CODE HERE
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz]))  # kg*m^2
        self.g = 9.81  # m/s^2
        self.K_F = 6.11e-8
        self.K_M = 1.5e-9
        self.k_p = np.diag(np.array([7, 7, 15]))
        # self.k_p = np.diag(np.array([k_p_x, k_p_y, k_p_z]))
        # self.k_p = np.diag(np.array([0, 6, 8]))
        self.k_d = np.diag(np.array([7, 7, 10]))
        # self.k_p = 3
        # self.k_d = 7.5
        self.k_p_Euler = np.diag(np.array([3300, 2000, 250]))
        self.k_d_Euler = np.diag(np.array([30, 20, 10]))
        self.gamma = self.k_drag / self.k_thrust
        self.K_R = np.diag(np.array([2000, 2000, 180])) #2200, 2200, 130
        self.K_omega = np.diag(np.array([100, 100, 70])) #100, 100 70

    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        cmd_accelaration = flat_output["x_ddot"].reshape(3, 1) - (
                    self.k_d @ ((state["v"] - flat_output["x_dot"]).reshape(3, 1))) - \
                           (self.k_p @ ((state["x"] - flat_output["x"]).reshape(3, 1)))

        # cmd_accelaration = flat_output["x_ddot"] - (self.k_d*(state["v"]-flat_output["x_dot"])) - \
        #                    (self.k_p*(state["x"]-flat_output["x"]))
        # cmd_accelaration = cmd_accelaration.reshape(3, 1)
        # print("CMD", cmd_accelaration.shape)
        # cmd_accelaration = flat_output["x_ddot"].reshape(3,1) - (self.k_d * (-flat_output["x_dot"].reshape(3,1) + state["v"].reshape(3,1))) - \
        #                    (self.k_p * (-flat_output["x"].reshape(3,1) + state["x"].reshape(3,1)))

        # print("CMD ACCELARATION", cmd_accelaration)

        # exit()
        F_des = (self.mass * cmd_accelaration) + np.array([[0], [0], [self.mass * self.g]])
        r = Rotation.from_quat(state["q"])
        R = r.as_matrix()
        b_3 = np.dot(R, np.array([[0], [0], [1]]))
        u1 = np.dot(b_3.T[0], F_des)
        # print("U1", u1)
        # b3_des = (F_des / np.linalg.norm(F_des)).reshape(-1, 1)
        b3_des = F_des / np.linalg.norm(F_des)

        # print("B3 des", b3_des)
        # u1 = np.dot(b3_des.reshape(1, -1)[0], F_des)
        yaw_angle = flat_output["yaw"]
        yaw_dir = np.array([[np.cos(yaw_angle)], [np.sin(yaw_angle)], [0]])
        cross_prod = np.cross(b3_des.reshape(1, 3), yaw_dir.reshape(1, 3))[0]
        b2_des = cross_prod / np.linalg.norm(cross_prod)
        new_cross_prod = np.cross(b2_des, b3_des.reshape(1, 3))
        # Calculate Rotation
        R_des = np.hstack((new_cross_prod[0].reshape(3, 1), b2_des.reshape(3, 1), b3_des))

        # Error in Rotation
        hat_matrix = (np.dot(R_des.T, R) - np.dot(R.T, R_des))


        v_operator_vector = np.array([hat_matrix[2][1], hat_matrix[0][2], hat_matrix[1][0]]).reshape(-1, 1)
        e_R = 0.5 * v_operator_vector
        e_omega = state["w"].reshape(-1, 1)

        # Apply torque in error direction to reduce error
        u2 = self.inertia @ (-self.K_R @ e_R - self.K_omega @ e_omega)
        u = np.concatenate((np.array([u1]), u2), axis=0)

        arm_length_matrix = np.array([[1, 1, 1, 1],
                                      [0, self.arm_length, 0, -self.arm_length],
                                      [-self.arm_length, 0, self.arm_length, 0],
                                      [self.gamma, -self.gamma, self.gamma, -self.gamma]])

        # arm_length_matrix = np.array([[self.k_thrust,  self.k_thrust,   self.k_thrust,  self.k_thrust],
        #                              [0,self.k_thrust*self.arm_length, 0,-self.k_thrust*self.arm_length],
        #                              [-self.k_thrust*self.arm_length, 0,self.k_thrust*self.arm_length,0],
        #                              [self.k_drag, -self.k_drag,self.k_drag,-self.k_drag]])

        motor_speed_sq = np.linalg.inv(arm_length_matrix) @ u
        # print("Force vector", force_vector.shape)

        cmd_motor_speeds = np.sqrt(np.abs(motor_speed_sq) / self.k_thrust) * np.sign(motor_speed_sq)
        # motor_speed_sq = np.where(motor_speed_sq < 0, 0, motor_speed_sq)
        # cmd_motor_speeds = np.sqrt(np.abs(motor_speed_sq))
        # print("CMD", cmd_motor_speeds)
        # exit()
        cmd_motor_speeds = cmd_motor_speeds.reshape(4, )
        cmd_thrust = u1
        cmd_moment = u2
        cmd_q = Rotation.from_matrix(R_des).as_quat()

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
        return control_input
