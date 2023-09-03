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

        # initialising through second order response for kp and kd (position control)
        position_T = 1.77  # 1.685# desired settling time, seconds  #1.874
        position_wn = 6.833 / position_T # natural frequency
        self.kp_pos = (position_wn**2)*0.7
        self.kd_pos = (2*position_wn)*0.7
        # s_time = 2.0
        # omega_n =
        # self.kp_pos = np.array([10,10,30])
        # self.kd_pos = np.array([5,5,20])

        # att_T = 0.35   # 0.35 # more unstable when increased, didn't work
        # att_omega_n = 5.83 / att_T
        # self.kp_att = att_omega_n**2
        # self.kd_att = 2*att_omega_n

        self.kp_att = (np.array([900, 900, 80]))*0.7 # for attitude control
        self.kd_att = (np.array([120, 120, 20]))*0.7 # for attitude control

        gam = self.k_drag / self.k_thrust
        # print("k", k)
        L = self.arm_length

        #TM - Total Moment
        #f - total thrust

        self.f_TM = np.array([[1,1,1,1],
                                 [0,L,0,-L],
                                 [-L,0,L,0],
                                 [gam,-gam,gam,-gam]])
        self.f_TM_inv = np.linalg.inv(self.f_TM)


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

        def normalize(x):
            return x/np.linalg.norm(x)

        # gives the a1, a2, a3 value of the vee matrix
        def skew_sym (S):
            return np.array([-S[1,2], S[0,2], -S[0,1]])

        pos_err = state['x'] - flat_output['x']
        dpos_err = state['v'] - flat_output['x_dot']

        # taking m common outside and multiplying
        r_ddot_des = flat_output['x_ddot'] - (self.kp_pos*pos_err) - (self.kd_pos*dpos_err)


        F_des = self.mass * (r_ddot_des + np.array([0,0,self.g]))

        R = Rotation.from_quat(state['q']).as_matrix()
        # print("R", R)
        b3 = R @ np.array([0,0,1])
        u1 = np.dot(F_des, b3)
        print("u1", u1)

        b3_des = normalize(F_des)
        psi = flat_output['yaw']
        a_psi = np.array([np.cos(psi), np.sin(psi), 0])
        b2_des = normalize(np.cross(b3_des, a_psi))
        b1_des = np.cross(b2_des, b3_des)

        R_des = np.column_stack([b1_des, b2_des, b3_des])

        # To calculate orientation error

        e_r = 0.5*(R_des.T @ R - R.T @ R_des)
        # print("er", e_r)
        att_err = skew_sym(e_r)
        # att_err = np.diag(e_r)

        w_des = np.array([0,0, flat_output['yaw_dot']])
        e_w = state['w'] - w_des
        print("e_w",e_w)

        u2 = self.inertia @ (-self.kp_att*att_err - self.kd_att*e_w)
        print("u2", u2)

        Total_moment = np.array([u1, u2[0], u2[1], u2[2]])
        cmd_motor_forces = self.f_TM_inv @ Total_moment
        cmd_motor_speeds = cmd_motor_forces/self.k_thrust
        cmd_motor_speeds = np.sign(cmd_motor_speeds) * np.sqrt(np.abs(cmd_motor_speeds))

        print("Motor speeds", cmd_motor_speeds)

        cmd_thrust = u1  # thrust is u1
        cmd_moment = u2  # moment is u2
        cmd_q = Rotation.from_matrix(R_des).as_quat()  # need to check whether this is correct

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}

        # print("flat output", flat_output)
        return control_input

