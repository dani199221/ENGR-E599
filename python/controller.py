import numpy as np


e1 = np.array([1,0,0])
e2 = np.array([0,1,0])
e3 = np.array([0,0,1])


class controller:
    def __init__(self, trajectory, mass, curr_pose):
        self.mass = mass
        self.trajectory = trajectory
        
        self.k_x = 0 
        self.k_v = 0
        self.k_R = 0
        self.k_w = 0

    
    def get_pos_error(self, x_curr, x_des):
        return np.array([x_curr[0]- x_des[0], x_curr[1]- x_des[1], x_curr[2]- x_des[2]])

    def get_vel_error(self, v_curr, v_des):
        return np.array([v_curr[0]- v_des[0], v_curr[1]- v_des[1], v_curr[2]- v_des[2]])
        
    def get_orientation_error(self, r_curr, r_des):
        # orien_err = 1/2 * ( RdesTranspose* R_curr - R_currTranspose * Rdes   )
        return 0

    def get_ang_velocity_error(self, ang_vel_curr, ang_vel_des):
        # ang_vel_err = ang_vel_in_body_frame -RT * RD * ang_vel_desired
        return 0

    def get_errors(self, curr_state, goal_state):
        x_des, v_des, r_des, ang_vel_des = get_goal_state() 
        
        e_x = get_pos_error(x_curr, x_des)
        e_v = get_vel_error(v_curr, v_des) 
        e_r = get_orientation_error(r_curr, r_des)
        e_w = get_ang_velocity_error(ang_vel_curr, ang_vel_des)
        
        return e_x, e_v, e_r, e_w 
    
    def get_force():
        

