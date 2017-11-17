import numpy as np


e1 = np.array([1,0,0])
e2 = np.array([0,1,0])
e3 = np.array([0,0,1])


class Controller:
    def __init__(self, mass):
        self.mass = mass
        self.k_x = 0 
        self.k_v = 0
        self.k_r = 0
        self.k_w = 0

    
    def get_pos_error(self, x_curr, x_des):
        return np.array([x_curr[0]- x_des[0], x_curr[1]- x_des[1], x_curr[2]- x_des[2]])

    def get_vel_error(self, v_curr, v_des):
        return np.array([v_curr[0]- v_des[0], v_curr[1]- v_des[1], v_curr[2]- v_des[2]])
        
    def get_orientation_error(self, R, Rd):
        # orien_err = 1/2 * ( RdesTranspose* R_curr - R_currTranspose * Rdes   )
        return 1.0/2 * self.vee_map(  np.dot(Rd.transpose(), R) - np.dot(R.transpose(), Rd)  )

    def get_ang_velocity_error(self, w, wd, R, Rd):
        # ang_vel_err = ang_vel_in_body_frame -RT * RD * ang_vel_desired
        return w - np.dot (np.dot(R.transpose(), Rd), wd)

    def get_errors(self, curr_state, goal_state):
        x_des, v_des, r_des, ang_vel_des = get_goal_state() 
        
        e_x = get_pos_error(x_curr, x_des)
        e_v = get_vel_error(v_curr, v_des) 
        e_r = get_orientation_error(r_curr, r_des)
        e_w = get_ang_velocity_error(ang_vel_curr, ang_vel_des)
        
        return e_x, e_v, e_r, e_w 
    
    def get_force(self, e_x, e_v, g, acc_desired, R):
        a = -1* (-1* self.k_x*e_x - self.k_v*e_v - self.mass * np.array([0,0,g]) + self.mass* acc_desired) 
        b = np.dot(R, np.array([0,0,1]))
        return np.dot(a,b)

    def get_M(self, e_r, e_w, w, J, R, Rd, wd, wddot):
        first = -1* self.k_r* e_r - k_w * e_w
        second = np.cross(w, np.dot(I, w))  
        a = np.dot(vee_map(w),R.transpose())
        b = np.dot(Rd,wd)
        c = np.dot(np.dot(R.transpose(),Rd),wddot)
        third = -1 * np.dot(J, self.vee_map(np.dot(a,b) - c))
        
        return first + second + third


    def vee_map(self, mat): #converts a 3x3 matrix to a 3x1 matrix
        return np.array([ mat[2][1], mat[0][2], mat[1][0]])


    def hat_map(self, mat): #returns the 3x3 skew symmetric matrix of the rotation matrix 
        return np.array([[          0, -1 * mat[2],     mat[1]],
                         [     mat[2],           0,-1 * mat[0]],
                         [-1 * mat[1],      mat[0],         0 ]])
 


