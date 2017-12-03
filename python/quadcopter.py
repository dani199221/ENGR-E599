import numpy as np
import scipy.integrate as integrate
import math
from math import sin, cos, tan, exp


#http://groups.csail.mit.edu/robotics-center/public_papers/Landry15.pdf
m = 4.34 #kg
g = 9.81 #m/s2

#The inertia matrix
J = I = np.array([[0.0820,         0,         0],\
                  [0,         0.0845,         0],\
                  [0,                 0, 0.1377]\
                 ])
ctf = 8.004e-4 #km/kf
d = 0.315  
A = np.array([[  1,   1,    1,   1],\
             [   0,  -d,    0,   d],\
             [   d,   0,   -d,   0],\
             [-ctf, ctf, -ctf, ctf]])


class Quadcopter:
    """ Quadcopter class

    state  - 1 dimensional vector but used as 13 x 1. [x, y, z, xd, yd, zd, p, q, r, pd, qd, rd]
    x y z  = position in inertial frame
    xd yd zd = velocity in inertial frame
    p q r = roll pitch yaw in body frame
    pd qd rd = roll dot pitch dot yaw dot in body frame
    
    """

    def __init__(self):
        self.state = np.zeros(12) 
    
    def position(self):
        return self.state[0:3]

    def velocity(self):
        return self.state[3:6]

    def attitude(self):
        return self.state[6:9]
    
    def angular_velocity(self):
        return self.state[9:12]
    
    def curr_state(self):
        return self.state

    #get the deriviative of the state based on the given F and M
    def state_dot(self, state, t, F, M):
        x, y, z, xdot, ydot, zdot, p, q, r, pd, qd, rd = state
        
       # linear acceleration equation(2) in the paper
        accel =   np.array([0, 0, g]) - 1.0/m * np.dot(self.rotation_matrix(),np.array([0, 0, F]))
        # angular acceleration equation(5) in the paper
        omega = np.array([pd,qd,rd])
        ang_accel = np.dot( np.linalg.inv(I) ,  M - np.cross(omega, np.dot(I,omega)) )
       
        state_dot = np.array([xdot, ydot, zdot, accel[0], accel[1], accel[2], pd, qd, rd, ang_accel[0], ang_accel[1], ang_accel[2]])
         
        return state_dot    
    
    #update the state of the quadrotor from based upon input F and M
    def update(self,t, dt, F, M):
        F = self.individual_motor_thust(F, M)
        F = np.sum(F)
        
        self.state = integrate.odeint(self.state_dot, self.state, [t,t+dt], args = (F, M))[1]
       
    #get individual motor thrusts from the input F and M
    def individual_motor_thust(self, F, M):
        #  [ F  ]    [ 1   1   1      1]    [ F1 ] 
        #  | M1 |  = | 0  -d   0      d| *  | F2 | 
        #  | M2 |    | d   0   -d     0|    | F3 |
        #  [ M3 ]    [-ctf ctf -ctf ctf]    [ F4 ]
        Ainv = np.linalg.inv(A) #inverse of the 4x4 matrix
        mat = np.insert(M,0,F) #make the matrix on the L.H.S of the equation
        return Ainv.dot(mat)

    #get F and M from f1 f2 f3 f4
    def getFM(self, f1,f2,f3,f4):
        #  [ F  ]    [ 1   1   1      1]    [ F1 ] 
        #  | M1 |  = | 0  -d   0      d| *  | F2 | 
        #  | M2 |    | d   0   -d     0|    | F3 |
        #  [ M3 ]    [-ctf ctf -ctf ctf]    [ F4 ]
        a = np.array([f1,f2,f3,f4]) 
        res = A.dot(a)
        f = res[0]
        M = np.array([res[1], res[2], res[3]])
        return f,M 
    
    #Rotation matrix R from the body frame to the inertial frame
    def rotation_matrix(self): #inverse of matrix is its transpose
        phi, theta, sy = self.state[6:9]
        return np.array([ [cos(sy)*cos(theta),cos(sy)*sin(theta)*sin(phi)-sin(sy)*cos(phi),cos(sy)*sin(theta)*cos(phi)+sin(sy)*sin(phi)],\
                          [sin(sy)*cos(theta),sin(sy)*sin(theta)*sin(phi)+cos(sy)*cos(phi),sin(sy)*sin(theta)*cos(phi)-cos(sy)*sin(phi)],\
                          [ -1* sin(theta)   ,cos(theta)*sin(phi)                         ,cos(theta)*cos(phi)                         ]\
                        ])
   



        

