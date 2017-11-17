import numpy as np
import scipy.integrate as integrate
import math
from math import sin, cos, tan, exp


#http://groups.csail.mit.edu/robotics-center/public_papers/Landry15.pdf
#specifications for crazyflie
m = 0.027 #kg
g = 9.81 #m/s2

#The inertia matrix
J = I = np.array([[2.3951e-5,         0,         0],\
                  [0,         2.3951e-5,         0],\
                  [0,                 0, 3.2347e-5]\
                 ])
km = 1.8580e-5 #Nms2
kf = 0.005022  #Ns2
ctf = km/kf
d = 0.092/2  #92mm propellor to propellor, d is the distance between center of mass and propellor
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
 
    def state_dot(self, state, t, F, M):
        x, y, z, xdot, ydot, zdot, p, q, r, pd, qd, rd = state

        # linear acceleration
        accel =   np.array([0, 0, g]) - 1.0/m * np.dot(self.rotation_matrix(),np.array([0, 0, F]))
       
        # angular acceleration - Euler's equation of motion
        omega = np.array([pd,qd,rd])
        pqrdot = np.dot( np.linalg.inv(I) ,  M - np.cross(omega, np.dot(I,omega)) )
        
        state_dot = np.zeros(12)
        state_dot[0]  = xdot
        state_dot[1]  = ydot
        state_dot[2]  = zdot
        state_dot[3]  = accel[0]
        state_dot[4]  = accel[1]
        state_dot[5]  = accel[2]
        state_dot[6]  = pd
        state_dot[7]  = qd
        state_dot[8]  = rd
        state_dot[9] = pqrdot[0]
        state_dot[10] = pqrdot[1]
        state_dot[11] = pqrdot[2]
        
        return state_dot    

    def update(self, dt, F, M):
        # limit thrust and Moment
        L = d 
        r = d
        #prop_thrusts = params.invA.dot(np.r_[np.array([[F]]), M])
        #prop_thrusts_clamped = np.maximum(np.minimum(prop_thrusts, params.maxF/4), params.minF/4)
        #F = np.sum(prop_thrusts_clamped)
        #M = params.A[1:].dot(prop_thrusts_clamped)
        self.state = integrate.odeint(self.state_dot, self.state, [0,dt], args = (F, M))[1]
       
    #Rotation matrix R from the body frame to the inertial frame
    def rotation_matrix(self): #inverse of matrix is its transpose
        phi, theta, sy = self.state[6:9]
        return np.array([ [cos(sy)*cos(theta), cos(sy)*sin(theta)*sin(phi) - sin(sy)*cos(phi), cos(sy)*sin(theta)*cos(phi) + sin(sy)*sin(phi)],\
                          [sin(sy)*cos(theta), sin(sy)*sin(theta)*sin(phi) + cos(sy)*cos(phi), sin(sy)*sin(theta)*cos(phi) - cos(sy)*sin(phi)],\
                          [ -1* sin(theta)   , cos(theta)*sin(phi)                           , cos(theta)*cos(phi)                           ]\
                        ])


q = Quadcopter()
q.update(0.1, 12, np.array([1,2,3]))
print q.state
q.update(0.1, 12, np.array([1,2,3]))
print q.state
