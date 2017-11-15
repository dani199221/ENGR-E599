import numpy as np
import math
from math import sin, cos, tan, exp

#http://groups.csail.mit.edu/robotics-center/public_papers/Landry15.pdf
#specifications for crazyflie
m = 0.027 #kg
g = 9.81 #m/s2
I = np.array([[2.3951e-5,         0,         0],\
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


#  ctf = km/kf kf and km are parameters that need to be measured 
#  [ F  ]    [ 1   1   1      1]    [ F1 ] 
#  | M1 |  = | 0  -d   0      d| *  | F2 | 
#  | M2 |    | d   0   -d     0|    | F3 |
#  [ M3 ]    [-ctf ctf -ctf ctf]    [ F4 ]

class Quadcopter:
    def __init__(self,linear_pos, angular_pos):
        # Position [x, y, z] of the quadrotor in inertial frame
        self.position = np.array(linear_pos) 
        # Velocity [dx/dt, dy/dt, dz/dt] of the quadrotor in inertial frame
        self.velocity = np.zeros((3,1)) 
        # Euler angles [phi, theta, psi] in body frame
        self.orientation = np.array(angular_pos)
        # Angular velocity [p, q, r]
        self.ang_velocity = np.zeros((3,1))

    #Rotation matrix R from the body frame to the inertial frame
    def rotation_matrix(self): #inverse of matrix is its transpose
        phi, theta, sy = self.orientation
        return np.array([ [cos(sy)*cos(theta), cos(sy)*sin(theta)*sin(phi) - sin(sy)*cos(phi), cos(sy)*sin(theta)*cos(phi) + sin(sy)*sin(phi)],\
                          [sin(sy)*cos(theta), sin(sy)*sin(theta)*sin(phi) + cos(sy)*cos(phi), sin(sy)*sin(theta)*cos(phi) - cos(sy)*sin(phi)],\
                          [ -1* sin(theta)   , cos(theta)*sin(phi)                           , cos(theta)*cos(phi)                           ]\
                        ])
   
    #transformation matrix for angular velocities from  inertial frame to the body frame
    def transformation_matrix(self):
        phi, theta, sy = self.angular_pos
        return np.array([[1, 0,                 -1*sin(theta)],\
                         [0, cos(phi),    cos(theta)*sin(phi)],\
                         [0, -1*sin(phi), cos(theta)*cos(phi)]\
                        ])
    

    #given F and M calculate the thrust of individual motors
    def individual_motor_thrust(self, F, M):
        Ainv = np.linalg.inv(A)
        mat = np.insert(M,0,F)
        return Ainv.dot(mat)


def main():
    q = Quadcopter([0,0,0], [0,0,0])
    print q.individual_motor_thrust(12, np.array([0,0,0]))


if __name__ == '__main__':
    main()

