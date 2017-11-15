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
    def rotation_matrix(self, orientation): #inverse of matrix is its transpose
        phi, theta, sy = orientation
        return np.array([ [cos(sy)*cos(theta), cos(sy)*sin(theta)*sin(phi) - sin(sy)*cos(phi), cos(sy)*sin(theta)*cos(phi) + sin(sy)*sin(phi)],\
                          [sin(sy)*cos(theta), sin(sy)*sin(theta)*sin(phi) + cos(sy)*cos(phi), sin(sy)*sin(theta)*cos(phi) - cos(sy)*sin(phi)],\
                          [ -1* sin(theta)   , cos(theta)*sin(phi)                           , cos(theta)*cos(phi)                           ]\
                        ])
   
    #transformation matrix for angular velocities from  inertial frame to the body frame
    def transformation_matrix(self, angular_pos):
        phi, theta, sy = angular_pos
        return np.array([[1, 0,                 -1*sin(theta)],\
                         [0, cos(phi),    cos(theta)*sin(phi)],\
                         [0, -1*sin(phi), cos(theta)*cos(phi)]\
                        ])
   

    #given F and M calculate the thrust of individual motors
    #equation (1) in paper
    def individual_motor_thrust(self, f, M):
        #  [ F  ]    [ 1   1   1      1]    [ F1 ] 
        #  | M1 |  = | 0  -d   0      d| *  | F2 | 
        #  | M2 |    | d   0   -d     0|    | F3 |
        #  [ M3 ]    [-ctf ctf -ctf ctf]    [ F4 ]

        Ainv = np.linalg.inv(A) #inverse of the 4x4 matrix
        mat = np.insert(M,0,f) #make the matrix on the L.H.S of the equation
        return Ainv.dot(mat)  #calculte F1 F2 F3 F4 and return 
    
    #equation (2) in paper
    #def velocity(self, pos_des):
    #equation (3) in paper
    def acceleration(self, f, orientation):
        # mvdot = mge3 - fRe3
        # vdot = ge3 - f/m * R * e3
        
        #calculate f/m * R * e3
        f_z = np.sum(F)/m
        f_z = np.array([0,0,f_z]) 
        rot_mat = self.rotation_matrix(orientation)
        res = np.dot(rot_mat,f_z)

        #calculate g_z
        g_z = np.zrray([0,0,g])
        
        return  g_z - res
    
    #equation (4) in paper
    #def R_dot(self, R, omega):
    
    #equation (5) in paper
    #def 
        


def main():
    q = Quadcopter([0,0,0], [0,0,0])
    print q.individual_motor_thrust(12, np.array([0,0,0]))


if __name__ == '__main__':
    main()

