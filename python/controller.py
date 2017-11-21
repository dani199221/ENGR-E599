import numpy as np
import scipy.integrate as integrate
import math
from math import sin, cos, tan
e2 = np.array([0,1,0])
e3 = np.array([0,0,1])

class Controller:
    def __init__(self, mass,t, xd, vd, ad, bd):
        self.g = 9.81
        self.mass = mass
        self.k_x = 0 
        self.k_v = 0
        self.k_r = 0
        self.k_w = 0
        self.t = t   #array of time
        self.xd = xd #dict of desired velocity over time
        self.vd = vd #dict of desired acceleration over time
        self.ad = ad #dict of desired acceleration over time
        self.b1d = b1d #dict of desired direction over time

    def getFM(self, curr_state, t, dt):
        
        #get e_x e_v
        e_x = curr_state[0:3] - self.xd[t]
        e_v = curr_state[3:6] - self.vd[t]
        
        #steps to get R and Rdesired
        
        ac_des = self.ad[t] #get the desired acceleration
        
        res = -1* self.k_x*e_x - self.k_v*e_v - self.mass * np.array([0,0,self.g]) + self.mass* ac_des
        b3d = res/np.linalg.norm(res) #get b3d from eq 12

        b2d = np.cross(b3d, b1d)/np.linalg.norm(np.cross(b3d, b1d)) #get b2d from b3d and b1d
        new_b1d = np.cross(b2d,b3d) #get new b1d from corss product of b2d and b3d

        #ger Rd from new_bq, b2d,b3d
        Rd =  np.array([[new_b1d[0], b2d[0], b3d[0]], [new_b1d[1], b2d[1], b3d[1]], [new_b1d[2], b2d[2], b3d[2]]]) 

        R = self.rotation_matrix(curr_state[6:9])#get rotation matrix from euler angles
        
        #get e_R
        e_R = 1.0/2 * vee_map(np.dot(Rd.T, R) - np.dot(R.T, Rd))
        
        #get e_w and pre requisites
        w = curr_state[9:12]
        #calculate wd
        #still not clear
        #need to calculate  Rd_dot somehow 
        Rd_dot = 0 # how to calculate this???
        wd = self.vee_map(np.dot(Rd.T,Rd_dot))

        e_w =  w - np.dot (np.dot(R.transpose(), Rd), wd)

        #calculate f
        a = -1* (-1* self.k_x*e_x - self.k_v*e_v - self.mass * np.array([0,0,g]) + self.mass* ac_des) 
        b = np.dot(R, np.array([0,0,1]))
        F = np.dot(a,b)

        #calculate M
        w = curr_state[9:12]
        first = -1* self.k_r* e_R - k_w * e_w
        second = np.cross(w, np.dot(I, w))  
        a = np.dot(vee_map(w),R.transpose())
        b = np.dot(Rd,wd)
        c = np.dot(np.dot(R.transpose(),Rd),wddot)
        third = -1 * np.dot(J, self.vee_map(np.dot(a,b) - c))
        
        M = first + second + third
 
        return F,M

    def vee_map(self, mat): #converts a 3x3 matrix to a 3x1 matrix
        return np.array([ mat[2][1], mat[0][2], mat[1][0]])

    #https://www.wikiwand.com/en/Skew-symmetric_matrix
    def hat_map(self, mat): #returns the 3x3 skew symmetric matrix of the rotation matrix which is 3x1 
        return np.array([[          0, -1 * mat[2],     mat[1]],
                         [     mat[2],           0,-1 * mat[0]],
                         [-1 * mat[1],      mat[0],         0 ]])
 
    def rotation_matrix(self, state): #inverse of matrix is its transpose
        phi, theta, sy = state
        return np.array([ [cos(sy)*cos(theta), cos(sy)*sin(theta)*sin(phi) - sin(sy)*cos(phi), cos(sy)*sin(theta)*cos(phi) + sin(sy)*sin(phi)],\
                          [sin(sy)*cos(theta), sin(sy)*sin(theta)*sin(phi) + cos(sy)*cos(phi), sin(sy)*sin(theta)*cos(phi) - cos(sy)*sin(phi)],\
                          [ -1* sin(theta)   , cos(theta)*sin(phi)                           , cos(theta)*cos(phi)                           ]\
                        ])

