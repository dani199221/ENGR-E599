from quadcopter import Quadcopter
from controller import Controller
import matplotlib.pyplot as plt
import numpy as np
import pdb
import math
from math import sin, cos, tan, exp

if __name__ == '__main__':
    print "hello world"
    quad = Quadcopter()
    v = []
    z = []
    pre = np.array([0,0,0])
    time = [x * 0.1 for x in range(0,100)]
    xd = []#[0.4*t, 0.4*sin(np.pi * t), 0.6*cos(np.pi*t)]
    vd = []#[0.4, 0.4*np.pi*cos(np.pi *t), -0.6*np.pi*sin(np.pi*t)]
    ad = []#[0, -0.4*np.pi*np.pi*sin(np.pi *t), -0.6*np.pi*np.pi*cos(np.pi*t)]
    b1d = []#[cos(np.pi*t), sin(np.pi)*t, 0]

    for t in time:
        xd.append([0.4*t, 0.4*sin(np.pi * t), 0.6*cos(np.pi*t)]) 
        vd.append([0.4, 0.4*np.pi*cos(np.pi *t), -0.6*np.pi*sin(np.pi*t)])
        ad.append([0, -0.4*np.pi*np.pi*sin(np.pi *t), -0.6*np.pi*np.pi*cos(np.pi*t)])
        b1d.append([cos(np.pi*t), sin(np.pi)*t, 0])

    print vd 
    print ad
    
    for t in range (0, len(time)):
        F,M = quad.getFM(0.027*9.81/4,0.027*9.81/4,0.027*9.81/4,0.027*9.81/4,)
        quad.update(t,0.1,F, M)
        pos = quad.position()
        vel = quad.velocity()
        z.append(pos[2])
        v.append(vel[2])
        print pos-pre
        pre = pos

    plt.plot(time, v, color='lightblue', linewidth=3)
    #plt.plot(itr, z, color='lightblue', linewidth=3)
    plt.show()
    
