from  quadcopter import Quadcopter
from controller import Controller
import numpy as np
import math
from math import sin, cos, tan, exp

from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':

    x_vald = []
    y_vald = []
    z_vald = []
    x_val = []
    y_val = []
    z_val = []
    quad = Quadcopter()
    cont = Controller()
    dt = 0.02 
    time = [x*dt for x in range(0,1000)]
    F = []
    M = []
    fil = open("f1.txt",'r')
    fig = pyplot.figure()
    ax = fig.add_subplot(111, projection='3d')
    for l in fil:
        a = l.split()
        Fa, Ma = quad.getFM( float(a[0]),float(a[1]),float(a[2]),float(a[3])) 
        F.append(Fa)
        M.append(Ma)
    fig = pyplot.figure(0)
    ax = fig.add_subplot(111, projection='3d')

    fig1 = pyplot.figure(1)
    ax2 = fig1.add_subplot(111, projection='3d')
    for t in time:
        curr_state = quad.curr_state();

        F, M = cont.update(curr_state, t, dt)
        quad.update(t,dt,F,M)
        x_vald.append(cont.get_xd()[0])
        y_vald.append(cont.get_xd()[1])
        z_vald.append(cont.get_xd()[2])
        x_val.append(quad.position()[0])
        y_val.append(quad.position()[1])
        z_val.append(quad.position()[2])
    #ax.plot(x_val[150:], y_val[150:], z_val[150:])
    #pyplot.show()
    ax2.plot(x_val[300:], y_val[300:], z_val[300:])
    #ax2.plot(x_vald[300:], y_vald[300:], z_vald[300:])
    ax2.text2D(0.05, 0.95, "Actual path noise", transform=ax.transAxes)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    pyplot.show()
