from quadcopter import Quadcopter
from controller import Controller
import numpy as np
import ipdb
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
    dt = 1 
    time = [x*dt for x in range(0,100)]
    for t in time:
        curr_state = quad.curr_state();
        F, M = cont.update(curr_state, t, dt)
        quad.update(t,dt,F,M)
        a = quad.position()
    #pyplot.plot(time,x_val, 'r',y_val,'g',z_val, 'b') 
    #pyplot.show()

