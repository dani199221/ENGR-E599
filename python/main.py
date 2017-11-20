from quadcopter import Quadcopter
from controller import Controller
import matplotlib.pyplot as plt
import numpy as np
import pdb



if __name__ == '__main__':
    print "hello world"
    quad = Quadcopter()
    v = []
    z = []
    pre = np.array([0,0,0])
    itr = [x * 0.1 for x in range(0,100)]
    for t in range (0, len(itr)):
        F,M = quad.getFM(-0.027*9.81/4,-0.027*9.81/4,-0.027*9.81/4,-0.027*9.81/4,)
        quad.update(t,0.1,F, M)
        pos = quad.position()
        vel = quad.velocity()
        z.append(pos[2])
        v.append(vel[2])
        print pos-pre
        pre = pos

    plt.plot(itr, z, color='lightblue', linewidth=3)
    plt.show()

