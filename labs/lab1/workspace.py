from lib.calculateFK import FK
from core.interfaces import ArmController

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from numpy.random import default_rng
import numpy as np

import random

fk = FK()

# the dictionary below contains the data returned by calling arm.joint_limits()
limits = [
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -1.7628, 'upper': 1.7628},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -3.0718, 'upper': -0.0698},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -0.0175, 'upper': 3.7525},
    {'lower': -2.8973, 'upper': 2.8973}
 ]

# TODO: create plot(s) which visualize the reachable workspace of the Panda arm,
# accounting for the joint limits.
#
# We've included some very basic plotting commands below, but you can find
# more functionality at https://matplotlib.org/stable/index.html
fig = plt.figure()
j0 = fig.add_subplot(711, projection='3d', title='Joint 0')
j1 = fig.add_subplot(712, projection='3d', title='Joint 1')
j2 = fig.add_subplot(713, projection='3d', title='Joint 2')
j3 = fig.add_subplot(714, projection='3d', title='Joint 3')
j4 = fig.add_subplot(715, projection='3d', title='Joint 4')
j5 = fig.add_subplot(716, projection='3d', title='Joint 5')
j6 = fig.add_subplot(717, projection='3d', title='Joint 6')

counter = 150.0
while counter>0:
    #ng=default_rng()
    #input = rng.random((1,7))
    #input = np.random.uniform(low=-4.0, high=4.0, size=(7))
    input = np.array([round(random.uniform(-2.8973, 2.8973),2),\
    round(random.uniform(-1.7628, 1.7628),2),\
    round(random.uniform(-2.8973, 2.8973),2),\
    round(random.uniform(-3.0718, -0.0698),2),\
    round(random.uniform(-2.8973, 2.8973),2),\
    round(random.uniform(-0.0175, 3.7525),2),\
    round(random.uniform(-2.8973, 2.8973),2)])
    
    
    result, T0e = FK.forward(fk, input)

    print(result)
    
    
    j0.scatter(result[0][0], result[0][1], result[0][2])
    j1.scatter(result[1][0], result[1][1], result[1][2])
    j2.scatter(result[2][0], result[2][1], result[2][2])
    j3.scatter(result[3][0], result[3][1], result[3][2])
    j4.scatter(result[4][0], result[4][1], result[4][2])
    j5.scatter(result[5][0], result[5][1], result[5][2])
    j6.scatter(result[6][0], result[6][1], result[6][2])
    
    counter = counter-0.1

# TODO: update this with real results
#ax.scatter(1,1,1) # plot the point (1,1,1)

plt.show()
