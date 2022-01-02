import numpy as np
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy

from lib.calculateFK import FK
import random
import math

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


fk = FK()

# get joint limits
lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])


# RRTGraph class for each joint
class RRTGraph:
    def __init__(self, start, goal, map, lowerLim, upperLim):     
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.map = map

        # Data structure for storing coordinates and parents
        self.q = []
        self.parent = []

        # Add start to the data structure
        self.q.append(start)
        self.parent.append(0)

        # Obstacles
        self.obstacles = map.obstacles

        # Path
        self.goalstate = None
        self.path = []

        # Bounaries
        self.lowerLim = lowerLim
        self.upperLim = upperLim


    def addNode(self, n, q):
        self.q.insert(n, q)

    def removeNode(self, n):
        self.q.pop(n)
    
    def addEdge(self, parent, child):
        self.parent.insert(child, parent)
    
    def removeEdge(self, n):
        self.parent.pop(n)

    def numOfNodes(self):
        return np.shape(self.q)[0] # Need to check if it can return num of rows

    def distance(self, n1, n2):
        n1_jointPositions, n1_T0e = FK.forward(fk, self.q[n1])
        n2_jointPositions, n2_T0e = FK.forward(fk, self.q[n2])

        distance = np.linalg.norm(n2_jointPositions[0] - n1_jointPositions[0])\
                    + np.linalg.norm(n2_jointPositions[1] - n1_jointPositions[1])\
                    + np.linalg.norm(n2_jointPositions[2] - n1_jointPositions[2])\
                    + np.linalg.norm(n2_jointPositions[3] - n1_jointPositions[3])\
                    + np.linalg.norm(n2_jointPositions[4] - n1_jointPositions[4])\
                    + np.linalg.norm(n2_jointPositions[5] - n1_jointPositions[5])\
                    + np.linalg.norm(n2_jointPositions[6] - n1_jointPositions[6])
        return distance
        

    def sample_with_step(self, step_percent):
        interval = self.upperLim - self.lowerLim
        last_config = self.q[self.numOfNodes()-1]
        step_upperLim = last_config + interval * step_percent
        step_lowerLim = last_config - interval * step_percent

        random_config_with_step = np.array([\
                    random.uniform(step_lowerLim[0], step_upperLim[0]),\
                    random.uniform(step_lowerLim[1], step_upperLim[1]),\
                    random.uniform(step_lowerLim[2], step_upperLim[2]),\
                    random.uniform(step_lowerLim[3], step_upperLim[3]),\
                    random.uniform(step_lowerLim[4], step_upperLim[4]),\
                    random.uniform(step_lowerLim[5], step_upperLim[5]),\
                    random.uniform(step_lowerLim[6], step_upperLim[6]),\
                    ]) 
        return np.clip(random_config_with_step, self.lowerLim, self.upperLim)

    def sample(self):
        return np.array([\
                    random.uniform(self.lowerLim[0], self.upperLim[0]),\
                    random.uniform(self.lowerLim[1], self.upperLim[1]),\
                    random.uniform(self.lowerLim[2], self.upperLim[2]),\
                    random.uniform(self.lowerLim[3], self.upperLim[3]),\
                    random.uniform(self.lowerLim[4], self.upperLim[4]),\
                    random.uniform(self.lowerLim[5], self.upperLim[5]),\
                    random.uniform(self.lowerLim[6], self.upperLim[6]),\
                    ]) 
    
    def nearest(self, n):
        min_distance = self.distance(0, n)
        nearest_node = 0
        for i in range(0, n):
            if self.distance(i, n) < min_distance:
                min_distance = self.distance(i, n)
                nearest_node = i
        return nearest_node
    
    def isRobotCollided(self, q1, q2):
        # detect collisions for a robot configuration q

        sampled_configs, number_of_sample = self.sampleCollisionDetectionPoints(q1, q2, 0.2)

        sampled_jointPositions = []

        for config in sampled_configs:
            current_jointPositions, current_T0e = FK.forward(fk, config)
            sampled_jointPositions.append(current_jointPositions)


        q1_jointPositions, q1_T0e = FK.forward(fk, q1)
        q2_jointPositions, q2_T0e = FK.forward(fk, q2)        
        for sample_index in range(number_of_sample - 1):
            for obstacle in self.obstacles:
                if (True in detectCollision(sampled_jointPositions[sample_index], sampled_jointPositions[sample_index + 1], obstacle)):
                    print("\nRobot collides with obstacles\n")
                    return True

        
        print("\nRobot does not collide with obstacles\n")
        return False

    def sampleCollisionDetectionPoints(self, q1, q2, interpolate_percent):
        sampled_configs = []
        increment = (q2 - q1)/(1 / interpolate_percent)
        interpolation_completion = 0
        number_of_sample = 0
        while (interpolation_completion != 1):
            q1 = q1 + increment
            sampled_configs.append(q1)
            interpolation_completion = interpolation_completion + interpolate_percent
            number_of_sample += 1
        sampled_configs.append(q2)
        number_of_sample += 1
        return sampled_configs, number_of_sample


    def connect(self, n1, n2):
        q1 = self.q[n1]
        q2 = self.q[n2]
        if self.isRobotCollided(q1, q2):
            self.removeNode(n2)
            return False
        else:
            self.addEdge(n1, n2)
            return True
    
    def checkGoalConnection(self, rand_node):  
        if (not self.isRobotCollided(self.q[rand_node], self.goal)):
            goal_node = self.numOfNodes()
            self.addNode(goal_node, self.goal)
            self.goalstate = goal_node
            connect_status = self.connect(rand_node, goal_node)
            self.goalFlag = True
            return True
        else:
            return False
    
    def expand(self):
        n = self.numOfNodes()
        #q = self.sample_with_step(0.1)
        q = self.sample()
        self.addNode(n, q) 
        nearest = self.nearest(n)

       # self.visualize(n, nearest)

        connect_status = self.connect(nearest, n)
        if connect_status:
            self.checkGoalConnection(n)
        return self.q, self.parent
    
    def pathToGoal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            new_config = self.parent[self.goalstate - 1]
            while(new_config != 0):
                self.path.append(new_config)
                new_config = self.parent[new_config]
            self.path.append(0)
        return self.goalFlag

    def getPathConfigs(self):
        path_configs = [] 
        path_configs.append(self.start)
        for node in self.path:
            q = self.q[node -1]
            path_configs.append(q)
        print(path_configs)
        return path_configs

    def visualize(self, n1, n2):
        q1 = self.q[n1]
        q2 = self.q[n2]
        q1_jointPositions, q1_T0e = FK.forward(fk, q1)
        q2_jointPositions, q2_T0e = FK.forward(fk, q2)

        fig = plt.figure()
        ax = plt.axes(projection='3d')
        q1_x = q1_jointPositions[:, 0]
        q1_y = q1_jointPositions[:, 1]
        q1_z = q1_jointPositions[:, 2]

        q2_x = q2_jointPositions[:, 0]
        q2_y = q2_jointPositions[:, 1]
        q2_z = q2_jointPositions[:, 2]

        ax.plot3D(q1_x, q1_y, q1_z, 'red')
        ax.plot3D(q2_x, q2_y, q2_z, 'blue')
        self.addVisualizedObstacles(ax)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()
        return True

    def addVisualizedObstacles(self, ax):
        obstacles = self.obstacles
        for obstacle in obstacles:
            xmin = obstacle[0]
            ymin = obstacle[1]
            zmin = obstacle[2]
            xmax = obstacle[3]
            ymax = obstacle[4]
            zmax = obstacle[5]

            print(obstacle)
            Z = np.array([\
                    [xmin, ymin, zmin],\
                    [xmin, ymin, zmax],\
                    [xmin, ymax, zmax],\
                    [xmin, ymax, zmin],\
                    [xmax, ymin, zmin],\
                    [xmax, ymin, zmax],\
                    [xmax, ymax, zmax],\
                    [xmax, ymax, zmin],\
            ])

            r = [-1,1]
            X, Y = np.meshgrid(r, r)
            
            ax.scatter3D(Z[:, 0], Z[:, 1], Z[:, 2])


            verts = [[Z[0],Z[1],Z[2],Z[3]],
                        [Z[4],Z[5],Z[6],Z[7]], 
                        [Z[0],Z[1],Z[5],Z[4]],
                        [Z[2],Z[3],Z[7],Z[6]], 
                        [Z[1],Z[2],Z[6],Z[5]],
                        [Z[4],Z[7],Z[3],Z[0]]]

            ax.add_collection3d(Poly3DCollection(verts,\
            facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))      
        return True
         



def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """
    
    graph = RRTGraph(start, goal, map, lowerLim, upperLim)
    iteration = 0
    while (not graph.pathToGoal()):
        q, parent = graph.expand()
        iteration += 1

        #print("q: ", graph.q, "\n")
        #print("parent: ", graph.parent)

    #graph.visualize(graph.numOfNodes() - 2, graph.numOfNodes() - 1)
    return graph.getPathConfigs()

if __name__ == '__main__':
    map_struct = loadmap("../maps/map2.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))