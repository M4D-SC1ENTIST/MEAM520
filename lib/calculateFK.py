import numpy as np
from math import pi
import math

class FK():

    def __init__(self):
        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout
        z1 = 0.141
        z2 = 0.192
        z3 = 0.195
        z4 = 0.121
        z5 = 0.083

        z6 = -0.051
        z7 = -0.159

        x1 = 0.082
        x2 = 0.125
        x3 = 0.259
        x4 = 0.088

        y1 = 0.015
        

        pass

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions - 7 x 3 matrix, where each row corresponds to a rotational joint of the robot
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your code starts here

        # jointPositions = np.zeros((7,3))
        
        # T0e = np.identity(4)

        #rotation_matrix_z = A_6_7 = np.array([[math.cos(pi/2), -math.sin(pi/2), 0, 0], [math.sin(pi/2), math.cos(pi/2), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        #DH Matrix
        """
        A_0_1 = np.array([[1, 0, 0, 0],[0, 1, 0, 0], [0, 0, 1, 0.141], [0, 0, 0, 1]])
        A_1_2 = np.array([[np.float(math.cos(q[0])), 0, np.float(math.sin(q[0])), 0], [np.float(math.sin(q[0])), 0, np.float(-math.cos(q[0])), 0], [0, 1, 0, np.float(0.192)], [0, 0, 0, 1]])
        A_2_3 = np.array([[np.float(math.cos(q[1])),0, np.float(-math.sin(q[1])), 0], [np.float(math.sin(q[1])), 0, np.float(math.cos(q[1])), 0*math.sin(q[1])], [0, -1, 0, 0], [0, 0, 0, 1]])
        A_3_4 = np.array([[math.cos(q[2]), 0, math.sin(q[2]), 0.082*math.cos(q[2])], [math.sin(q[2]), 0, -math.cos(q[2]), 0.082*math.sin(q[2])], [0, 1, 0, 0.316], [0, 0, 0, 1]])
        A_4_5 = np.array([[math.cos(q[3]-pi/2), 0, math.sin(q[3]-pi/2), math.cos(q[3]+pi/2)*0.125-math.sin(q[3]+pi/2)*0.083], [math.sin(q[3]-pi/2), 0, -math.cos(q[3]-pi/2), 0.083*math.cos(q[3]+pi/2)+math.sin(q[3]+pi/2)*0.125], [0, 1, 0, 0], [0, 0, 0, 1]])
        
        A_5_6 = np.array([[math.cos(q[4]), 0, math.sin(q[4]), -0.259], [0, 1, 0, 0], [-math.sin(q[4]), 0, math.cos(q[4]), 0], [0, 0, 0, 1]])
        #A_5_6 = np.array([[1, 0, 1, -0.259], [0, math.cos(q[4]), -math.sin(q[4]), 0], [0, math.sin(q[4]), math.cos(q[4]), 0], [0, 0, 0, 1]])
        #A_6_7 = np.array([[math.cos(q[5]), -math.sin(q[5]), 0, -0.088], [math.sin(q[5]), math.cos(q[5]), 0, 0], [0, 0, 1, -0.051], [0, 0, 0, 1]])   #Z ROTAT
        #A_6_7 = np.array([[math.cos(q[5]), 0, math.sin(q[5]), 0], [0, 1 , 0, 0], [-math.sin(q[5]), 0, math.cos(q[5]), 0], [0, 0, 0, 1]])  #Y ROTAT
        A_6_7 = np.array([[1, 0, 1, 0], [0, math.cos(q[5]), -math.sin(q[5]), 0], [0, math.sin(q[5]), math.cos(q[5]), 0], [0, 0, 0, 1]])   # X ROTAT
        #A_6_7_r = np.array([[math.cos(-q[5]), -math.sin(-q[5]), 0, -0.088], [math.sin(-q[5]), math.cos(-q[5]), 0, 0], [0, 0, 1, -0.051], [0, 0, 0, 1]])
        T0e = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(A_0_1, A_1_2) ,A_2_3), A_3_4), A_4_5), A_5_6), A_6_7)
        
        J_0_1 = np.array([[1, 0, 0, 0],[0, 1, 0, 0], [0, 0, 1, np.float(0.141)], [0, 0, 0, 1]])
        J_1_2 = np.array([[np.float(math.cos(q[0])), 0, np.float(math.sin(q[0])), 0], [np.float(math.sin(q[0])), 0, np.float(-math.cos(q[0])), 0], [0, 1, 0, np.float(0.192)], [0, 0, 0, 1]])
        J_2_3 = np.array([[np.float(math.cos(q[1])),0, np.float(-math.sin(q[1])), 0], [np.float(math.sin(q[1])), 0, np.float(math.cos(q[1])), 0*math.sin(q[1])+0.195], [0, -1, 0, 0], [0, 0, 0, 1]])
        J_3_4 = np.array([[math.cos(q[2]), 0, math.sin(q[2]), 0.082*math.cos(q[2])], [math.sin(q[2]), 0, -math.cos(q[2]), 0.082*math.sin(q[2])], [0, 1, 0, 0.316], [0, 0, 0, 1]])
        #J_3_4 = np.array([[0],[0],[0.082],[1]])
        J_4_5 = np.array([[math.cos(q[3]-pi/2), 0, math.sin(q[3]-pi/2), math.cos(q[3]+pi/2)*0.125-math.sin(q[3]+pi/2)*0.083], [math.sin(q[3]-pi/2), 0, -math.cos(q[3]-pi/2), 0.083*math.cos(q[3]+pi/2)+math.sin(q[3]+pi/2)*0.125], [0, 1, 0, 0], [0, 0, 0, 1]])
        """
        #J_5_6 = np.array([[math.cos(q[4]), 0, math.sin(q[4]), -0.259], [0, 1, 0, 0], [-math.sin(q[4]), 0, math.cos(q[4]), 0], [0, 0, 0, 1]])
        
        
        #J_5_6 = np.matmul(A_4_5, np.array([0,0,0,1]]) 
        
        #J_5_6 = np.array([[math.cos(q[4]-pi), 0, math.sin(q[4]-pi), -0.384], [math.sin(q[4]-pi), 0, -math.cos(q[4]-pi), -0.015], [0, -1, 0, 0], [0, 0, 0, 1]])
        #J_6_7 = np.array([[math.cos(q[4]-pi), 0, math.sin(q[4]-pi), -0.384], [math.sin(q[4]-pi), 0, -math.cos(q[4]-pi), -0.015], [0, -1, 0, 0], [0, 0, 0, 1]])
        #J_6_7 = np.array([[math.cos(q[5]-pi/2), 0, -math.sin(q[5]-pi/2), 0.472*math.cos(q[5]-pi/2)], [math.sin(q[5]-pi/2), 0, math.cos(q[5]-pi/2), 0.472*math.sin(q[5]-pi/2)], A_6_7])
        #J_6_7 = np.array([[-math.cos(q[5]), math.sin(q[5]), 0, 0.472*math.sin(q[5])], [-math.sin(q[5]), -math.cos(q[5]), 0, -0.472*math.cos(q[5])], [0, 0, 1, 0], [0, 0, 0, 1]])

        

        """
        Pos_5_6 = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(A_0_1, A_1_2) ,A_2_3), A_3_4), A_4_5), A_5_6), np.array([0,0,0,1]))
        Pos_6_7 = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(A_0_1, A_1_2) ,A_2_3), A_3_4), A_4_5), A_5_6), A_6_7), np.array([-0.088,-0.051, 0,1]))

        jointPositions = np.array([[J_0_1[0][3], J_0_1[1][3], J_0_1[2][3]],[np.matmul(A_0_1, J_1_2)[0][3], np.matmul(A_0_1, J_1_2)[1][3], np.matmul(A_0_1, J_1_2)[2][3]],\
            [np.matmul(np.matmul(A_0_1, A_1_2) ,J_2_3)[0][3], np.matmul(np.matmul(A_0_1, A_1_2) ,J_2_3)[1][3], np.matmul(np.matmul(A_0_1, A_1_2) ,J_2_3)[2][3]],\
            [np.matmul(np.matmul(np.matmul(A_0_1, A_1_2) ,A_2_3), J_3_4)[0][3],np.matmul(np.matmul(np.matmul(A_0_1, A_1_2) ,A_2_3), J_3_4)[1][3], np.matmul(np.matmul(np.matmul(A_0_1, A_1_2) ,A_2_3), J_3_4)[2][3]],\
            [np.matmul(np.matmul(np.matmul(np.matmul(A_0_1, A_1_2) ,A_2_3), A_3_4), J_4_5)[0][3], np.matmul(np.matmul(np.matmul(np.matmul(A_0_1, A_1_2) ,A_2_3), A_3_4), J_4_5)[1][3], np.matmul(np.matmul(np.matmul(np.matmul(A_0_1, A_1_2) ,A_2_3), A_3_4), J_4_5)[2][3]],\
            [Pos_5_6[0], Pos_5_6[1], Pos_5_6[2]],\
            [Pos_6_7[0], Pos_6_7[1], Pos_6_7[2]]])
        """
        AW0 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.141],[0,0,0,1]])
        A01 = np.array([[math.cos(q[0]), 0, math.sin(q[0]), 0], [math.sin(q[0]), 0, -math.cos(q[0]), 0], [0, 1, 0, 0.192], [0, 0, 0, 1]])
        A12 = np.array([[math.cos(-q[1]), 0, -math.sin(-q[1]), 0], [math.sin(-q[1]), 0, math.cos(-q[1]), 0], [0, -1, 0, 0], [0, 0, 0, 1]])
        A23 = np.array([[math.cos(q[2]), 0, math.sin(q[2]), 0.082*math.cos(q[2])], [math.sin(q[2]), 0, -math.cos(q[2]), 0.082*math.sin(q[2])], [0, 1, 0, 0.316], [0, 0, 0, 1]])
        A34 = np.array([[math.cos(q[3]+pi), 0, math.sin(q[3]+pi), 0.083*math.cos(q[3]+pi)], [math.sin(q[3]+pi), 0, -math.cos(q[3]+pi), 0.083*math.sin(q[3]+pi)], [0, 1, 0, 0], [0, 0, 0, 1]])
        A45 = np.array([[math.cos(q[4]), 0, -math.sin(q[4]), 0], [math.sin(q[4]), 0, math.cos(q[4]), 0], [0, -1, 0, 0.384], [0, 0, 0, 1]])
        A56 = np.array([[math.cos(q[5]-pi), 0, math.sin(q[5]-pi), 0.088*math.cos(q[5]-pi)], [math.sin(q[5]-pi), 0, -math.cos(q[5]-pi), 0.088*math.sin(q[5]-pi)], [0, 1, 0, 0],[0, 0, 0, 1]])
        A67 = np.array([[math.cos(q[6]-pi/4), -math.sin(q[6]-pi/4), 0, 0], [math.sin(q[6]-pi/4), math.cos(q[6]-pi/4), 0, 0], [0, 0, 1, 0.21], [0, 0, 0, 1]])

        
        T0 = np.matmul(AW0, A01)
        T1 = np.matmul(T0, A12)
        T2 = np.matmul(T1, A23)
        T3 = np.matmul(T2, A34)
        T4 = np.matmul(T3, A45)
        T5 = np.matmul(T4, A56)
        T6 = np.matmul(T5, A67)
        T0e = T6

        P1 = np.array([0,0,0.141,1])
        P2 = np.matmul(T0, np.array([0,0,0,1]))
        P3 = np.matmul(T1, np.array([0,0,0.195,1]))
        P4 = np.matmul(T2, np.array([0,0,0,1]))
        P5 = np.matmul(T3, np.array([0,0,0.125,1]))
        P6 = np.matmul(T4, np.array([0,0,-0.015,1]))
        P7 = np.matmul(T5, np.array([0,0,0.051,1]))

        #Joint specifically for collision detection enhancement
        """
        P8 = np.matmul(T0, np.array([0,0,0.096,1]))
        P9 = np.matmul(T1, np.array([0,0,0.0975,1]))

        P10 = np.matmul(T3, np.array([0,0,0.0625,1]))

        P11 = np.matmul(T5, np.array([0,0,0.0255,1]))
        """

        jointPositions = np.array([[P1[0], P1[1], P1[2]],\
        [P2[0], P2[1], P2[2]],\
        [P3[0], P3[1], P3[2]],\
        [P4[0], P4[1], P4[2]],\
        [P5[0], P5[1], P5[2]],\
        [P6[0], P6[1], P6[2]],\
        [P7[0], P7[1], P7[2]]])

        # Your code ends here
        

        return jointPositions, T0e
       
    # feel free to define additional helper methods to modularize your solution

if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)