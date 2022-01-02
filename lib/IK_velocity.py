import numpy as np
from numpy.linalg import matrix_rank
from lib.calcJacobian import calcJacobian


def IK_velocity(q_in, v_in, omega_in):
    """
    :param q: 0 x 7 vector corresponding to the robot's current configuration.
    :param v: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 0 x 7 vector corresponding to the joint velocities. If v and omega
         are infeasible, then dq should minimize the least squares error. If v
         and omega have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    ## STUDENT CODE GOES HERE

    dq = np.zeros(7)

    J_raw = calcJacobian(q_in)
    zeta_raw = np.concatenate((v_in, omega_in), axis=0)

    zeta = zeta_raw[~np.isnan(zeta_raw)]
    J = J_raw[~np.isnan(zeta_raw)]


    J_Augmented = np.column_stack((J, zeta.T))

    J_Plus = np.matmul(J.transpose(), np.linalg.inv(np.matmul(J, J.transpose())))



    dq = np.linalg.lstsq(J, zeta, rcond=None)[0]
    
    """
    if ((matrix_rank(J)) == (matrix_rank(J_Augmented))):
         dq = np.linalg.solve(J_Plus, zeta)
    elif (matrix_rank(J) != matrix_rank(J_Augmented)):
         dq = np.linalg.lstsq(J_Plus, zeta, rcond=None)

     """

    return dq
