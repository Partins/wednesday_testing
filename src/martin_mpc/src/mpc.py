#!/usr/bin/env python3

import rospy
import time
import math
import numpy as np
from scipy.signal import cont2discrete as c2d
import cvxpy as cp
import matplotlib.pyplot as plt 
from std_msgs.msg import Float32MultiArray

## Services
from gazebo_msgs.srv import GetModelState
from martin_mpc.srv import mpcsrv, mpcsrvResponse


class UAV_model:

    def __init__(self):
        
        rospy.init_node('mpc', anonymous=True)
        self.mpc_srv = rospy.Service('MPC_calc', mpcsrv, self.mpc_calc)
        self.Ax = 0.01
        self.Ay = 0.01
        self.Az = 0.01
        self.gravity = 9.80
        self.tau_roll = 0.05 # Time constant
        self.tau_pitch = 0.05 # Time constant
        self.K_roll = 0.15  # Roll angle gain
        self.K_pitch = 0.15 # Pitch angle gain
        RATE = 20
        self.Q = np.diag([1.5, 1.5, 15.0, 1.0, 1.0, 1.0, 0.03, 0.03])
        #self.Q = np.diag([10, 10, 15.0, 3.0, 3.0, 3.0, 1.0, 1.0]) # Andreas
        self.R = 1*np.diag([3, 3, 1.0])
        #self.R = 1*np.diag([20.0, 20.0, 20.0]) # Andreas

        # Max/min values
        self.umin = np.array([-3.14/12, -3.14/12, 0.0])
        self.umax = np.array([3.14/12, 3.14/12, 18])

        self.xmin = np.array([-np.inf, -np.inf, 0, -5.0, -5.0, -5.0,
                 -3.14/12,-3.14/12])
        self.xmax = np.array([np.inf, np.inf, 10.0, 5.0, 5.0, 5.0,
                  3.14/12, 3.14/12])

        self.Ac = np.zeros(shape=(8, 8), dtype=float)
        self.Bc = np.zeros(shape=(8, 3), dtype=float)

        # Martin
        self.Ac[0,3] = 1
        self.Ac[1,4] = 1
        self.Ac[2,5] = 1
        self.Ac[3,3] = -self.Ax
        self.Ac[3,7] = -self.gravity
        self.Ac[4,4] = -self.Ay
        self.Ac[4,6] = -self.gravity
        self.Ac[5,5] = -self.Az
        self.Ac[6,6] = -1.0 / self.tau_roll
        self.Ac[7,7] = -1.0 / self.tau_pitch

        # Paper
        #self.Ac[0,3] = 1
        #self.Ac[1,4] = 1
        #self.Ac[2,5] = 1
        #self.Ac[3,3] = -self.Ax
        #self.Ac[3,6] = self.gravity
        #self.Ac[4,4] = -self.Ay
        #self.Ac[4,7] = -self.gravity
        #self.Ac[5,5] = -self.Az
        #self.Ac[6,6] = -1.0 / self.tau_roll
        #self.Ac[7,7] = -1.0 / self.tau_pitch

        # Input transfer matrix
        self.Bc[5,2] = 1
        self.Bc[6,0] = self.K_roll / self.tau_roll
        self.Bc[7,1] = self.K_pitch / self.tau_pitch

        self.Cc = [1, 1, 1, 1, 1, 1, 1, 1]
        self.Dc = 0

        # Discretization
        self.Ad, self.Bd, self.Cd, self.Dd, dt = c2d((self.Ac, self.Bc, \
                                                     self.Cc, self.Dc), 1/RATE)
        rospy.logwarn("MPC READY")
        rospy.spin()
    
    
    def mpc_calc(self, req):
        
        
        rospy.logwarn("inside MPC")
        # Prediction horizon
        N = 15
        # Define problem
        # Three inputs roll desired, pitch desired, thrust
        u = cp.Variable((3,N))
        # Eight states
        x = cp.Variable((8,N+1))
        
        cost = 0
        constraints = [x[:,0] == req.states]
        for k in range(0,N):
            cost += cp.quad_form(x[:,k] - req.reference, self.Q) + cp.quad_form(u[:,k] - req.input_ref, self.R) #+ cp.quad_form(u[:,k]-u[:,k-1], Rd)
            constraints += [x[:,k+1] == self.Ad@x[:,k] + self.Bd@u[:,k]]
            constraints += [self.umin <= u[:,k], u[:,k] <= self.umax]
            #constraints += [self.xmin <= x[:,k], x[:,k] <= self.xmax]

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve()

        #[u.value[:,0], x.value[:,0]]
        
        
        return mpcsrvResponse(u.value[:,0])



if __name__ == '__main__':
    multirotor = UAV_model()

