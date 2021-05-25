#!/usr/bin/env python3
import rospy
import time
import math
import numpy as np
from scipy.signal import cont2discrete as c2d
import cvxpy as cp
import matplotlib.pyplot as plt 


from rosflight_msgs.msg import Command, RCRaw, Status
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from apriltag_ros.msg import AprilTagDetectionArray
from rosflight_extras.srv import arm_uav
from martin_mpc.srv import mpcsrv
from rosflight_extras.msg import *


# PWM values if it was a real controller
STICK_LOW   = 1000
STICK_MID   = 1500
STICK_HIGH  = 2000
SWITCH_LOW  = 1000
SWITCH_HIGH = 2000

ROLL_CHN    = 0
PITCH_CHN   = 1
THR_CHN     = 2
YAW_CHN     = 3
AUX1        = 4 # RC_OVR
AUX2        = 5
AUX3        = 6
AUX4        = 7 # ARM

class UAV:

    def __init__(self):

        rospy.init_node('UAV_control', anonymous=True)
        self.msg_raw = RCRaw()  # 1st Config in arm()
        self.msg = Command()    # 1st Config in enable_computer_control()
        self.plot_msg = 0.0 # For plotting

        self.pub_raw = rospy.Publisher('multirotor/RC', RCRaw, queue_size=1)
        self.pub_command = rospy.Publisher('command', Command, queue_size=1)
        self.sub_status = rospy.Subscriber('status', Status, self.get_status)
        self.tag_pos = rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.get_tag)
        self.pub_plotter = rospy.Publisher('plotter', float_array, queue_size=1)
        self.set_state_service = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        self.arm_srv    = rospy.ServiceProxy('arm_UAV', arm_uav)
        self.pc_control = rospy.ServiceProxy('pc_control', arm_uav)
        self.mpc_calc   = rospy.ServiceProxy('MPC_calc', mpcsrv)
        self.x_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_model = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        #self.sub_state = rospy.Subscriber('gazebo/model_states', ModelStates, self.state_update)
        state = ModelState()
        state.model_name = 'multirotor'
        state.reference_frame = 'ground_plane'
        state.pose.position.x = -1.0
        state.pose.position.y = -2.0
        state.pose.position.z = 0.05
        # pose
        #quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)

        # yaw 90 deg: z=w=0.7068
        # yaw 180deg: z=1
        state.pose.orientation.x = 0
        state.pose.orientation.y = 0
        state.pose.orientation.z = 0
        state.pose.orientation.w = 0
        # twist
        state.twist.linear.x = 0
        state.twist.linear.y = 0
        state.twist.linear.z = 0
        state.twist.angular.x = 0
        state.twist.angular.y = 0
        state.twist.angular.z = 0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = self.set_state_service
            result = set_state(state)
            assert result.success is True
        except rospy.ServiceException:
            print("/gazebo/get_model_state service call failed") 

        #time.sleep(0.5)
        
        RATE = 20 # [Hz]
        self.rate = rospy.Rate(RATE)
        self.is_armed = False
        self.tot_error  = [0, 0, 0]       # Total error used for integral part
        self.prev_error = [0, 0, 0]     # Previous error used for derivative part
        self.tag_x = 0
        self.tag_y = 0
        self.pos = [0, 0, 0]

    # Notes: Have references as topic and not service
    # Landing: Geometrical contraints for landing
    def run(self):
        rospy.logwarn('ARMING...')
        self.arm_srv.call(True)
        rospy.logwarn("Enabling computer control")
        self.pc_control.call(True)
        self.pub_command.publish(self.msg) # Spamming to avoid timeout

        # Main loop
        self.xr = [0.0 ,0.0, 1.0, 0.0 ,0.0 ,0.0 ,0.0, 0.0]
        i = 0
        while not rospy.is_shutdown():
            i = i+1
            self.msg.z = 0
            if i >= 100:
                self.xr = [0.0 ,0.0, 2.0, 0.0 ,0.0 ,0.0 ,0.0, 0.0]
                #self.msg.z = 0.1
            
            #if i >= 200:
            #    self.xr = [2.0 ,4.0, 3.0, 0.0 ,0.0 ,0.0 ,0.0, 0.0]

            self.states = self.get_model('multirotor', 'ground_plane')
            self.x_states[0] = self.states.pose.position.x
            self.x_states[1] = self.states.pose.position.y
            self.x_states[2] = self.states.pose.position.z
            self.x_states[3] = self.states.twist.linear.x
            self.x_states[4] = self.states.twist.linear.y
            self.x_states[5] = self.states.twist.linear.z
            y = self.states.pose.orientation.y
            x = self.states.pose.orientation.x
            z = self.states.pose.orientation.z
            w = self.states.pose.orientation.w
#           
            tmp_eul = self.quaternion_to_euler_angle_vectorized1( \
                                            self.states.pose.orientation.w, \
                                            self.states.pose.orientation.x, \
                                            self.states.pose.orientation.y, \
                                            self.states.pose.orientation.z)
                                                        
            self.x_states[6] = tmp_eul[0]   # Roll
            self.x_states[7] = tmp_eul[1]   # Pitch
            #rospy.logwarn(self.x_states)
            if i >= 200:
                self.xr = [self.x_states[0] - self.tag_y , self.x_states[1] - self.tag_x, 2.0, 0.0 ,0.0 ,0.0 ,0.0, 0.0]
                #self.xr = [0,0, 1.50, 0.0 ,0.0 ,0.0 ,0.0, 0.0]
                
            resp1 = self.mpc_calc(self.xr, [0.0, 0.0, 9.8], self.x_states)
            #rospy.logwarn(resp1.control_signals[2]/9.8)
            #rospy.logwarn(self.xr - self.x_states)
            test_msg = float_array()
            test_msg.header.stamp = rospy.Time.now()
            test_msg.data = [self.tag_x, self.tag_y]
            self.msg.header.stamp = rospy.Time.now()
            self.pub_plotter.publish(test_msg)
            self.msg.mode = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE

            self.msg.x =  math.cos(-tmp_eul[2]) * resp1.control_signals[0] + math.sin(-tmp_eul[2]) * resp1.control_signals[1]
            self.msg.y = -math.sin(-tmp_eul[2]) * resp1.control_signals[0] + math.cos(-tmp_eul[2]) * resp1.control_signals[1]
            
            self.msg.F = resp1.control_signals[2]/9.8

            self.pub_command.publish(self.msg)
            self.rate.sleep()


    def quaternion_to_euler_angle_vectorized1(self, w, x, y, z):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z 
        

        
        
        
    def get_tag(self, msg):
        #rospy.logwarn(type(msg.detections[0].pose.pose.pose.position.x))
        try:
            self.tag_x = msg.detections[0].pose.pose.pose.position.x
            self.tag_y = msg.detections[0].pose.pose.pose.position.y
            #rospy.logwarn([msg.detections[0].pose.pose.pose.position.x, msg.detections[0].pose.pose.pose.position.y])
        except:
            pass
            #self.tag_x = -self.tag_x
            #self.tag_y = -self.tag_y
            
    def get_status(self, msg):
        self.is_armed = msg.armed
        self.is_rc_override = msg.rc_override

    # Subscriber callback. But I'm wondering about race conditions. Maybe good
    # to use ServiceProxy to get the states directly in the controller when 
    # needed. Will keep this here for now
    #def state_update(self, msg):
    #    
    #    self.x_states[0] = msg.pose[1].position.x
    #    self.x_states[1] = msg.pose[1].position.y
    #    self.x_states[2] = msg.pose[1].position.z
    #    self.x_states[3] = msg.twist[1].linear.x
    #    self.x_states[4] = msg.twist[1].linear.y
    #    self.x_states[5] = msg.twist[1].linear.z
#
    #    tmp_eul = self.quaternion_to_euler_angle_vectorized1( \
    #                                        msg.pose[1].orientation.w, \
    #                                        msg.pose[1].orientation.x, \
    #                                        msg.pose[1].orientation.y, \
    #                                        msg.pose[1].orientation.z)
    #    self.x_states[6] = tmp_eul[0]   # Roll
    #    self.x_states[7] = tmp_eul[1]   # Pitch                
    
if __name__ == '__main__':
    try:
        multirotor = UAV()
        multirotor.run()
    except rospy.ROSInterruptException:
        pass
        







