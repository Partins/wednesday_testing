#!/usr/bin/env python3

import rospy


from rosflight_msgs.msg import Command, RCRaw, Status
from rosflight_extras.srv import arm_uav

# Parameters
RATE = 20 # Rate at which to send commands to UAV when arming, disarming etc.

# PWM values
STICK_LOW   = 1000
STICK_MID   = 1500
STICK_HIGH  = 2000
SWITCH_LOW  = 1000
SWITCH_HIGH = 2000

# Channels
ROLL_CHN    = 0
PITCH_CHN   = 1
THR_CHN     = 2
YAW_CHN     = 3
AUX1        = 4 # RC_OVR
AUX2        = 5
AUX3        = 6
AUX4        = 7 # ARM

class ROSFLIGHT_ARMING:

    def __init__(self):

        rospy.init_node('rosflight_arming', anonymous=True)
        self.msg_raw = RCRaw()  
        self.msg = Command()
        self.rate = rospy.Rate(RATE) # [Hz]
        self.is_armed = False
        self.is_rc_override = False

        self.sub_status = rospy.Subscriber('status', Status, self.get_status)
        self.pub_raw = rospy.Publisher('multirotor/RC', RCRaw, queue_size=1)

        self.arming_service = rospy.Service('arm_UAV', arm_uav, self.arm_fun)
        self.pc_control_Service = rospy.Service('pc_control', arm_uav, self.enable_computer_control)


        rospy.spin()
    # Gets status if UAV is armed or not. Requires rosflight_io to run
    def get_status(self, msg):
        self.is_armed = msg.armed
        self.is_rc_override = msg.rc_override
    
    # Arming the UAV
    # Handles both arming and disarming
    def arm_fun(self, req):
        rospy.logwarn("Service started")
        self.msg_raw.header.stamp        = rospy.Time.now()
        self.msg_raw.values[ROLL_CHN]    = STICK_MID  
        self.msg_raw.values[PITCH_CHN]   = STICK_MID
        self.msg_raw.values[THR_CHN]     = STICK_LOW
        self.msg_raw.values[YAW_CHN]     = STICK_MID
        self.msg_raw.values[AUX1]        = SWITCH_HIGH
        self.msg_raw.values[AUX2]        = SWITCH_LOW
        self.msg_raw.values[AUX3]        = SWITCH_LOW

        if req.arm is True:
            rospy.logwarn("ARMING")
            self.msg_raw.values[AUX4]    = SWITCH_HIGH
        else:
            rospy.logwarn("DISARMING")
            self.msg_raw.values[AUX4]    = SWITCH_LOW


        arming_tries = 0
        while arming_tries < 20:
            self.pub_raw.publish(self.msg_raw)
            arming_tries += 1
            self.rate.sleep()
        
        return self.is_armed

    # enable_computer_control assumes armed. Default timeout which mean command
    # to /command have to be send with a rate > 10 HZ.
    def enable_computer_control(self, req): 

            self.msg_raw.header.stamp        = rospy.Time.now()
            self.msg_raw.values[ROLL_CHN]    = STICK_MID  
            self.msg_raw.values[PITCH_CHN]   = STICK_MID
            self.msg_raw.values[THR_CHN]     = STICK_LOW
            self.msg_raw.values[YAW_CHN]     = STICK_MID
            self.msg_raw.values[AUX1]        = SWITCH_LOW
            self.msg_raw.values[AUX2]        = SWITCH_LOW
            self.msg_raw.values[AUX3]        = SWITCH_LOW

            if req.arm is True:
                rospy.logwarn("Enabling Computer Control")
                self.msg_raw.values[AUX4]    = SWITCH_HIGH
            else:
                rospy.logwarn("Disabling Computer Control")
                self.msg_raw.values[AUX4]    = SWITCH_LOW

            for i in range(20):
                self.pub_raw.publish(self.msg_raw)
                self.rate.sleep()

            return self.is_rc_override



if __name__ == '__main__':
    try:
        multirotor = ROSFLIGHT_ARMING()

    except rospy.ROSInterruptException:
        pass
