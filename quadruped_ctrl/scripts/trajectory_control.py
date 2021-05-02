#!/usr/bin/env python
import rospy
from quadruped_ctrl.hardware_bridge.myhardware_bridge import MyHardwareBridge
from zebra_msgs.msg import ZebraJointControl
from sensor_msgs.msg import JointState
import os
import math

class test:
    def __init__(self):
        rospy.init_node("trajectory_controller")
        self._joint_control_publisher = rospy.Publisher('joint_control', ZebraJointControl, queue_size=100)
        self._joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=100)

        joint_control = ZebraJointControl()
        joint_control.position = [0] * 12
        joint_control.velocity = [0] * 12
        joint_control.kp = [10] * 12
        joint_control.kd = [0.1] * 12
        joint_control.effort = [0] * 12

        joint_state = JointState()
        joint_state.name = ["front_right_leg/joint1","front_right_leg/joint2","front_right_leg/joint3",
                            "front_left_leg/joint1","front_left_leg/joint2","front_left_leg/joint3",
                            "hind_left_leg/joint1","hind_left_leg/joint2","hind_left_leg/joint3",
                            "hind_right_leg/joint1","hind_right_leg/joint2","hind_right_leg/joint3"]
        joint_state.position = [0] * 12
        joint_state.velocity = [0] * 12
        joint_state.effort = [0] * 12

        self._joint_control = joint_control
        self._joint_state = joint_state

        self._communication_freq = 100
        self._count = 0
        self._joint_ready_flag = 0
        self._joint_limit_min = 0

        self._hardware = MyHardwareBridge(self._communication_freq, ["can0"], "can")
        self._hardware.enable_all_joints()
        rospy.Timer(rospy.Duration(1.0 / self._communication_freq), self.controlCallback)
        rospy.spin()
        

    def controlCallback(self, event):

        loop_period = 2 # cosine wave period (sec)
        joint_limit_max = math.pi * 0.5 # max limit of jonit (rad)
        joint_limit_min = math.pi * -0.5 # min limit of jonit (rad)
        
        self._hardware.communicate(self._joint_control) # send data
        imu, leg = self._hardware.get_data()            # receive data

        # organize leg state data
        self._joint_state.position = leg[:12]
        self._joint_state.velocity = leg[12:12*2]
        self._joint_state.effort = leg[12*2:12*3]

        # publish joint control command and joint state to ros topic
        self._joint_control_publisher.publish(self._joint_control)
        self._joint_state_publisher.publish(self._joint_state)

        if self._joint_ready_flag <= 20: #wait until the values are stable
            self._joint_ready_flag += 1
            self._joint_limit_min = leg[3]
        else:
            if  self._count < self._communication_freq * loop_period:
                self._count += 1
            else:
                self._count = 0
            if self._count > self._communication_freq * loop_period / 2:
                self._joint_limit_min = joint_limit_min
            self._joint_control.position[3] = (joint_limit_max - self._joint_limit_min) / 2.0 * math.cos(2 * math.pi *(self._count / float(self._communication_freq *loop_period)) + math.pi) +(joint_limit_max + self._joint_limit_min) / 2.0


if __name__ == "__main__":
    test()