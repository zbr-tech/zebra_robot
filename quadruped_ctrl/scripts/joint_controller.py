#!/usr/bin/env python
import rospy
from quadruped_ctrl.hardware_bridge.myhardware_bridge import MyHardwareBridge
from zebra_msgs.msg import ZebraJointControl
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import os
import math
import time

class joint_controller:
    def __init__(self):
        self.MOTOR_NUM = 12 # number of joints
        rospy.init_node("joint_controller")
        rospy.Subscriber("joint_trajectory_point", JointTrajectoryPoint, self.msgCallback)
        self._joint_control_publisher = rospy.Publisher('joint_control', ZebraJointControl, queue_size=100)
        self._joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=100)

        joint_control = ZebraJointControl()
        joint_control.position = [0] * self.MOTOR_NUM
        joint_control.velocity = [0] * self.MOTOR_NUM
        joint_control.kp = [0] * self.MOTOR_NUM
        joint_control.kd = [0] * self.MOTOR_NUM
        joint_control.effort = [0] * self.MOTOR_NUM

        joint_state = JointState()
        joint_state.name = ["front_right_leg/joint1","front_right_leg/joint2","front_right_leg/joint3",
                            "front_left_leg/joint1","front_left_leg/joint2","front_left_leg/joint3",
                            "hind_left_leg/joint1","hind_left_leg/joint2","hind_left_leg/joint3",
                            "hind_right_leg/joint1","hind_right_leg/joint2","hind_right_leg/joint3"]
        joint_state.position = [0] * self.MOTOR_NUM
        joint_state.velocity = [0] * self.MOTOR_NUM
        joint_state.effort = [0] * self.MOTOR_NUM

        self._joint_control = joint_control
        self._joint_state = joint_state
        self._has_received = [False] * self.MOTOR_NUM
        self._has_arrived = [True] * self.MOTOR_NUM
        self._acceleration = [0.1] * self.MOTOR_NUM    # avoid zero division
        self._velocity_max = [0.1] * self.MOTOR_NUM    # avoid zero division
        self._position = [0] * self.MOTOR_NUM
        self._initial_position = [0] * self.MOTOR_NUM
        self._initial_time = [0] * self.MOTOR_NUM
   
        communication_freq = 500
        self._hardware = MyHardwareBridge(communication_freq, ["can0"], "can")
        self._hardware.enable_all_joints()
        rospy.Timer(rospy.Duration(1.0 / communication_freq), self.controlCallback)
        rospy.spin()

    def msgCallback(self,data):
        # update params
        for i in range(self.MOTOR_NUM):
            if self._has_arrived[i] is True:
                rospy.logdebug("Joint "+ str(i+1) + " received command. Updated target position")
                self._has_received[i] = True
                self._position[i] = data.positions[i]
                self._velocity_max[i] = data.velocities[i]
                self._acceleration[i] = data.accelerations[i]
                self._joint_control.kp[i] = 20.0  # set kp and kd after sending command to joints
                self._joint_control.kd[i] = 0.2 
            else:
                rospy.logwarn("Joint "+ str(i+1) + " received command but ignored")

    def controlCallback(self, event):
        # communicate with joints
        self._hardware.communicate(self._joint_control) # send data to joints
        imu, leg = self._hardware.get_data()            # receive data from joints

        # publish joint control command and joint state to topic
        self._joint_state.position = leg[:self.MOTOR_NUM]
        self._joint_state.velocity = leg[self.MOTOR_NUM:self.MOTOR_NUM*2]
        self._joint_state.effort = leg[self.MOTOR_NUM*2:self.MOTOR_NUM*3]
        self._joint_state_publisher.publish(self._joint_state)
        self._joint_control_publisher.publish(self._joint_control)

        for i in range(self.MOTOR_NUM):
            # initialize params
            if self._has_received[i] is True:   # when received command from topic
                self._has_received[i] = False
                self._has_arrived[i] = False
                self._initial_position[i] = leg[i]
                self._initial_time[i] = time.time()
            else:   # check if joint follows the trajectory
                if abs(self._joint_control.position[i] - leg[i]) > 0.1:
                    rospy.logwarn("Joint "+ str(i+1) + " is not following the trajectory")

            # local params
            position_diff = self._position[i] - self._initial_position[i]
            time_diff = time.time() - self._initial_time[i]
            
            # check if target position is negative or positive
            if position_diff < 0:
                self._acceleration[i] = abs(self._acceleration[i]) * -1.0
                self._velocity_max[i] = abs(self._velocity_max[i]) * -1.0

            # Check if the maximum velocity can be reached or not
            if pow(self._velocity_max[i],2) > self._acceleration[i] * position_diff:
                time_mid = math.sqrt(position_diff / self._acceleration[i])
                time_end = time_mid * 2
            else :
                time_mid = self._velocity_max[i] / self._acceleration[i]
                time_end = time_mid + position_diff / self._velocity_max[i]

            # Calculate velocity and posiion
            if (0 <= time_diff) & (time_diff <= time_mid):    # self._acceleration phase
                self._joint_control.velocity[i] = self._acceleration[i] * time_diff
                self._joint_control.position[i] = 1 / 2.0 * self._acceleration[i] * pow(time_diff , 2)  + self._initial_position[i]
            elif (time_mid < time_diff) & (time_diff <= time_end - time_mid): # Constant Velocity phase
                self._joint_control.velocity[i] = self._velocity_max[i]
                self._joint_control.position[i] = self._velocity_max[i] * (time_diff - time_mid / 2)  + self._initial_position[i]
            elif (time_end - time_mid < time_diff) & (time_diff <= time_end): # Deacceleration phase
                self._joint_control.velocity[i] = self._acceleration[i] * (time_end - time_diff)
                self._joint_control.position[i] = position_diff - self._acceleration[i] / 2.0 * pow((time_end - time_diff),2)  + self._initial_position[i]
            else:   # Reached target
                self._has_arrived[i] = True

if __name__ == "__main__":
    joint_controller()