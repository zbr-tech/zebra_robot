#!/usr/bin/env python
import rospy
from quadruped_ctrl.hardware_bridge.myhardware_bridge import MyHardwareBridge
from zebra_msgs.msg import ZebraJointControl
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import os
import math
import time

class trajectory_controller:
    def __init__(self):
        rospy.init_node("trajectory_controller")
        rospy.Subscriber("joint_target_trajectory", JointTrajectoryPoint, self.msgCallback)
        self._joint_control_publisher = rospy.Publisher('joint_control', ZebraJointControl, queue_size=100)
        self._joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=100)

        joint_control = ZebraJointControl()
        joint_control.position = [0] * 12
        joint_control.velocity = [0] * 12
        joint_control.kp = [0] * 12
        joint_control.kd = [0] * 12
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
        self._receive_command_flag = True
        self._acceleration = 0.1    # avoid zero division
        self._velocity_max = 0.1    # avoid zero division
        self._target_position = 0
        self._initial_position = 0
        self._initial_time = 0
   
        communication_freq = 100
        self._hardware = MyHardwareBridge(communication_freq, ["can0"], "can")
        self._hardware.enable_all_joints()
        rospy.Timer(rospy.Duration(1.0 / communication_freq), self.controlCallback)
        rospy.spin()

    def msgCallback(self,data):
        print("received")
        self._receive_command_flag = True
        # Update params
        self._target_position = data.positions[0]
        self._velocity_max = data.velocities[0]
        self._acceleration = data.accelerations[0]
        self._joint_control.kp = [5.0] * 12 # Set kp and kd after sending command to joints
        self._joint_control.kd = [0.2] * 12

    def controlCallback(self, event):
        # Communicate with joints
        self._hardware.communicate(self._joint_control) # send data to joints
        imu, leg = self._hardware.get_data()            # receive data from joints

        # publish joint control command and joint state to topic
        self._joint_state.position = leg[:12]
        self._joint_state.velocity = leg[12:12*2]
        self._joint_state.effort = leg[12*2:12*3]
        self._joint_state_publisher.publish(self._joint_state)
        self._joint_control_publisher.publish(self._joint_control)


        # Initialize params
        if self._receive_command_flag is True:
            self._receive_command_flag = False
            self._initial_position = leg[4]
            self._initial_time = time.time()

        # Local params
        target_position_diff = self._target_position - self._initial_position
        time_now = time.time() - self._initial_time
        
        # Check if target position is negative or positive
        if target_position_diff < 0:
            self._acceleration = abs(self._acceleration) * -1.0
            self._velocity_max = abs(self._velocity_max) * -1.0

        # Check if the maximum velocity can be reached or not
        if pow(self._velocity_max,2) > self._acceleration * target_position_diff:
            time_mid = math.sqrt(target_position_diff / self._acceleration)
            time_end = time_mid * 2
        else :
            time_mid = self._velocity_max / self._acceleration
            time_end = time_mid + target_position_diff / self._velocity_max

        # Calculate velocity and posiion
        if (0 <= time_now) & (time_now <= time_mid):    # self._acceleration phase
            self._joint_control.velocity[4] = self._acceleration * time_now
            self._joint_control.position[4] = 1 / 2.0 * self._acceleration * pow(time_now , 2)  + self._initial_position
        elif (time_mid < time_now) & (time_now <= time_end - time_mid): # Constant Velocity phase
            self._joint_control.velocity[4] = self._velocity_max
            self._joint_control.position[4] = self._velocity_max * (time_now - time_mid / 2)  + self._initial_position
        elif (time_end - time_mid < time_now) & (time_now <= time_end): # Deacceleration phase
            self._joint_control.velocity[4] = self._acceleration * (time_end - time_now)
            self._joint_control.position[4] = target_position_diff - self._acceleration / 2.0 * pow((time_end - time_now),2)  + self._initial_position
        else:   # Reached target
            pass

if __name__ == "__main__":
    trajectory_controller()