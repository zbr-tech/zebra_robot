#!/usr/bin/env python
import rospy
from quadruped_ctrl.hardware_bridge.myhardware_bridge import MyHardwareBridge
from zebra_msgs.msg import ZebraJointControl
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import os
import math
import time
import rosparam

class joint_controller:
    def __init__(self):
        self.MOTOR_NUM = 1 # number of joints
        rospy.init_node("joint_controller")
        #rospy.Subscriber("set_servo_on", Bool, self.servoOnCallback)
        #rospy.Subscriber("set_joint_angle", ZebraJointControl, self.msgCallback)
        self._joint_control_publisher = rospy.Publisher('joint_control', ZebraJointControl, queue_size=100)
        self._joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=100)

        # joint control command
        joint_control = ZebraJointControl()
        joint_control.position = [0] * self.MOTOR_NUM
        joint_control.velocity = [0] * self.MOTOR_NUM
        joint_control.kp = [0] * self.MOTOR_NUM
        joint_control.kd = [0] * self.MOTOR_NUM
        joint_control.effort = [0] * self.MOTOR_NUM
        self._joint_control = joint_control

        # joint state feedback
        joint_state = JointState()
        joint_state.name = [""] * self.MOTOR_NUM
        joint_state.position = [0] * self.MOTOR_NUM
        joint_state.velocity = [0] * self.MOTOR_NUM
        joint_state.effort = [0] * self.MOTOR_NUM
        for i in range(self.MOTOR_NUM):
            joint_state.name[i] = rosparam.get_param("joint"+str(i+1)+"/name")
        self._joint_state = joint_state

        self._has_arrived = [True] * self.MOTOR_NUM
        self._acceleration = [0.1] * self.MOTOR_NUM    # avoid zero division
        self._velocity_max = [0.1] * self.MOTOR_NUM    # avoid zero division
        self._position = [0] * self.MOTOR_NUM
        self._position_old = [0] * self.MOTOR_NUM
        self._initial_position = [0] * self.MOTOR_NUM
        self._initial_time = [0] * self.MOTOR_NUM

        self._count = 0

        communication_freq = 500
        self._hardware = MyHardwareBridge(communication_freq, ["can0"], "can")
        self._hardware.enable_all_joints()
        rospy.Timer(rospy.Duration(1.0 / communication_freq), self.controlCallback)
        rospy.spin()
    
    def communicate(self):
        # communicate with joints
        self._hardware.communicate(self._joint_control) # send data to joints
        imu, leg = self._hardware.get_data()            # receive data from joints

        # publish joint control command and joint state to topic
        self._joint_state.position = leg[:self.MOTOR_NUM]
        self._joint_state.velocity = leg[self.MOTOR_NUM:self.MOTOR_NUM*2]
        self._joint_state.effort = leg[self.MOTOR_NUM*2:self.MOTOR_NUM*3]
        self._joint_state_publisher.publish(self._joint_state)
        self._joint_control_publisher.publish(self._joint_control)
    

    def safetyCheck(self):
        # joint state check for safety
        for i in range(self.MOTOR_NUM):
            error_flag = False
            if self._has_arrived[i] is False and abs(self._joint_state.position[i] - self._joint_control.position[i]) > 0.1:
                rospy.logerr_once("Joint "+ str(i+1) + " not following trajectory")
                error_flag = True
            if self._joint_state.position[i] > rosparam.get_param("joint"+str(i+1)+"/position/max") or self._joint_state.position[i] < rosparam.get_param("joint"+str(i+1)+"/position/min"):
                rospy.logerr_once("Joint "+ str(i+1) + " over position")
                error_flag = True
            if abs(self._joint_state.velocity[i]) > rosparam.get_param("joint"+str(i+1)+"/velocity/max"):
                rospy.logerr_once("Joint "+ str(i+1) + " over velocity")
                error_flag = True
            if abs(self._joint_state.effort[i]) > rosparam.get_param("joint"+str(i+1)+"/current"):
                rospy.logerr_once("Joint "+ str(i+1) + " over current")
                error_flag = True
            if error_flag is True:
                self._hardware.disable_all_joints()
                self._has_arrived =  [True] * self.MOTOR_NUM
    

    def setJointAngle(self):
        for i in range(self.MOTOR_NUM):
            # initialize params when target position is updated
            if self._position[i] is not self._position_old[i]:
                self._position_old[i] = self._position[i]
                self._has_arrived[i] = False
                self._initial_position[i] = self._joint_state.position[i]
                self._initial_time[i] = time.time()
                self._velocity_max[i] = rosparam.get_param("joint"+str(i+1)+"/velocity/default")
                self._acceleration[i] = rosparam.get_param("joint"+str(i+1)+"/acceleration/default")
                self._joint_control.kp[i] = rosparam.get_param("joint"+str(i+1)+"/kp/default")
                self._joint_control.kd[i] = rosparam.get_param("joint"+str(i+1)+"/kd/default")

            # calculte position and velocity when joint has not reached target position yet
            if self._has_arrived[i] is False:
                # local params
                position_diff = self._position[i] - self._initial_position[i]
                time_from_start = time.time() - self._initial_time[i]
                
                # check if target position is negative or positive
                if position_diff < 0:
                    self._acceleration[i] = abs(self._acceleration[i]) * -1.0
                    self._velocity_max[i] = abs(self._velocity_max[i]) * -1.0

                # Check if the maximum velocity can be reached or not
                if pow(self._velocity_max[i],2) > self._acceleration[i] * position_diff:
                    time_mid = math.sqrt(abs(position_diff / self._acceleration[i]))
                    time_end = time_mid * 2
                else :
                    time_mid = self._velocity_max[i] / self._acceleration[i]
                    time_end = time_mid + position_diff / self._velocity_max[i]

                # Calculate velocity and posiion
                if (0 <= time_from_start) & (time_from_start <= time_mid):    # self._acceleration phase
                    self._joint_control.velocity[i] = self._acceleration[i] * time_from_start
                    self._joint_control.position[i] = 1 / 2.0 * self._acceleration[i] * pow(time_from_start , 2)  + self._initial_position[i]
                elif (time_mid < time_from_start) & (time_from_start <= time_end - time_mid): # Constant Velocity phase
                    self._joint_control.velocity[i] = self._velocity_max[i]
                    self._joint_control.position[i] = self._velocity_max[i] * (time_from_start - time_mid / 2)  + self._initial_position[i]
                elif (time_end - time_mid < time_from_start) & (time_from_start <= time_end): # Deacceleration phase
                    self._joint_control.velocity[i] = self._acceleration[i] * (time_end - time_from_start)
                    self._joint_control.position[i] = position_diff - self._acceleration[i] / 2.0 * pow((time_end - time_from_start),2)  + self._initial_position[i]
                else:   # Reached target
                    self._has_arrived[i] = True
                    rospy.loginfo("Joint "+ str(i+1) + " has reached target position")
                    #rospy.loginfo("Target: "+ str(self._position[i]) + "  Actual: "+ str(self._joint_state.position[i]) )

    def calculateTorque(self):
        for i in range(self.MOTOR_NUM):
            mass = 0.35 # kg
            length = 0.3#0.301 # kg
            gravity = 9.80665
            self._joint_control.effort[i] = mass * gravity * length * math.sin(self._joint_state.position[i])

    def controlCallback(self, event):
        if self._count is 0:
            if self._has_arrived[0] is True:
                self._position[0] = 90.0 / 180.0 * 3.14
                self._count += 1
        else:
            if self._has_arrived[0] is True:
                self._position[0] = self._position[0] * -1.0
        self.communicate()
        self.safetyCheck()
        #self.setJointAngle()
        self.calculateTorque()
        

if __name__ == "__main__":
    joint_controller()