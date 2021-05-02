#!/usr/bin/env python
import rospy
from quadruped_ctrl.hardware_bridge.myhardware_bridge import MyHardwareBridge
from zebra_msgs.msg import ZebraJointControl
import os


class test:
    def __init__(self):
        rospy.init_node("joint_control")
        communication_freq = 100
        self._hardware = MyHardwareBridge(communication_freq, ["can0"], "can")
        joint_control = ZebraJointControl()
        joint_control.position = [0] * 12
        joint_control.velocity = [0] * 12
        joint_control.kp = [5] * 12
        joint_control.kd = [0] * 12
        joint_control.effort = [0] * 12
        self._joint_control = joint_control
        self._add = 0.1
        self._hardware.reset_robot()
        rospy.Timer(rospy.Duration(1.0 / communication_freq), self.controlCallback)
        rospy.spin()

    def controlCallback(self, event):
        target = self._joint_control.position[4]
        target += self._add
        if target > 12 or target < 0:
            self._add *= -1
        self._joint_control.position[4] = target
        self._hardware.communicate(self._joint_control)
        imu, leg = self._hardware.get_data()
        print("des: ", self._joint_control.position[4])
        print("ret: ", leg[4])


if __name__ == "__main__":
    test()