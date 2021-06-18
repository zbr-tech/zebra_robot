#!/usr/bin/env python
import os
import rospy
from quadruped_ctrl.hardware_bridge.motor_serial import MotorSerial
from quadruped_ctrl.hardware_bridge.motor_can import MotorCan
from quadruped_ctrl.hardware_bridge.myhardware_base import MyHardwareBase

# [TODO] this is leg 1 mode. should be change to 4 leg mode
class MyHardwareBridge(MyHardwareBase):
    def __init__(self, communication_freq, ports, interface):
        self.LEGPERMODULE = 1  # [TODO]
        self.MOTOR_NUM = 1
        self.GR = 6
        self._leg_data = [0] * 3 * self.MOTOR_NUM
        self._imu_data = [0] * 10
        self._motor_modules = []

        timeout = 1.0 / communication_freq / self.MOTOR_NUM
        if interface == "serial":
            self._motor_modules = [MotorSerial(port, timeout) for port in ports]
        elif interface == "can":
            for port in ports:
                try:
                    self._motor_modules.append(MotorCan(port, timeout))
                    rospy.loginfo("Connected to port " + port)
                except:
                    rospy.logerr("Port "+ port + " not found")

    def communicate(self, joint_control):
        for i in range(self.LEGPERMODULE):
            id = i + 1
            p_des = joint_control.position[i] / self.GR  * self.GR * -1.0 # really?
            v_des = joint_control.velocity[i] / self.GR  * self.GR * -1.0 # really?
            kp = joint_control.kp[i]
            kd = joint_control.kd[i]
            i_ff = joint_control.effort[i] / self.GR * self.GR * -1.0 # really?

            module_id = i // self.LEGPERMODULE
            self._motor_modules[module_id].send_command(id, p_des, v_des, kp, kd, i_ff)
            ret = self._motor_modules[module_id].receive()
            if ret is not None:
                id, posi, vel,current= ret
                i = id - 1
                if i < self.MOTOR_NUM:
                    self._leg_data[i] = posi * self.GR  / self.GR * -1.0 # really?
                    self._leg_data[i + self.MOTOR_NUM * 1] = vel * self.GR  / self.GR * -1.0 # really?
                    self._leg_data[i + self.MOTOR_NUM * 2] = current * -1.0 # really?
            else:
                #rospy.logerr("Joint " + str(i) + " not connected")
                pass

    def reset_robot(self):
        for i in range(self.MOTOR_NUM):
            module_id = i // self.LEGPERMODULE
            self._motor_modules[module_id].enable_motor(i + 1)
            self._motor_modules[module_id].zero_position(i + 1)
        rospy.loginfo("Reset")

    def enable_all_joints(self):
        for i in range(self.MOTOR_NUM):
            module_id = i // self.LEGPERMODULE
            self._motor_modules[module_id].enable_motor(i + 1)
        rospy.loginfo("Enable all joints")

    def disable_all_joints(self):
        for i in range(self.MOTOR_NUM):
            module_id = i // self.LEGPERMODULE
            self._motor_modules[module_id].disable_motor(i + 1)
        rospy.loginfo("Disable all joints")

    def get_data(self):
        return self._imu_data, self._leg_data