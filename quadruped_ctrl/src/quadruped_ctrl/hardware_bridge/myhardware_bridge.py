#!/usr/bin/env python
import os
import rospy
from quadruped_ctrl.hardware_bridge.motor_serial import MotorSerial
from quadruped_ctrl.hardware_bridge.motor_can import MotorCan
from quadruped_ctrl.hardware_bridge.myhardware_base import MyHardwareBase

# [TODO] this is leg 1 mode. should be change to 4 leg mode
class MyHardwareBridge(MyHardwareBase):
    def __init__(self, communication_freq, ports, interface):
        self.LEGPERMODULE = 12  # [TODO]
        self.MOTOR_NUM = 12
        self.GR = 6
        self._leg_data = [0] * 2 * self.MOTOR_NUM
        self._imu_data = [0] * 10
        self._motor_modules = []
        timeout = 1.0 / communication_freq / self.MOTOR_NUM
        if interface == "serial":
            self._motor_modules = [MotorSerial(port, timeout) for port in ports]
        elif interface == "can":
            for port in ports:
                os.system("sudo ip link set {} type can bitrate 1000000".format(port))
                os.system("sudo ifconfig {} up".format(port))
                self._motor_modules.append(MotorCan(port, timeout))

    def communicate(self, joint_control):
        for i in range(12):
            id = i + 1
            p_des = joint_control.position[i]  # * self.GR
            v_des = joint_control.velocity[i]  # * self.GR
            kp = joint_control.kp[i]
            kd = joint_control.kd[i]
            i_ff = joint_control.effort[i] / self.GR

            module_id = i // self.LEGPERMODULE
            self._motor_modules[module_id].send_command(id, p_des, v_des, kp, kd, i_ff)
            ret = self._motor_modules[module_id].receive()
            if ret is not None:
                id, posi, vel, _ = ret
                i = id - 1
                if i < self.MOTOR_NUM:
                    self._leg_data[i] = posi  # / self.GR
                    self._leg_data[i + self.MOTOR_NUM] = vel  # / self.GR

    def reset_robot(self):
        for i in range(self.MOTOR_NUM):
            module_id = i // self.LEGPERMODULE
            self._motor_modules[module_id].enable_motor(i + 1)
            self._motor_modules[module_id].zero_position(i + 1)
        print("reset")

    def get_data(self):
        return self._imu_data, self._leg_data
