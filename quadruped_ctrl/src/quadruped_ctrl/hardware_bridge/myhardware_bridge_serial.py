#!/usr/bin/env python
import rospy
from quadruped_ctrl.hardware_bridge.motor_serial import MotorSerial
from quadruped_ctrl.hardware_bridge.myhardware_base import MyHardwareBase

# [TODO] this is leg 1 mode. should be change to 4 leg mode
class MyHardwareBridgeSerial(MyHardwareBase):
    def __init__(self, communication_freq):
        ports = ["/dev/ttyACM0"]  # [TODO]
        self.LEGPERMODULE = 12  # [TODO]
        self.MOTOR_NUM = 12
        self.GR = 6
        timeout = 1.0 / communication_freq / self.MOTOR_NUM
        self._motor_modules = [MotorSerial(port, timeout) for port in ports]
        self._leg_data = [0] * 2 * self.MOTOR_NUM
        self._imu_data = [0] * 10

    def send(self, joint_control):
        for i in range(12):
            id = i + 1
            p_des = joint_control.position[i] * self.GR
            v_des = joint_control.velocity[i] * self.GR
            kp = joint_control.kp[i]
            kd = joint_control.kd[i]
            i_ff = joint_control.effort[i] / self.GR

            module_id = i // self.LEGPERMODULE
            self._motor_modules[module_id].send_command(id, p_des, v_des, kp, kd, i_ff)
            ret = self._motor_modules[module_id].receive()
            if ret is not None:
                id, posi, vel, _ = ret
                i = id - 1
                if i < 12:
                    self._leg_data[i] = posi / self.GR
                    self._leg_data[i + self.MOTOR_NUM] = vel / self.GR

    def reset_robot(self):
        for i in range(self.MOTOR_NUM):
            module_id = i // self.LEGPERMODULE
            self._motor_modules[module_id].enable_motor(1)
        print("enable")

    def get_data(self):
        return self._imu_data, self._leg_data