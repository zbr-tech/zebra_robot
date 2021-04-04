#!/usr/bin/python
from quadruped_ctrl.hardware_bridge.myhardware_bridge_serial import (
    MyHardwareBridgeSerial,
)
from quadruped_ctrl.hardware_bridge.myhardware_sim import MyHardwareSim


class MyHardwareDual:
    def __init__(self, sim_freq, communication_freq, position_control_mode):
        self._sim = MyHardwareSim(sim_freq, communication_freq, position_control_mode)
        self._serial = MyHardwareBridgeSerial(communication_freq)

    def reset_robot(self):
        self._sim.reset_robot()
        self._serial.reset_robot()

    def send(self, joint_control):
        self._sim.send(joint_control)
        self._serial.send(joint_control)

    def get_data(self):
        ret_sim = self._sim.get_data()
        ret_serial = self._serial.get_data()
        print("ret sim   : ", ret_sim[1][0], ret_sim[1][12])
        print("ret serial: ", ret_serial[1][0], ret_serial[1][12])
        return ret_sim

    def check_mode(self):
        return self._sim.check_mode()
