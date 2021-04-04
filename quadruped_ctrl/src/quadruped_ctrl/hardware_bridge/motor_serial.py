#!/usr/bin/env python
"""
Ben Katz
Motor Module Python API
Assumes the serial device is a nucleo running the firmware at:
Corresponding STM32F446 Firmware here:
https://os.mbed.com/users/benkatz/code/CanMaster/
"""
import serial
from struct import unpack, pack


class MotorSerial:
    def __init__(self, port, timeout):
        # try:
        self.ser = serial.Serial(port, timeout=timeout)
        self.ser.baudrate = 921600
        print("connected " + port)

    def receive(self):
        b_rx = self.ser.read(13)
        if len(b_rx) < 13:
            ret = None
        else:
            ret = (
                ord(b_rx[0][0]),
                unpack("f", b_rx[1:5])[0],
                unpack("f", b_rx[5:9])[0],
                unpack("f", b_rx[9:13])[0],
            )  # id, position, velocity, effort
        return ret

    def enable_motor(self, id):
        """
        Puts motor with CAN ID "id" into torque-control mode.  2nd red LED will turn on
        """
        b = b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC"
        b = b + bytes(bytearray([id]))
        self.ser.write(b)
        # self.ser.flushInput()

    def disable_motor(self, id):
        """
        Removes motor with CAN ID "id" from torque-control mode.  2nd red LED will turn off
        """
        b = b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD"
        b = b + bytes(bytearray([id]))
        self.ser.write(b)

    def send_command(self, id, p_des, v_des, kp, kd, i_ff):
        """
        send_command(desired position, desired velocity, position gain, velocity gain, feed-forward current)

        Sends data over CAN, reads response, and populates rx_data with the response.
        """
        id = int(id)
        b = (
            bytes(bytearray([id]))
            + pack("f", p_des)
            + pack("f", v_des)
            + pack("f", kp)
            + pack("f", kd)
            + pack("f", i_ff)
        )
        # print(int.from_bytes(b, byteorder="big"))
        self.ser.write(b)