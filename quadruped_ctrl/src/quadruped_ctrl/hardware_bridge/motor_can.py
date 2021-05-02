#!/usr/bin/env python
import os
import can


def float_to_uint(x, x_min, x_max, bits):
    """
    Converts a float to an unsigned int, given range and number of bits
    """
    span = x_max - x_min
    offset = x_min
    return int((x - offset) * (float((1 << bits) - 1)) / span)


def uint_to_float(x_int, x_min, x_max, bits):
    """
    converts unsigned int to float, given range and number of bits
    """
    # x_int = int(x_int)
    span = x_max - x_min
    offset = x_min
    return float(x_int) * span / (float((1 << bits) - 1)) + offset


class MotorCan:
    def __init__(self, port, timeout):
        os.system("sudo ip link set {} type can bitrate 1000000".format(port))
        os.system("sudo ifconfig {} up".format(port))
        self._can0 = can.interface.Bus(channel=port, bustype="socketcan_ctypes")
        # try:
        self._timeout = timeout
        self.P_MIN = -95.5
        self.P_MAX = 95.5
        self.V_MIN = -45.0
        self.V_MAX = 45.0
        self.KP_MIN = 0.0
        self.KP_MAX = 500.0
        self.KD_MIN = 0.0
        self.KD_MAX = 5.0
        self.I_MIN = -18.0
        self.I_MAX = 18.0

    def receive(self):
        msg = self._can0.recv(self._timeout)
        if msg is None:
            return None
        id = int(msg.data[0])
        p_int = (msg.data[1] << 8) | msg.data[2]
        v_int = (msg.data[3] << 4) | (msg.data[4] >> 4)
        i_int = ((msg.data[4] & 0xF) << 8) | msg.data[5]
        position = uint_to_float(p_int, self.P_MIN, self.P_MAX, 16)
        velocity = uint_to_float(v_int, self.V_MIN, self.V_MAX, 12)
        current = uint_to_float(i_int, self.I_MIN, self.I_MAX, 12)
        return id, position, velocity, current

    def enable_motor(self, id):
        msg = can.Message(
            arbitration_id=id,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
            extended_id=False,
        )
        self._can0.send(msg)

    def disable_motor(self, id):
        msg = can.Message(
            arbitration_id=id,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
            extended_id=False,
        )
        self._can0.send(msg)

    def zero_position(self, id):
        msg = can.Message(
            arbitration_id=id,
            data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE],
            extended_id=False,
        )
        self._can0.send(msg)

    def send_command(self, id, p_des, v_des, kp, kd, i_ff):
        data = [0] * 8
        p_int = float_to_uint(p_des, self.P_MIN, self.P_MAX, 16)
        v_int = float_to_uint(v_des, self.V_MIN, self.V_MAX, 12)
        kp_int = float_to_uint(kp, self.KP_MIN, self.KP_MAX, 12)
        kd_int = float_to_uint(kd, self.KD_MIN, self.KD_MAX, 12)
        t_int = float_to_uint(i_ff, self.I_MIN, self.I_MAX, 12)
        data[0] = p_int >> 8
        data[1] = p_int & 0xFF
        data[2] = v_int >> 4
        data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
        data[4] = kp_int & 0xFF
        data[5] = kd_int >> 4
        data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
        data[7] = t_int & 0xFF
        msg = can.Message(arbitration_id=id, data=data, extended_id=False)
        self._can0.send(msg)