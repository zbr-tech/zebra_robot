#! /usr/bin/env python
# import os
# import can


class MyHardwareBridge:
    def init(self, bitrate):
        # os.system('sudo ip link set can0 type can bitrate {}'.format(bitrate))
        # os.system('sudo ifconfig can0 up')

        # can0 = can.interface.Bus(channel='can0', bustype='socketcan_ctypes')

        # while True:
        #     msg = can0.recv(30.0)
        #     print(msg)
        #     if msg is None:
        #         print('No message was received')

        # os.system('sudo ifconfig can0 down')
        pass

    def reset_robot(self):
        pass

    def get_data(self):
        pass


if __name__ == "__main__":
    a = MyHardwareBridge()
    a.init()
