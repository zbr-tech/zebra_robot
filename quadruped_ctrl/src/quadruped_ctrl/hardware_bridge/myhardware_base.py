#!/usr/bin/env python


class MyHardwareBase:
    def send(self, joint_control):
        pass

    def get_data(self):
        pass

    def reset_robot(self):
        pass

    def check_mode(self):
        return None