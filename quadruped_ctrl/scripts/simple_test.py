#!/usr/bin/env python
import rospy
from quadruped_ctrl.myhardware_bridge import MyHardwareBridge
from zebra_msgs.msg import ZebraJointControl


class test:
    def __init__(self):
        rospy.init_node("joint_control")
        communication_freq = 100
        self._hardware = MyHardwareBridge(communication_freq)
        self._hardware.reset_robot()
        joint_control = ZebraJointControl()
        joint_control.position = [0] * 12
        joint_control.velocity = [0] * 12
        joint_control.kp = [0] * 12
        joint_control.kd = [1] * 12
        joint_control.effort = [0] * 12
        joint_control.velocity[0] = 5
        self._joint_control = joint_control
        self._add = 0.01
        rospy.Timer(rospy.Duration(1.0 / communication_freq), self.controlCallback)
        rospy.spin()

    def controlCallback(self, event):
        self._joint_control.velocity[0] += self._add
        if self._joint_control.velocity[0] > 8 or self._joint_control.velocity[0] < 3:
            self._add *= -1
        self._hardware.send(self._joint_control)


if __name__ == "__main__":
    test()
"""
from quadruped_ctrl.motormodule import MotorModuleController

MOTOR_NUM = 1

# global params
enable_flag = 0
disable_flag = 0
target_rad = [0] * MOTOR_NUM
device = "/dev/ttyACM0"
# os.system(["sudo", "chmod", "666", device])
# os.system("sudo chmod 666 "+ device)
mmc = MotorModuleController(device, 0.5 / 10)  # master controller port name
count = 5
add = 0.01


def controlCallback(event):
    global count
    global add
    count += add
    if count > 8 or count < 3:
        add = -add
    print(count)
    for i in range(MOTOR_NUM):  # send target command and update joint state
        # target_rad[i] = coun
        mmc.send_command(
            i + 1, 0, count, 0, 1, 0.0
        )  # (id, position, velocity, kp, kd, torque)
        print(i + 1, 0, count, 0, 1, 0.0)


def main():
    rospy.init_node("joint_control")
    # rospy.sleep(1)
    mmc.enable_motor(1)
    rospy.Timer(rospy.Duration(1.0 / 10), controlCallback)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
"""