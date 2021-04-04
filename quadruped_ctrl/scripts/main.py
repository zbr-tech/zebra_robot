#!/usr/bin/env python

import os
import numpy
import threading
import ctypes
import copy
import rospy
import rospkg
from geometry_msgs.msg import Twist
from quadruped_ctrl.srv import QuadrupedCmd, QuadrupedCmdResponse
from quadruped_ctrl.ctype_utils import StructPointer, ZebraPointer, convert_type
from quadruped_ctrl.hardware_bridge.myhardware_sim import MyHardwareSim
from quadruped_ctrl.hardware_bridge.myhardware_dual import MyHardwareDual


class GameManager:
    def make_cpp_gait_ctrller(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path("quadruped_ctrl")
        so_file = path.replace(
            "src/zebra_robot/quadruped_ctrl", "devel/lib/libquadruped_ctrl.so"
        )
        if not os.path.exists(so_file):
            so_file = path.replace(
                "src/zebra_robot/quadruped_ctrl", "build/lib/libquadruped_ctrl.so"
            )
        if not os.path.exists(so_file):
            rospy.logerr("cannot find cpp.so file")
        else:
            rospy.loginfo("find so file = " + so_file)
        cpp_gait_ctrller = ctypes.cdll.LoadLibrary(so_file)
        cpp_gait_ctrller.toque_calculator.restype = ctypes.POINTER(StructPointer)
        # add by shimizu
        cpp_gait_ctrller.get_zebra_joint_control.restype = ctypes.POINTER(ZebraPointer)
        return cpp_gait_ctrller

    def init(self):
        rospy.init_node("quadruped_simulator", anonymous=True)
        # get params
        self._hardware_freq = rospy.get_param("/hardware/hardware_freq")
        self._hardware_bridge_mode = rospy.get_param("/hardware/hardware_bridge_mode")
        self._communication_freq = rospy.get_param("/hardware/communication_freq")
        position_control_mode = rospy.get_param("/hardware/use_position_control")
        self._motor_kp = rospy.get_param("/hardware/kp")
        self._motor_kd = rospy.get_param("/hardware/kd")
        ###

        self._cpp_gait_ctrller = self.make_cpp_gait_ctrller()
        if self._hardware_bridge_mode == "simulator":
            self._hardware = MyHardwareSim(
                self._hardware_freq, self._communication_freq, position_control_mode
            )
        elif self._hardware_bridge_mode == "dual":
            self._hardware = MyHardwareDual(
                self._hardware_freq, self._communication_freq, position_control_mode
            )
        self.reset_robot()

        add_thread = threading.Thread(target=self.thread_job)
        add_thread.start()
        rospy.Service("gait_type", QuadrupedCmd, self.callback_gait)
        rospy.Service("robot_mode", QuadrupedCmd, self.callback_mode)
        rospy.Subscriber("cmd_vel", Twist, self.callback_body_vel, buff_size=10000)

    def cal_output(self, imu_data, leg_data):
        tau = self._cpp_gait_ctrller.toque_calculator(
            convert_type(imu_data), convert_type(leg_data)
        )
        joint_pointer = self._cpp_gait_ctrller.get_zebra_joint_control()

        joint_control = copy.deepcopy(joint_pointer.contents)
        for i in range(12):
            # joint_control.position[i] = joint_pointer.contents.position[i]
            # joint_control.velocity[i] = joint_pointer.contents.velocity[i]
            joint_control.kp[i] = (
                self._motor_kp if joint_pointer.contents.kp[i] > 0 else 0
            )
            joint_control.kd[i] = (
                self._motor_kd if joint_pointer.contents.kd[i] > 0 else 0
            )
        return joint_control

    # def run_one_step(self):
    #     imu_data, leg_data, base_pos = self._hardware.get_data()
    #     # print(imu_data)
    #     # raw_input()
    #     # if self._skip_count % self._skip_num == 0:
    #     # stamp_nsec = rospy.Time.now().to_nsec()
    #     joint_control = self.cal_output(imu_data, leg_data)
    #     # self._hardware.set_joint_control(joint_control)
    #     # print("t[ms]: ", (rospy.Time.now().to_nsec() - stamp_nsec) / 1000000)
    #     if self._use_simulator:
    #         self.safty_check()
    #     self._hardware.send(joint_control)

    def safty_check(self):
        check = self._hardware.check_mode()
        if check is None:
            return
        if check["reset"]:
            self.reset_robot()
        if check["low_flag"]:
            self._cpp_gait_ctrller.set_robot_mode(convert_type(1))
        if check["high_flag"]:
            self._cpp_gait_ctrller.set_robot_mode(convert_type(0))

    def reset_robot(self):
        # self._skip_count = 0
        self._hardware.reset_robot()
        # if self._use_simulator:
        stand_kp, stand_kd, joint_kp, joint_kd = 100.0, 1.0, 0.0, 0.05
        self._cpp_gait_ctrller.init_controller(
            convert_type(self._communication_freq),
            convert_type([stand_kp, stand_kd, joint_kp, joint_kd]),
        )
        for _ in range(10):
            imu_data, leg_data = self._hardware.get_data()
            self._cpp_gait_ctrller.pre_work(
                convert_type(imu_data), convert_type(leg_data)
            )
        # self._cpp_gait_ctrller.set_robot_mode(convert_type(1))
        # for _ in range(200):
        #     self.run_one_step()
        self._cpp_gait_ctrller.set_robot_mode(convert_type(0))

    def main(self):
        rate = rospy.Rate(self._communication_freq)  # hz
        while not rospy.is_shutdown():
            imu_data, leg_data = self._hardware.get_data()
            joint_control = self.cal_output(imu_data, leg_data)
            self.safty_check()  # [TODO] not need if use simulator
            self._hardware.send(joint_control)
            rate.sleep()

    def thread_job(self):
        rospy.spin()

    def callback_gait(self, req):
        self._cpp_gait_ctrller.set_gait_type(convert_type(req.cmd))
        return QuadrupedCmdResponse(0, "get the gait")

    def callback_mode(self, req):
        self._cpp_gait_ctrller.set_robot_mode(convert_type(req.cmd))
        return QuadrupedCmdResponse(0, "get the mode")

    def callback_body_vel(self, msg):
        vel = [msg.linear.x, msg.linear.y, msg.angular.x]
        self._cpp_gait_ctrller.set_robot_vel(convert_type(vel))


if __name__ == "__main__":
    game_manager = GameManager()
    game_manager.init()
    game_manager.main()
