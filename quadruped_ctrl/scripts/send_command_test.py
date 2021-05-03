#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint

class test:
    def __init__(self):
        rospy.init_node('send_command_test')
        self._joint_target_publisher = rospy.Publisher('joint_target_trajectory', JointTrajectoryPoint, queue_size=10)
        joint_target = JointTrajectoryPoint()
        joint_target.positions = [3.14*2] * 1
        joint_target.velocities = [5] * 1
        joint_target.accelerations = [5] * 1
        self.joint_target = joint_target


        self._joint_target_publisher.publish(self.joint_target)
        print("published")

        communication_freq = 1
        rospy.Timer(rospy.Duration(1.0 / communication_freq), self.controlCallback)
        rospy.spin()

    def controlCallback(self, event):
        self._joint_target_publisher.publish(self.joint_target)
        print("published")
        pass

if __name__ == '__main__':
    test()