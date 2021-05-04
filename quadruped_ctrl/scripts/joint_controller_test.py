#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint

class test:
    def __init__(self):
        rospy.init_node('joint_controller_test')
        self._joint_target_publisher = rospy.Publisher('joint_trajectory_point', JointTrajectoryPoint, queue_size=10)
        joint_target = JointTrajectoryPoint()
        joint_target.positions = [3.14 * 1.0] * 12
        joint_target.velocities = [10] * 12
        joint_target.accelerations = [10] * 12
        self._joint_target = joint_target

        communication_freq = 0.4
        rospy.Timer(rospy.Duration(1.0 / communication_freq), self.controlCallback)
        rospy.spin()

    def controlCallback(self, event):
        for i in range(12):
            self._joint_target.positions[i] = self._joint_target.positions[i] * -1
        self._joint_target_publisher.publish(self._joint_target)

if __name__ == '__main__':
    test()