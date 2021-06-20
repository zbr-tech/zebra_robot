#!/usr/bin/env python
import rospy
import math
from zebra_msgs.msg import ZebraJointControl
from std_msgs.msg import Bool
class test:
    def __init__(self):
        self.MOTOR_NUM = 1 # number of joints
        rospy.init_node('joint_controller_test')
        self._servo_on_publisher = rospy.Publisher('set_servo_on', Bool, queue_size=10)
        self._joint_target_publisher = rospy.Publisher('set_joint_angle', ZebraJointControl, queue_size=10)
        
        joint_target = ZebraJointControl()
        joint_target.position = [math.pi * 0.5] * self.MOTOR_NUM    # target position
        joint_target.velocity = [10.0] * self.MOTOR_NUM             # velocity limit
        joint_target.effort = [10.0] * self.MOTOR_NUM               # acceleration
        joint_target.kp = [10.0] * self.MOTOR_NUM                   # kp
        joint_target.kd = [0.1] * self.MOTOR_NUM                    # kd
        self._joint_target = joint_target
        
        #self._enable_joint_publisher.publish(True)
        rospy.Timer(rospy.Duration(2), self.controlCallback)
        rospy.spin()

    def controlCallback(self, event):
        #self._servo_on_publisher.publish(True)
        for i in range(self.MOTOR_NUM):
            self._joint_target.position[i] = self._joint_target.position[i] * -1
        self._joint_target_publisher.publish(self._joint_target)

if __name__ == '__main__':
    test()