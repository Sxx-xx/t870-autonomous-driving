#! /usr/bin/env python3

import rospy
from morai_msgs.msg import CtrlCmd

# 전역 변수 초기화
speed = 0
steer = 0
gear = 0
encoder = 0

def generate_fixed_ctrl_cmd():
    cmd = CtrlCmd()
    cmd.gear = 0  # 고정된 기어 값
    cmd.accel = 30  # 고정된 속도 값
    cmd.steering = 0 # 고정된 조향 값
    cmd.brake = 1  # 고정된 브레이크 값
    return cmd

if __name__ == '__main__':
    rospy.init_node('cmd_publisher')

    pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
    rate = rospy.Rate(20)  # 20Hz로 퍼블리시

    while not rospy.is_shutdown():
        ctrl_cmd = generate_fixed_ctrl_cmd()
        rospy.loginfo(f"Publishing: gear={ctrl_cmd.gear}, accel={ctrl_cmd.accel}, steering={ctrl_cmd.steering}, brake={ctrl_cmd.brake}")
        pub.publish(ctrl_cmd)
        rate.sleep()
