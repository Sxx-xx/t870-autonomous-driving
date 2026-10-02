#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from MaRRTPathPlanNode import MaRRTPathPlanNode

def main():
    rospy.init_node('MaRRTPathPlanNode')
    maRRTPathPlanNode = MaRRTPathPlanNode()

    rate = rospy.Rate(1000) # big amount on purpose

    while not rospy.is_shutdown():
        maRRTPathPlanNode.sampleTree()
        rate.sleep()

    # Spin until ctrl + c
    # rospy.spin()

if __name__ == '__main__':
    main()