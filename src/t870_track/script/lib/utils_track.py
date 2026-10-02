#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rospkg
from geometry_msgs.msg import Point
from std_msgs.msg import Float64,Int16,Float32MultiArray, Int32, Float32
#import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi
from vehicle_msgs.msg import Waypoint, WaypointsArray
from math import *

import math
import time
import serial

class purePursuit_nogps :
    def __init__(self):
        self.forward_point=Waypoint()
        self.vehicle_length=1.08  #1.375
        self.steering=0

    def getVelStatus(self,msg):
        self.current_vel=msg.velocity

#######Local Path#######
    def steering_angle(self, newpoints):

        if (len(newpoints.waypoints) <= 0) :
            self.steering = 0
            return self.steering, self.forward_point

        self.forward_point = newpoints.waypoints[0]

        # elif (len(newpoints.waypoints) == 1):
        #     self.forward_point = newpoints[0]

        # else :
        #     for i in newpoints.waypoints :
        #         dis=sqrt(pow(i.x,2)+pow(i.y,2))

        #         if(dis>=2) :
        #             self.forward_point = newpoints.waypoints[i]
        #             break

        #         elif(i == len(newpoints)-1):
        #             self.forward_point = newpoints.waypoints[1]

        dx = self.forward_point.x
        dy = self.forward_point.y
        dis = math.sqrt(dx*dx + dy*dy)
        alpha = atan(dy/dx) #radian
        delta = 2 * self.vehicle_length * sin(alpha)/dis
        self.steering= -atan(delta)*180/pi #deg
        self.steering *= 1.3
        rospy.loginfo("target point : {0}        {1}".format(self.forward_point.x, self.forward_point.y))
        rospy.loginfo("target steer : {0}".format(self.steering))
        if self.steering <0:
            return self.steering-11, self.forward_point
        else:
            return self.steering+9.2, self.forward_point

class pidController : ## 속도 제어를 위한 PID 적용 ##
    def __init__(self):
        self.p_gain=1.0 #t
        self.i_gain=0.4  #steady state error
        self.d_gain=0.35 #overshoot
        #self.controlTime=0.05
        self.controlTime=0.1
        self.prev_error=0
        self.i_control=0


    def pid(self, current_vel,target_velocity):
        error= target_velocity-current_vel

        p_control=self.p_gain*error
        self.i_control+=self.i_gain*error*self.controlTime
        d_control=self.d_gain*(error-self.prev_error)/self.controlTime

        output=p_control+self.i_control+d_control
        self.prev_error=error
        rospy.loginfo("current_vel : {0}     target_velocity : {1}".format(current_vel, target_velocity))
        rospy.loginfo("p_control : {0}     i_control : {1}      d_control : {2}".format(p_control, self.i_control, d_control))
        rospy.loginfo("error : {0}        output : {1}".format(error, output))

        return output
