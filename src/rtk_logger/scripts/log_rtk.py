#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import NavSatFix
import datetime
import utm

def log_rtk_data(data):
    # 위도와 경도를 UTM 좌표로 변환
    utm_coord = utm.from_latlon(data.latitude, data.longitude)
    easting, northing, zone_number, zone_letter = utm_coord

    # 파일을 열고 데이터를 추가합니다. 파일 경로는 적절히 수정하세요.
    with open("/home/navigator/logging_test_20250707.txt", "a") as file:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = "{} - Lat: {}, Lon: {}, Alt: {}, UTM Easting: {}, Northing: {}, Zone: {}{}\n".format(
            timestamp, data.latitude, data.longitude, data.altitude, easting, northing, zone_number, zone_letter)
        file.write(log_entry)

def listener():
    # 노드 초기화 및 토픽 구독
    rospy.init_node('rtk_logger_node', anonymous=True)
    rospy.Subscriber("/ublox_gps/fix", NavSatFix, log_rtk_data)
    rospy.spin()

if __name__ == '__main__':
    listener()

