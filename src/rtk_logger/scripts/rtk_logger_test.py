#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavPVT
import datetime
import utm

# 전역 변수로 lat, lon, heading 선언
current_lat = 0.0
current_lon = 0.0
current_heading = 0.0
def log_navpvt_data(event):
    # 전역 변수 사용
    global current_lat, current_lon, current_heading

    # GPS 데이터를 UTM 좌표로 변환sudo chmod 777 /dev/ttyACM0
    utm_coords = utm.from_latlon(current_lat, current_lon)
    easting, northing, zone_number, zone_letter = utm_coords

    # 파일을 열고 데이터를 추가합니다. 파일 경로는 적절히 수정하세요.
    with open("/home/navigator/logging/lil_ob.txt", "a") as file:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = (
            f"{timestamp} - Lat: {current_lat}, Lon: {current_lon}, "
            f"Heading: {current_heading}, Easting: {easting}, "
            f"Northing: {northing}, Zone: {zone_number}{zone_letter}\n"
        )
        file.write(log_entry)

def navsatfix_callback(data):
    global current_lat, current_lon
    current_lat = data.latitude
    current_lon = data.longitude

def navpvt_callback(data):
    global current_heading
    current_heading = data.heading

def listener():
    # 노드 초기화 및 토픽 구독
    rospy.init_node('navpvt_logger_node', anonymous=True)
    rospy.Subscriber("/ublox/fix", NavSatFix, navsatfix_callback)
    rospy.Subscriber("/ublox/navpvt", NavPVT, navpvt_callback)
    rospy.Timer(rospy.Duration(1), log_navpvt_data)
    rospy.spin()

if __name__ == '__main__':
    
    listener()
    
