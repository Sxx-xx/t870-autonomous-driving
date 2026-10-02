#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import time

def main():
    rospy.init_node('stop_line_publisher')
    pub = rospy.Publisher('/stop_line', Bool, queue_size=1)

    rate = rospy.Rate(20)  # 20Hz 퍼블리시
    last_detect_time = 0.0
    stop_line_active = False
    camera_start_time = rospy.Time.now().to_sec() + 5.0  # 예: 시작 후 5초 뒤 카메라 작동 시작

    while not rospy.is_shutdown():
        msg = Bool()
        current_time = rospy.Time.now().to_sec()

        # ▶️ 카메라 작동 구간일 때만 정지선 인식 작동
        if current_time >= camera_start_time:
            # ✅ 예시로 5초 간격마다 정지선 인식됨을 가정
            if int(current_time) % 5 == 0 and not stop_line_active:
                last_detect_time = current_time
                stop_line_active = True

            # 정지선 인식 후 1.0초간 True 유지
            if (current_time - last_detect_time) <= 1.0:
                msg.data = True
            else:
                msg.data = False
                stop_line_active = False
        else:
            msg.data = False  # 카메라 시작 전에는 무조건 False

        pub.publish(msg)
        rospy.loginfo(f"[퍼블리시] stop_line = {msg.data}")
        rate.sleep()

if __name__ == '__main__':
    main()
