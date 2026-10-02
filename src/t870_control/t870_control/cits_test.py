#!/usr/bin/env python3

import rospy
import time
from v2x_msgs.msg import Spat, Interchange, Id, State, State_time_speed

def publish_v2x_messages():
    rospy.init_node('v2x_manual_publisher', anonymous=True)
    pub = rospy.Publisher('v2x_data', Spat, queue_size=10)
    
    rate = rospy.Rate(10)  # 10Hz로 발행

    # 발행할 신호 상태와 기간 설정 (단위: 0.1초)
    event_states = [
        ("stop-And-Remain", 300),  # 20초 동안 발행 (200 * 0.1초)
        ("protected-Movement-Allowed", 200)  # 5초 동안 발행 (50 * 0.1초)
    ]

    index = 0  # 현재 이벤트 상태의 인덱스
    state_start_time = rospy.Time.now().to_sec()
    current_state_duration = event_states[index][1] / 10.0  # 초 단위로 변환

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - state_start_time

        # 상태 전환 체크
        if elapsed_time >= current_state_duration:
            index = (index + 1) % len(event_states)
            state_start_time = current_time
            current_state_duration = event_states[index][1] / 10.0  # 다음 상태의 기간 설정
            rospy.loginfo("Switching to state: %s", event_states[index][0])

        # SPaT 메시지 생성
        spat_msg = Spat()
        interchange = Interchange()
        interchange.id = Id(region=0, id=300)  # 교차로 ID 설정
        state = State()
        state.movementName = "STR"  # 직진 신호
        state.signalGroup = 15  # Signal Group 설정
        state_time_speed = State_time_speed()
        state_time_speed.event_state = event_states[index][0]  # 현재 이벤트 상태 설정
        remaining_time = int((current_state_duration - elapsed_time) * 10)  # 남은 시간 계산 (단위: 0.1초)
        state_time_speed.timing.minEndTime = remaining_time
        state_time_speed.timing.maxEndTime = remaining_time

        # 수정된 부분: state_time_speed를 단일 객체로 할당
        state.state_time_speed = state_time_speed

        # states를 리스트로 설정
        interchange.states = [state]
        spat_msg.interchanges = [interchange]

        # 메시지 퍼블리시
        pub.publish(spat_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_v2x_messages()
    except rospy.ROSInterruptException:
        pass
