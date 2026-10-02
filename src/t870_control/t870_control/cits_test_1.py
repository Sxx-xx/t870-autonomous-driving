#!/usr/bin/env python3

import rospy
from v2x_msgs.msg import Spat, Interchange, Id, State, State_time_speed

def publish_v2x_messages():
    rospy.init_node('v2x_manual_publisher', anonymous=True)
    pub = rospy.Publisher('v2x_data', Spat, queue_size=10)
    
    rate = rospy.Rate(10)  # 10Hz로 발행

    # Interchange 정의
    interchanges = {
        300: {
            15: [
                ("stop-And-Remain", 150),  # 15초
                ("protected-Movement-Allowed", 150)  # 15초
            ],
            14: [
                ("protected-Movement-Allowed", 100),  # 10초
                ("stop-And-Remain", 200)  # 20초
            ]
        },
        3000: {
            11: [
                ("protected-Movement-Allowed", 100),  # 10초
                ("stop-And-Remain", 100)  # 10초
            ],
            2: [
                ("stop-And-Remain", 150),  # 15초
                ("protected-Movement-Allowed", 150)  # 15초
            ]
        },
        200: {
            14: [
                ("stop-And-Remain", 200),  # 20초
                ("protected-Movement-Allowed", 200)  # 20초
            ],
            15: [
                ("protected-Movement-Allowed", 150),  # 15초
                ("stop-And-Remain", 150)  # 15초
            ]
        }
    }

    # 상태 추적을 위한 데이터 구조 초기화
    state_tracking = {}
    for interchange_id, signal_groups in interchanges.items():
        state_tracking[interchange_id] = {}
        for signal_group, states in signal_groups.items():
            state_tracking[interchange_id][signal_group] = {
                'states': states,
                'current_index': 0,
                'start_time': rospy.Time.now().to_sec(),
                'current_duration': states[0][1] / 10.0  # 초 단위
            }

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()

        spat_msg = Spat()
        spat_msg.interchanges = []

        for interchange_id, signal_groups in interchanges.items():
            interchange = Interchange()
            interchange.id = Id(region=0, id=interchange_id)
            interchange.states = []

            for signal_group, states in signal_groups.items():
                tracking = state_tracking[interchange_id][signal_group]
                elapsed_time = current_time - tracking['start_time']

                # 상태 전환 체크
                if elapsed_time >= tracking['current_duration']:
                    tracking['current_index'] = (tracking['current_index'] + 1) % len(tracking['states'])
                    tracking['start_time'] = current_time
                    tracking['current_duration'] = tracking['states'][tracking['current_index']][1] / 10.0
                    rospy.loginfo(f"Switching to state (ID {interchange_id}, SignalGroup {signal_group}): {tracking['states'][tracking['current_index']][0]}")

                # 현재 상태 정보 가져오기
                current_state, duration = tracking['states'][tracking['current_index']]
                remaining_time = int((tracking['current_duration'] - (current_time - tracking['start_time'])) * 10)

                # State_time_speed 객체 생성
                sts = State_time_speed()
                sts.event_state = current_state
                sts.timing.minEndTime = remaining_time
                sts.timing.maxEndTime = remaining_time

                # State 객체 생성
                state = State()
                state.movementName = "STR"  # 직진 신호
                state.signalGroup = signal_group
                state.state_time_speed = sts

                # Interchange에 State 추가
                interchange.states.append(state)

            # SPaT 메시지에 Interchange 추가
            spat_msg.interchanges.append(interchange)

        # 메시지 퍼블리시
        pub.publish(spat_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_v2x_messages()
    except rospy.ROSInterruptException:
        pass