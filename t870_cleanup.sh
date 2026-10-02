#!/bin/bash
# T870 노드 정리.
#
# 노드가 남는 이유: 터미널 창을 닫으면 SIGHUP 이 가는데 ros2 launch 가 그걸
# 자식들에게 정리 신호로 전달하지 못하면 자식이 고아로 살아남는다. launch
# 부모만 kill 해도 마찬가지다. 특히 remote_mode_control_node 는 웹 제어용
# 8080 포트를 물고 있어서, 살아남으면 다음 launch 의 새 인스턴스가
# "Address already in use" 로 1.5 초 만에 죽는다. 그러면 /cmd_vel 이
# 사라져 조종기를 AUTO 로 올려도 차가 전혀 움직이지 않는다.
#
# 쓰는 법: launch 를 Ctrl+C 로 끄고 몇 초 기다린 뒤 실행한다.
#          터미널 창을 그냥 닫아버렸을 때도 실행한다.

SELF=$$
alive() {
    ps -eo pid=,cmd= | awk -v self="$SELF" '
        $1 == self { next }
        /t870_cleanup/ { next }
        /install_t870\/t870_control\/lib/ ||
        /install_t870\/sllidar/ ||
        /install_t870\/ma_rrt/ ||
        /install_t870\/t870_track/ ||
        /robot_localization\/(ekf_node|navsat_transform)/ ||
        /ros2 launch t870/ { print $1 }'
}

echo "T870 노드 정리"
list=$(alive)
n=$(echo "$list" | grep -c . )
if [ "$n" -eq 0 ]; then
    echo "  실행 중인 노드 없음"
else
    echo "  $n 개 발견 -> 정상 종료 요청 (SIGINT)"
    for pid in $list; do kill -INT "$pid" 2>/dev/null; done
    for i in 1 2 3 4 5; do
        sleep 1
        [ -z "$(alive)" ] && break
    done
    left=$(alive)
    if [ -n "$left" ]; then
        echo "  안 죽은 $(echo "$left" | grep -c .) 개 -> 강제 종료 (SIGKILL)"
        for pid in $left; do kill -9 "$pid" 2>/dev/null; done
        sleep 2
    fi
fi

echo
echo "확인"
c=$(alive | grep -c .)
printf "  t870 노드   : %s\n" "$([ "$c" -eq 0 ] && echo '0 개 (깨끗)' || echo "$c 개 남음")"

printf "  8080 포트   : "
if ss -ltn 2>/dev/null | grep -q ':8080 '; then
    echo "아직 점유중 <- remote_mode_control_node 좀비"
else
    echo "free"
fi

echo "  시리얼 포트 :"
for d in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyACM3 /dev/ttyUSB0; do
    [ -e "$d" ] || continue
    who=$(fuser "$d" 2>/dev/null | tr -d ' ')
    if [ -n "$who" ]; then
        printf "     %-14s 점유중 (%s)\n" "$d" "$(ps -p "$who" -o comm= 2>/dev/null | head -1)"
    else
        printf "     %-14s free\n" "$d"
    fi
done
echo
echo "  MAVROS 는 건드리지 않는다. ttyACM 하나를 물고 있는 것이 정상이다."
