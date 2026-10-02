#!/usr/bin/env bash
# T870 대회 실행. 한 터미널에서 전부 띄우고 상태를 보여준다.
#
#   ./run.sh              점검 -> MAVROS -> 통합 launch -> 대시보드
#   ./run.sh --no-mavros  MAVROS 가 이미 돌고 있을 때
#   ./run.sh --log        대시보드 대신 로그를 필터링해서 본다
#
# Ctrl-C 한 번이면 전부 정리된다.

# set -u 는 쓰지 않는다. ROS 의 setup.bash 가 미정의 변수를 참조해서
# AMENT_TRACE_SETUP_FILES: unbound variable 로 죽는다.
WS=/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws
PIXHAWK=/dev/serial/by-id/usb-Holybro_Pixhawk6C_45002F001451333531373234-if00
WANT_MAVROS=1
VIEW=dash
for a in "$@"; do
  case "$a" in
    --no-mavros) WANT_MAVROS=0 ;;
    --log)       VIEW=log ;;
    *) echo "모르는 옵션: $a"; exit 1 ;;
  esac
done

# conda 가 끼면 ROS 가 파이썬 3.13 을 잡아 rclpy 임포트가 실패한다.
if [ -n "$CONDA_PREFIX" ]; then
  echo "conda 제거: ${CONDA_DEFAULT_ENV:-base}"
  PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | paste -sd:)
  unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
  export PATH
fi

cd "$WS" || exit 1

# ---- 1. 점검 -------------------------------------------------------------
echo "== 점검 =="
fail=0
for d in "$PIXHAWK" \
         /dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_34331323136351211280-if00 \
         /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_dc29dffaaa31f111b5fb935f30d20014-if00-port0 \
         /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00; do
  if [ -e "$d" ]; then echo "  OK   $(basename "$d" | cut -c5-28)"
  else echo "  없음 $(basename "$d" | cut -c5-28)"; fail=1; fi
done
for c in c920 sonix; do
  if ls /dev/v4l/by-id/ 2>/dev/null | grep -qi "$c.*index0"; then echo "  OK   카메라 $c"
  else echo "  없음 카메라 $c"; fail=1; fi
done
[ "$fail" = 1 ] && { echo; echo "장치가 빠졌다. USB 확인 후 다시."; exit 1; }

# 유령 프로세스 정리
ghosts=$(pgrep -f "static_transform_publisher|webcam_pub_node|ffplay" | wc -l)
if [ "$ghosts" -gt 0 ]; then
  echo "  유령 프로세스 $ghosts 개 정리"
  pkill -f static_transform_publisher; pkill -f webcam_pub_node; pkill -f ffplay
  sleep 2
fi
[ "$WANT_MAVROS" = 1 ] && pgrep -f mavros_node >/dev/null && {
  echo "  기존 MAVROS 종료"; pkill -f mavros_node; sleep 2; }

source /opt/ros/jazzy/setup.bash
source "$WS/install_t870/setup.bash"

# ---- 2. 정리 담당 --------------------------------------------------------
PIDS=()
cleanup() {
  echo; echo "== 정리 =="
  for p in "${PIDS[@]}"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
  sleep 1
  pkill -f "ros2 launch t870_control" 2>/dev/null
  [ "$WANT_MAVROS" = 1 ] && pkill -f mavros_node 2>/dev/null
  pkill -f webcam_pub_node 2>/dev/null
  pkill -f static_transform_publisher 2>/dev/null
  echo "  완료. 로그: /tmp/t870_competition.log"
  exit 0
}
trap cleanup INT TERM

# ---- 3. MAVROS -----------------------------------------------------------
if [ "$WANT_MAVROS" = 1 ]; then
  echo "== MAVROS =="
  ros2 run mavros mavros_node --ros-args \
    -p fcu_url:="serial://$PIXHAWK:115200" \
    -p system_id:=255 -p component_id:=191 \
    -p target_system_id:=1 -p target_component_id:=1 \
    > /tmp/t870_mavros.log 2>&1 &
  PIDS+=($!)
  sleep 4
  grep -qi "got hearbeat\|heartbeat\|CON: Got" /tmp/t870_mavros.log 2>/dev/null \
    && echo "  heartbeat OK" || echo "  heartbeat 아직 (계속 진행)"
else
  echo "== MAVROS 건너뜀 =="
fi

# ---- 4. 통합 launch ------------------------------------------------------
echo "== 통합 launch =="
ros2 launch t870_control t870_competition.launch.py \
  > /tmp/t870_competition.log 2>&1 &
PIDS+=($!)

for i in $(seq 1 20); do
  sleep 1
  if grep -q "mission manager ready" /tmp/t870_competition.log 2>/dev/null; then
    echo "  미션 매니저 기동"; break; fi
  [ "$i" = 20 ] && echo "  20초 내 기동 확인 실패. 로그를 볼 것."
done
sleep 3
echo "  카메라: $(grep -c '발행' /tmp/t870_competition.log) 회 보고"
grep -m2 "발행 .* Hz" /tmp/t870_competition.log | sed 's/^/    /'
grep -m1 "Lookahead override" /tmp/t870_competition.log | sed 's/.*\]: /    /'
grep -m1 "path follower ready" /tmp/t870_competition.log | sed 's/.*\]: /    /'
echo
echo "리모콘 AUTO = 주행,  MANUAL = 정지.   Ctrl-C 로 전부 종료."
echo "-----------------------------------------------------------------"

# ---- 5. 화면 -------------------------------------------------------------
if [ "$VIEW" = dash ]; then
  /usr/bin/python3 "$WS/bev_calib/dash.py"
else
  tail -f /tmp/t870_competition.log | grep --line-buffered -E \
    "AVOID|Traffic light|PARKING|동적|인계|PATH SWITCH|E-STOP|발행|읽기실패"
fi
cleanup
