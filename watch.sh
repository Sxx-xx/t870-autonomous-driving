#!/usr/bin/env bash
# 주행 확인용. 터미널 3 에서 이것만 치면 된다.
#   ./watch.sh
# 상태 한 줄 + 중요 로그가 같이 흐른다. Ctrl-C 로 종료.
WS=/home/sxx/Desktop/colcon_ws./colcon_ws/colcon_ws
if [ -n "$CONDA_PREFIX" ]; then
  PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | paste -sd:)
  unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH; export PATH
fi
cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash
source "$WS/install_t870/setup.bash"
exec /usr/bin/python3 "$WS/bev_calib/watch.py" "$@"
