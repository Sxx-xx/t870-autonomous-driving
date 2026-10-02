#!/home/navigator/catkin_ws/venv/bin/python
# 위 해석기는 python3.9 가상환경

import sys
import torch, cv2
import time
from collections import deque
import rospy
from std_msgs.msg import String
SHOW_FPS_OVERLAY = True  # True면 imshow에 그려서 보이게
PRINT_FPS_EVERY  = 1.0   # 초 단위: 로그 출력 주기

# # --- ROS 가져오기: venv에서 rospy가 안 보일 때 대비 ---
# try:
#     import rospy
# except ImportError:
#     sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
#     import rospy
# from std_msgs.msg import String

# 1) 모델 로드 (커스텀 가중치)
model = torch.hub.load('/home/navigator/yolov5', 'custom',
                       path='/home/navigator/yolov5/runs/train/exp3/weights/best.pt',
                       source='local', force_reload=True)
model.conf = 0.4   # NMS confidence
model.iou  = 0.45   # NMS IoU
# model.classes = [0]  # 특정 클래스만 사용할 때(예: 'text'가 0번일 때)

# 2) ROS 퍼블리셔 준비
rospy.init_node('yolov5_text_pub', anonymous=False)
pub_a = rospy.Publisher('/yolo/current_text_a', String, queue_size=10)
pub_b = rospy.Publisher('/yolo/current_text_b', String, queue_size=10)
rate = rospy.Rate(15)  # Hz

cap = cv2.VideoCapture(0
                       )

_last_log_t = time.perf_counter()
_loop_dt_win = deque(maxlen=30)   # 전체 루프 FPS(캡처+추론+후처리)
_inf_dt_win  = deque(maxlen=30)   # 순수 추론 FPS(모델 forward만)

while not rospy.is_shutdown():
    t_loop0 = time.perf_counter()
    ok, frame = cap.read()
    if not ok:
        rospy.logwarn("Failed to read frame from camera")
        break

    # 3) 추론
    t_inf0 = time.perf_counter()
    results = model(frame, size=512)
    t_inf1 = time.perf_counter()
    _inf_dt_win.append(t_inf1 - t_inf0)

    # 4) 박스 파싱: [x1, y1, x2, y2, conf, cls]
    boxes = results.xyxy[0].cpu().numpy()

    # 5) 현재 프레임에서 "가장 확신(conf) 높은" 클래스 이름 하나 선택
    current_name = 'none'
    if boxes.size > 0:
        # conf가 최대인 인덱스
        best_idx = boxes[:, 4].argmax()
        cls_id = int(boxes[best_idx, 5])
        current_name = model.names[int(cls_id)]

        # # 보기용 오버레이(선택)
        x1, y1, x2, y2, conf, _ = boxes[best_idx]
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
        cv2.putText(frame, f'{current_name} {conf:.2f}', (x1, y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # 6) ROS로 전송(String): 인식 없으면 'none'
    if current_name in ['A1','A2','A3']:
        pub_a.publish(String(data=current_name))
    elif current_name in ['B1','B2','B3']:
        pub_b.publish(String(data=current_name))


    t_loop1 = time.perf_counter()
    _loop_dt_win.append(t_loop1 - t_loop0)


    # --- 화면 오버레이(선택) ---
    # if SHOW_FPS_OVERLAY and len(_loop_dt_win) >= 5:
    #     avg_loop_dt = sum(_loop_dt_win)/len(_loop_dt_win)
    #     avg_inf_dt  = sum(_inf_dt_win) /len(_inf_dt_win)  if _inf_dt_win else 0.0
    #     loop_fps = (1.0/avg_loop_dt) if avg_loop_dt > 0 else 0.0
    #     inf_fps  = (1.0/avg_inf_dt ) if avg_inf_dt  > 0 else 0.0

    #     cv2.putText(frame, f"FPS(loop): {loop_fps:.1f}",
    #                 (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
    #     cv2.putText(frame, f"FPS(infer): {inf_fps:.1f}",
    #                 (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

    cv2.imshow('text-detect', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    
    rate.sleep()

cap.release()
cv2.destroyAllWindows()
