#!/usr/bin/env python3
"""차가 안 움직일 때 어느 단계에서 끊겼는지 아두이노 텔레메트리로 가린다.

펌웨어가 100 ms 마다 한 줄씩 상태를 뱉는다. 그 한 줄에 조종기 입력부터
모터 PWM 까지 전 단계가 다 들어 있어서, 어디서 멈췄는지 바로 갈린다.

    RC:MANUAL TH:1744/45 ST:1496/0 | POT:549 | SteerPWM:LOST
    | SteerTarget:543 | Tgt:0.10 | Act:0.07 | C/100ms:1 | Cnt:2 | PWM:8

  RC      조종기 모드. LOST 면 수신기나 텔레메트리가 끊긴 것
  TH      스로틀 스틱 (원시 us / 퍼센트)
  ST      조향 스틱
  Tgt     펌웨어가 잡은 목표 속도 km/h
  Act     엔코더 실측 속도 km/h
  PWM     모터로 나간 duty

ROS 스택이 포트를 물고 있으면 못 읽는다. 먼저 bash t870_cleanup.sh.

쓰는 법 (conda 말고 시스템 파이썬으로):
    /usr/bin/python3 arduino_check.py
    /usr/bin/python3 arduino_check.py 30      # 30초만
"""
import re
import sys
import time

import serial

PORT = ('/dev/serial/by-id/'
        'usb-Arduino__www.arduino.cc__0043_34331323136351211280-if00')
FIELD = re.compile(
    r'RC:(?P<rc>\w+)\s+TH:(?P<th_us>-?\d+)/(?P<th>-?\d+)\s+'
    r'ST:(?P<st_us>-?\d+)/(?P<st>-?\d+).*?'
    r'Tgt:(?P<tgt>-?[\d.]+).*?Act:(?P<act>-?[\d.]+).*?PWM:(?P<pwm>-?\d+)')


def verdict(row):
    """한 줄을 보고 어느 단계가 막혔는지 고른다."""
    if row['rc'] == 'LOST':
        return '조종기 신호 없음 -> 수신기 전원/바인딩, 아두이노 텔레메트리 배선'
    if abs(row['th']) < 5:
        return '스로틀 스틱이 중립이다 (정상. 밀어 보고 TH 가 변하는지 봐라)'
    if abs(row['tgt']) < 0.02:
        return ('스틱은 움직이는데 목표가 0 이다 -> 중립 대기 래치나 '
                'E-stop. 스틱을 중립까지 완전히 돌렸다 다시 밀어 봐라')
    if row['pwm'] == 0:
        return '목표는 있는데 PWM 이 0 이다 -> 펌웨어 게이트(정지 문턱/방향 반전 보호)'
    if abs(row['act']) < 0.03:
        return 'PWM 은 나가는데 안 돈다 -> 모터/드라이버 전원, 체인, 커플러'
    return '정상. 목표와 실측이 모두 살아 있다'


def main():
    limit = float(sys.argv[1]) if len(sys.argv) > 1 else float('inf')
    started = time.time()
    last = None
    seen = 0
    print('아두이노 상태 감시. 조종기 스로틀을 앞뒤로 밀어 봐라. Ctrl+C 로 종료.')
    print('-' * 72)
    try:
        with serial.Serial(PORT, 115200, timeout=1) as port:
            time.sleep(2.0)
            port.reset_input_buffer()
            while time.time() - started < limit:
                line = port.readline().decode('ascii', 'ignore').strip()
                match = FIELD.search(line)
                if not match:
                    continue
                seen += 1
                row = {k: (float(v) if k in ('tgt', 'act') else
                           (v if k == 'rc' else int(v)))
                       for k, v in match.groupdict().items()}
                state = (row['rc'], abs(row['th']) > 5, abs(row['tgt']) > 0.02,
                         row['pwm'] > 0, abs(row['act']) > 0.03)
                if state != last:
                    last = state
                    print('  RC:%-7s TH:%+4d%%  Tgt:%+5.2f  PWM:%3d  Act:%+5.2f'
                          % (row['rc'], row['th'], row['tgt'], row['pwm'],
                             row['act']))
                    print('     -> %s' % verdict(row))
    except KeyboardInterrupt:
        pass
    except (OSError, serial.SerialException) as error:
        raise SystemExit('시리얼을 못 열었다 (스택이 물고 있나?): %s' % error)
    finally:
        print('-' * 72)
        if seen == 0:
            print('  텔레메트리 한 줄도 못 받았다.')
            print('  -> 아두이노가 멎었거나 펌웨어가 안 돈다. 리셋 버튼을 눌러라.')
        else:
            print('  %d줄 받았다. 펌웨어는 살아 있다.' % seen)


if __name__ == '__main__':
    main()
