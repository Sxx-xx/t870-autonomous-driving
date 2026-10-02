#!/usr/bin/env python3
"""MANUAL 주행 중 RTK 가 언제 끊기는지 눈으로 보고 기록한다.

수신기 시리얼을 직접 읽으므로 ROS 를 띄우지 않는다. 조종기로 MANUAL
주행하면서 이 창만 보면 된다. 끊길 때와 붙을 때 굵게 한 줄씩 찍고,
끊긴 순간의 속도와 선회율을 같이 남긴다. 그러면 코너 때문인지 속도
때문인지 나중에 숫자로 가릴 수 있다.

RMC 문장에서 대지속도와 진행방위를 같이 읽는다. 방위 변화율이 크면
코너를 돌고 있는 것이다. 라이다도 ROS 도 필요 없다.

쓰는 법 (conda 말고 시스템 파이썬으로):
    /usr/bin/python3 rtk_monitor.py          # 무한, Ctrl+C 로 종료
    /usr/bin/python3 rtk_monitor.py 300      # 300초만
"""
import math
import signal
import sys
import time

import serial

PORT = ('/dev/serial/by-id/'
        'usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00')
OUT = '/tmp/rtk_monitor.csv'
# 4=RTK Fix, 5=RTK Float. 둘 다 보정이 붙은 상태다.
RTK = ('4', '5')
NAME = {'0': '측위불가', '1': '단독측위', '2': 'DGPS',
        '4': 'RTK Fix', '5': 'RTK Float'}
BAR = '=' * 62


def nmea_degrees(value, hemi):
    degrees = int(float(value) / 100)
    decimal = degrees + (float(value) - degrees * 100) / 60
    return -decimal if hemi in ('S', 'W') else decimal


def main():
    limit = float(sys.argv[1]) if len(sys.argv) > 1 else float('inf')
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

    drops = []            # (시각, 길이, 속도, 선회율, 직전품질)
    locked = None         # 지금 RTK 인가
    lost_at = None
    speed_kmh = 0.0
    course = None
    course_time = None
    turn_rate = 0.0
    quality = '0'
    started = time.time()

    print('RTK 감시 시작. 조종기로 MANUAL 주행하면서 이 창을 봐라.')
    print('기록: %s     끝내려면 Ctrl+C' % OUT)
    print(BAR)

    stream = open(OUT, 'w', encoding='utf-8')
    stream.write('time,quality,age_s,sats,hdop,lat,lon,speed_kmh,'
                 'course_deg,turn_deg_s,rtk\n')
    try:
        with serial.Serial(PORT, 38400, timeout=1) as port:
            while time.time() - started < limit:
                line = port.readline().decode('ascii', 'ignore').strip()
                now = time.time()

                if 'RMC' in line[:7]:
                    field = line.split(',')
                    if len(field) > 8 and field[2] == 'A':
                        try:
                            speed_kmh = float(field[7] or 0.0) * 1.852
                            if field[8]:
                                new_course = float(field[8])
                                if course is not None and course_time:
                                    step = (new_course - course + 540.0) % 360.0 - 180.0
                                    dt = now - course_time
                                    if dt > 0.05:
                                        turn_rate = step / dt
                                course, course_time = new_course, now
                        except ValueError:
                            pass
                    continue

                if 'GGA' not in line[:7]:
                    continue
                field = line.split(',')
                if len(field) < 14 or not field[2] or not field[4]:
                    continue
                quality = field[6]
                age, sats, hdop = field[13] or '-', field[7], field[8]
                lat = nmea_degrees(field[2], field[3])
                lon = nmea_degrees(field[4], field[5])
                is_rtk = quality in RTK

                stream.write('%.3f,%s,%s,%s,%s,%.8f,%.8f,%.2f,%s,%.1f,%d\n' % (
                    now, quality, age, sats, hdop, lat, lon, speed_kmh,
                    '' if course is None else '%.1f' % course,
                    turn_rate, 1 if is_rtk else 0))
                stream.flush()

                if locked is None:
                    locked = is_rtk
                    print('  시작 상태: %s   위성 %s   HDOP %s'
                          % (NAME.get(quality, quality), sats, hdop))
                    if not is_rtk:
                        lost_at = now
                    continue

                if locked and not is_rtk:
                    locked = False
                    lost_at = now
                    print(BAR)
                    print('  !! RTK 끊김   %s   %s'
                          % (time.strftime('%H:%M:%S'),
                             NAME.get(quality, quality)))
                    print('     속도 %.1f km/h,  선회율 %+.0f 도/초  %s'
                          % (speed_kmh, turn_rate,
                             '<- 코너 중' if abs(turn_rate) > 8 else '<- 직진 중'))
                    print(BAR)
                elif not locked and is_rtk:
                    locked = True
                    held = now - lost_at if lost_at else 0.0
                    drops.append((lost_at, held, speed_kmh, turn_rate))
                    print('  -- 복귀   %s   끊긴 시간 %.1f초   (%s)'
                          % (time.strftime('%H:%M:%S'), held,
                             NAME.get(quality, quality)))
                    print(BAR)
    finally:
        if not locked and lost_at:
            drops.append((lost_at, time.time() - lost_at, speed_kmh, turn_rate))
        stream.close()
        print()
        print(BAR)
        print('  요약: 끊김 %d회, 총 %.0f초' % (
            len(drops), sum(d[1] for d in drops)))
        if drops:
            print()
            print('    시각      길이    속도       선회율    판정')
            for at, held, spd, turn in drops:
                print('    %s  %5.1f초  %4.1f km/h  %+5.0f 도/초  %s' % (
                    time.strftime('%H:%M:%S', time.localtime(at)), held, spd,
                    turn, '코너' if abs(turn) > 8 else '직진'))
            corner = sum(1 for d in drops if abs(d[3]) > 8)
            print()
            print('    코너에서 %d회, 직선에서 %d회'
                  % (corner, len(drops) - corner))
            fast = [d for d in drops if d[2] >= 5.0]
            if fast:
                print('    5 km/h 이상에서 %d회, 평균 %.0f초'
                      % (len(fast), sum(d[1] for d in fast) / len(fast)))
        print('  전체 기록: %s' % OUT)
        print(BAR)


if __name__ == '__main__':
    main()
