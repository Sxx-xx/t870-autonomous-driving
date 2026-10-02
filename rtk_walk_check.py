#!/usr/bin/env python3
"""RTK 보정 상태를 위치와 함께 기록한다.

걸으면서 돌리면 어디서 끊기는지 남는다. 끊긴 지점이 한곳에 몰리면 그 자리의
전파 장애(건물/나무)이고, 고르게 흩어지면 케이블이나 커넥터 접촉 문제다.
"""
import serial, time, math, sys
from collections import Counter

PORT = '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'
QUALITY = {'0': '측위불가', '1': '단독측위', '2': 'DGPS', '4': 'RTK Fix', '5': 'RTK Float'}
OUT = '/tmp/rtk_walk.csv'


def nmea_degrees(value, hemi):
    degrees = int(float(value) / 100)
    decimal = degrees + (float(value) - degrees * 100) / 60
    return -decimal if hemi in ('S', 'W') else decimal


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 300
    counts = Counter()
    drops = 0
    previous = None
    print('%s 초간 기록. Ctrl+C 로 중단.' % int(seconds))
    print('품질이 바뀔 때만 한 줄씩 찍는다. 전체 기록은 %s' % OUT)
    with serial.Serial(PORT, 38400, timeout=1) as port, open(OUT, 'w') as out:
        out.write('time,quality,age_s,sats,hdop,lat,lon\n')
        end = time.time() + seconds
        while time.time() < end:
            line = port.readline().decode('ascii', 'ignore').strip()
            if 'GGA' not in line[:7]:
                continue
            f = line.split(',')
            if len(f) < 14 or not f[2] or not f[4]:
                continue
            quality, age, sats, hdop = f[6], f[13], f[7], f[8]
            lat = nmea_degrees(f[2], f[3])
            lon = nmea_degrees(f[4], f[5])
            counts[quality] += 1
            out.write('%.3f,%s,%s,%s,%s,%.7f,%.7f\n'
                      % (time.time(), quality, age, sats, hdop, lat, lon))
            out.flush()
            if quality != previous:
                if previous is not None and quality in ('0', '1'):
                    drops += 1
                print('  %s  %-10s age=%-5s 위성=%s  %.6f, %.6f'
                      % (time.strftime('%H:%M:%S'), QUALITY.get(quality, quality),
                         age or '-', sats, lat, lon))
                previous = quality
    total = sum(counts.values()) or 1
    print()
    print('=== 요약 ===')
    for key, n in counts.most_common():
        print('  %-10s %5d회 (%.0f%%)' % (QUALITY.get(key, key), n, 100 * n / total))
    print('  RTK 이탈 횟수: %d' % drops)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n중단됨. 기록은 %s 에 남아 있다.' % OUT)
