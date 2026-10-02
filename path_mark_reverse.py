#!/usr/bin/env python3
"""경로의 어떤 구간을 후진으로 표시한다.

follower 는 경로 파일의 target_speed 부호로 전/후진을 가린다. 음수면
그 웨이포인트는 후진 구간이다. T 자 주차처럼 "idx A 에서 idx B 까지
후진" 을 경로 파일 하나로 표현할 수 있다.

전환 지점에서는 follower 가 먼저 차를 세운다(reverse_switch_speed_mps).
모터 드라이버 보호 때문에 펌웨어도 같은 일을 하므로 전환은 반드시
정지를 거친다. 그 자리에 점을 몇 개 더 두면 정지가 부드러워진다.

쓰는 법:
    # idx 40~55 를 후진 1.0 m/s 로
    python3 path_mark_reverse.py gps_recordings/park.csv 40 55 --speed 1.0

    # 되돌리기 (전진으로)
    python3 path_mark_reverse.py gps_recordings/park.csv 40 55 --forward

    # 파일은 그대로 두고 결과만 보기
    python3 path_mark_reverse.py gps_recordings/park.csv 40 55 --dry-run
"""
import argparse
import csv
import shutil


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('csv_file')
    parser.add_argument('start', type=int, help='시작 idx (포함)')
    parser.add_argument('end', type=int, help='끝 idx (포함)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='구간 속도 m/s, 절댓값으로 준다 (기본 1.0)')
    parser.add_argument('--forward', action='store_true',
                        help='후진 표시를 풀고 전진으로 되돌린다')
    parser.add_argument('--dry-run', action='store_true')
    arguments = parser.parse_args()

    with open(arguments.csv_file, newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames or []
        rows = list(reader)
    if 'target_speed' not in fieldnames:
        raise SystemExit('target_speed 컬럼이 없다')
    if not 0 <= arguments.start <= arguments.end < len(rows):
        raise SystemExit('idx 범위가 경로(0~%d) 밖이다' % (len(rows) - 1))

    value = abs(arguments.speed) if arguments.forward else -abs(arguments.speed)
    for index in range(arguments.start, arguments.end + 1):
        rows[index]['target_speed'] = '%.4f' % value

    marked = [i for i, r in enumerate(rows) if float(r['target_speed'] or 0) < 0]
    runs = []
    for i in marked:
        if runs and i == runs[-1][1] + 1:
            runs[-1][1] = i
        else:
            runs.append([i, i])

    print('%s  %d점' % (arguments.csv_file, len(rows)))
    print('  idx %d~%d 을 %s %.2f m/s (%.1f km/h) 로 표시'
          % (arguments.start, arguments.end,
             '전진' if arguments.forward else '후진',
             abs(value), abs(value) * 3.6))
    print()
    print('  이 파일의 후진 구간: %s'
          % (', '.join('idx %d~%d (%d점)' % (a, b, b - a + 1) for a, b in runs)
             or '없음'))

    if arguments.dry_run:
        print()
        print('--dry-run 이라 파일은 그대로 두었다.')
        return

    shutil.copy2(arguments.csv_file, arguments.csv_file + '.bak')
    with open(arguments.csv_file, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print()
    print('  반영했다. 원본은 %s.bak' % arguments.csv_file)
    print('  주행 명령에 gps_allow_reverse:=true 를 붙여야 후진이 동작한다.')


if __name__ == '__main__':
    main()
