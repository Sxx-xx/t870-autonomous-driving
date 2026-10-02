"""대회용 CSV 가 실제로 쓸 수 있는 상태인지 검사한다.

QGIS 로 다시 내보낼 때마다 같은 사고가 반복됐다.
  - 인코딩이 CP949/latin-1 로 돌아온다 -> follower 가 UnicodeDecodeError 로 죽는다
  - WP300/301 이 뒤바뀌어 들어온다     -> 없는 후진 구간이 생긴다
  - 후진 구간이 launch 설정과 어긋난다 -> 기어가 반대로 들어간다

follower 가 읽는 방식(easting/northing 우선)과 같게 검사한다.
"""
import csv
import io
import math
import re

import pytest

LAUNCH = 'src/t870_control/launch/t870_competition.launch.py'


def routes_dir():
    source = io.open(LAUNCH, encoding='utf-8').read()
    return re.search(r"ROUTES = WORKSPACE \+ '([^']+)'", source).group(1)


def launch_csv_names():
    source = io.open(LAUNCH, encoding='utf-8').read()
    return sorted(set(re.findall(r"ROUTES \+ '/([^']+\.csv)'", source)))


def path_for(name):
    source = io.open(LAUNCH, encoding='utf-8').read()
    workspace = re.search(r"WORKSPACE = '([^']+)'", source).group(1)
    return workspace + routes_dir() + '/' + name


def load(name):
    """follower 의 load_route 와 같은 컬럼을 쓴다."""
    with io.open(path_for(name), encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    return [(float(r['easting']), float(r['northing'])) for r in rows]


def cusps(points, degrees=90.0):
    """진행 방향이 크게 꺾이는 지점 = 전/후진 전환점."""
    found = []
    for i in range(1, len(points) - 1):
        ax = points[i][0] - points[i - 1][0]
        ay = points[i][1] - points[i - 1][1]
        bx = points[i + 1][0] - points[i][0]
        by = points[i + 1][1] - points[i][1]
        na, nb = math.hypot(ax, ay), math.hypot(bx, by)
        if na < 1e-6 or nb < 1e-6:
            continue
        cosine = max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))
        if math.degrees(math.acos(cosine)) >= degrees:
            found.append(i)
    return found


@pytest.fixture(scope='module')
def names():
    return launch_csv_names()


def test_launch_references_five_routes(names):
    assert len(names) == 5, names


@pytest.mark.parametrize('name', launch_csv_names())
def test_csv_is_utf8(name):
    """CP949 로 돌아오면 follower 가 UnicodeDecodeError 로 죽는다."""
    raw = open(path_for(name), 'rb').read()
    try:
        raw.decode('utf-8')
    except UnicodeDecodeError as error:
        pytest.fail(
            '%s 가 UTF-8 이 아니다 (%s, offset %d). 변환할 것:\n'
            '  iconv -f cp949 -t utf-8 %s > /tmp/x && mv /tmp/x %s'
            % (name, error.reason, error.start, name, name))


@pytest.mark.parametrize('name', launch_csv_names())
def test_utm_columns_are_complete(name):
    """lat/lon 은 수기 삽입점에서 비어 있어도 되지만 UTM 은 안 된다.

    follower 가 easting/northing 을 먼저 쓴다.
    """
    with io.open(path_for(name), encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    blank = [i for i, r in enumerate(rows)
             if not r['easting'].strip() or not r['northing'].strip()]
    assert not blank, '%s 의 UTM 이 빈 행: %s' % (name, blank[:10])


@pytest.mark.parametrize('name', launch_csv_names())
def test_no_spurious_cusp_at_waypoint_300(name):
    """WP300/301 전치. 재내보내기 때마다 재발했다.

    두 점이 뒤바뀌면 거기에 없는 후진 구간이 생겨 기어가 반대로 들어간다.
    고치는 법: index 값은 그대로 두고 나머지 필드를 맞바꾼다.
    """
    found = [i for i in cusps(load(name)) if i in (300, 301)]
    assert not found, (
        '%s 의 WP%s 가 뒤바뀌었다. index 는 두고 나머지 필드를 교환할 것'
        % (name, found))


# launch 의 후진 구간 설정. start:stop 이면 전환점은 start-1 과 stop 이다.
REVERSE = {'T2.csv': (309, 321), 'P1.csv': (641, 647), 'P2.csv': (649, 655)}


@pytest.mark.parametrize(('name', 'span'), sorted(REVERSE.items()))
def test_reverse_range_matches_the_geometry(name, span):
    start, stop = span
    found = cusps(load(name))
    for shift in (start - 1, stop):
        assert shift in found, (
            '%s 에 WP%d 전환점이 없다. launch 의 %d:%d 와 안 맞는다 '
            '(실제 전환점 %s)' % (name, shift, start, stop, found))


def test_t1_reverse_range_matches_the_geometry():
    assert cusps(load('T1.csv')) == [305, 318], 'launch 는 306:318 을 쓴다'
