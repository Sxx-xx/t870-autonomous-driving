#!/usr/bin/env python3
"""u-blox 수신기 설정을 읽고, 필요하면 동적 모델을 바꾼다.

u-center 는 윈도우 전용이라 이 노트북에서 못 쓴다. 대신 UBX 이진
메시지를 직접 만들어 USB 로 주고받는다. 외부 라이브러리가 필요 없다.

기본은 읽기만 한다. 바꾸려면 --set-automotive 를 명시해야 한다.
수신기 설정은 되돌리기 번거로우므로 함부로 쓰지 않는다.

쓰는 법 (ROS 스택은 꺼야 한다. 시리얼은 하나만 열린다):
    /usr/bin/python3 ubx_config.py                    # 현재 설정 보기
    /usr/bin/python3 ubx_config.py --set-automotive   # 동적 모델 변경 + 플래시 저장
    /usr/bin/python3 ubx_config.py --set-automotive --no-save   # 저장 없이 (전원 끄면 원복)
"""
import argparse
import struct
import sys
import time

import serial

PORT = ('/dev/serial/by-id/'
        'usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00')

DYN_MODEL = {
    0: 'Portable', 2: 'Stationary', 3: 'Pedestrian', 4: 'Automotive',
    5: 'Sea', 6: 'Airborne <1g', 7: 'Airborne <2g', 8: 'Airborne <4g',
    9: 'Wrist', 10: 'Bike', 11: 'Mower', 12: 'E-scooter',
}
FIX_MODE = {1: '2D only', 2: '3D only', 3: 'Auto 2D/3D'}
GNSS_ID = {0: 'GPS', 1: 'SBAS', 2: 'Galileo', 3: 'BeiDou',
           4: 'IMES', 5: 'QZSS', 6: 'GLONASS', 7: 'NavIC'}


def checksum(payload):
    a = b = 0
    for byte in payload:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return bytes((a, b))


def frame(cls, msg_id, payload=b''):
    body = bytes((cls, msg_id)) + struct.pack('<H', len(payload)) + payload
    return b'\xb5\x62' + body + checksum(body)


def read_reply(port, cls, msg_id, timeout=2.0):
    """UBX 한 프레임을 기다린다. NMEA 가 섞여 흐르므로 동기 바이트를 찾는다."""
    end = time.time() + timeout
    buffer = b''
    while time.time() < end:
        chunk = port.read(port.in_waiting or 1)
        if not chunk:
            continue
        buffer += chunk
        while True:
            start = buffer.find(b'\xb5\x62')
            if start < 0 or len(buffer) < start + 6:
                break
            length = struct.unpack('<H', buffer[start + 4:start + 6])[0]
            total = start + 6 + length + 2
            if len(buffer) < total:
                break
            packet = buffer[start:total]
            buffer = buffer[total:]
            if packet[2] == cls and packet[3] == msg_id:
                return packet[6:6 + length]
    return None


def wait_ack(port, cls, msg_id, timeout=2.0):
    end = time.time() + timeout
    while time.time() < end:
        payload = read_reply(port, 0x05, 0x01, timeout=end - time.time())
        if payload and len(payload) >= 2:
            if payload[0] == cls and payload[1] == msg_id:
                return True
        nak = read_reply(port, 0x05, 0x00, timeout=0.2)
        if nak and len(nak) >= 2 and nak[0] == cls and nak[1] == msg_id:
            return False
    return None


def show_nav5(port):
    port.write(frame(0x06, 0x24))
    payload = read_reply(port, 0x06, 0x24)
    if payload is None or len(payload) < 36:
        print('  CFG-NAV5 응답 없음')
        return None
    dyn = payload[2]
    fix_mode = payload[3]
    min_elev = struct.unpack('<b', payload[12:13])[0]
    print('  동적 모델        : %d (%s)%s'
          % (dyn, DYN_MODEL.get(dyn, '?'),
             '   <- 코너에서 튈 수 있다' if dyn != 4 else '   <- 차량용, 맞다'))
    print('  측위 모드        : %d (%s)' % (fix_mode, FIX_MODE.get(fix_mode, '?')))
    print('  최소 위성 고도각 : %d 도' % min_elev)
    return dyn


def show_gnss(port):
    port.write(frame(0x06, 0x3E))
    payload = read_reply(port, 0x06, 0x3E)
    if payload is None or len(payload) < 4:
        print('  CFG-GNSS 응답 없음 (최신 펌웨어는 이 메시지를 안 받는다)')
        return
    blocks = payload[3]
    on = []
    for i in range(blocks):
        block = payload[4 + i * 8:12 + i * 8]
        if len(block) < 8:
            break
        gnss_id, min_ch, max_ch = block[0], block[1], block[2]
        flags = struct.unpack('<I', block[4:8])[0]
        state = '켜짐' if flags & 1 else '꺼짐'
        print('    %-8s %s  (채널 %d~%d)'
              % (GNSS_ID.get(gnss_id, gnss_id), state, min_ch, max_ch))
        if flags & 1:
            on.append(GNSS_ID.get(gnss_id, gnss_id))
    print('  켜진 위성군: %s' % ', '.join(on))


def set_model(port, model, save):
    # mask=0x0001 -> dynModel 만 적용한다. 나머지 필드는 건드리지 않는다.
    payload = bytearray(36)
    struct.pack_into('<H', payload, 0, 0x0001)
    payload[2] = model
    port.write(frame(0x06, 0x24, bytes(payload)))
    result = wait_ack(port, 0x06, 0x24)
    print('  동적 모델 -> %-12s: %s'
          % (DYN_MODEL.get(model, model),
             {True: 'ACK (적용됨)', False: 'NAK (거부됨)',
              None: '응답 없음'}[result]))
    if not save or result is not True:
        return result is True
    # CFG-CFG: 현재 설정을 BBR + Flash + EEPROM + SPI flash 에 저장한다.
    save_payload = struct.pack('<IIIB', 0x00000000, 0x0000FFFF, 0x00000000, 0x17)
    port.write(frame(0x06, 0x09, save_payload))
    saved = wait_ack(port, 0x06, 0x09)
    print('  플래시 저장             : %s'
          % {True: 'ACK (전원 꺼도 유지)', False: 'NAK',
             None: '응답 없음'}[saved])
    return saved is True


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--set-automotive', action='store_true',
                        help='동적 모델을 Automotive(4) 로 바꾼다')
    parser.add_argument('--set-model', type=int, metavar='N',
                        help='동적 모델 번호를 직접 지정 (되돌릴 때 2=Stationary)')
    parser.add_argument('--no-save', action='store_true',
                        help='플래시에 저장하지 않는다 (전원 끄면 원복)')
    arguments = parser.parse_args()

    try:
        port = serial.Serial(arguments.port, 115200, timeout=0.5)
    except (OSError, serial.SerialException) as error:
        raise SystemExit('시리얼을 못 열었다 (ROS 스택이 물고 있나?): %s' % error)

    with port:
        port.reset_input_buffer()
        print('현재 설정')
        print('-' * 52)
        show_nav5(port)
        print()
        print('  위성군')
        show_gnss(port)
        model = 4 if arguments.set_automotive else arguments.set_model
        if model is None:
            print()
            print('바꾸려면: /usr/bin/python3 %s --set-automotive'
                  % sys.argv[0].split('/')[-1])
            return
        print()
        print('변경')
        print('-' * 52)
        set_model(port, model, not arguments.no_save)
        print()
        print('변경 후 설정')
        print('-' * 52)
        time.sleep(0.3)
        port.reset_input_buffer()
        show_nav5(port)


if __name__ == '__main__':
    main()
