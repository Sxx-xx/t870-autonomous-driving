#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from control_msgs.msg import Velocity, Gear
from morai_msgs.msg import CtrlCmd
import serial
import struct
import threading

# Set initial parameters
rospy.set_param('PORT', '/dev/ttyUSB1')
START_BITS = b"\x53\x54\x58"
END_BITS = b"\x0D\x0A"
count_alive = 0

def GetAorM():
    return b"\x01"

def GetESTOP():
    return b"\x00"

def GetGEAR(gear_s):
    return struct.pack("B", gear_s)

def GetSPEED(speed_s):
    speed_abs = abs(int(speed_s))
    speed_abs = min(speed_abs, 255)  # Limit to 255
    SPEED1 = struct.pack("B", speed_abs)
    SPEED0 = b"\x01" if speed_s < 0 else b"\x00"  # Use SPEED0 for direction
    return SPEED0, SPEED1

def GetSTEER(steer_s):
    steer_s = int(steer_s * 71)
    steer_max = 2000
    steer_min = -2000

    if steer_s > steer_max:
        steer_s = steer_max
    elif steer_s < steer_min:
        steer_s = steer_min

    STEER = struct.pack(">h", steer_s)
    return STEER[0:1], STEER[1:2]

def GetBRAKE(brake_s):
    return struct.pack("B", int(brake_s))

def Send_to_ERP42(gear_s, speed_s, steer_s, brake_s):
    global count_alive
    count_alive = (count_alive + 1) % 256

    AorM = GetAorM()
    ESTOP = GetESTOP()
    GEAR = GetGEAR(gear_s)
    SPEED0, SPEED1 = GetSPEED(speed_s)
    STEER0, STEER1 = GetSTEER(steer_s)
    BRAKE = GetBRAKE(brake_s)
    ALIVE = struct.pack("B", count_alive)

    packet = START_BITS + AorM + ESTOP + GEAR + SPEED0 + SPEED1 + STEER0 + STEER1 + BRAKE + ALIVE + END_BITS
    ser.write(packet)

def Gear_Conv(gear_b):
    return gear_b[0]

def Speed_Conv(speed0_b, speed1_b):
    speed = speed0_b[0] + (speed1_b[0] << 8)
    return -speed if speed0_b[0] & 0x80 else speed

def Steer_Conv(steer_bytes):
    steer = struct.unpack(">h", steer_bytes)[0]
    return steer

def Encoder_Conv(enc0_b, enc1_b, enc2_b, enc3_b):
    encoder = enc0_b[0] | (enc1_b[0] << 8) | (enc2_b[0] << 16) | (enc3_b[0] << 24)
    if encoder & 0x80000000:
        encoder -= 0x100000000
    return encoder
# 초기 변수 설정
gear = 0
speed = 0
steer = 0
encoder = 0

gear_s = 0
speed_s = 0
steer_s = 0
brake_s = 0

# def cmd_callback(data):
#     global gear_s, speed_s, steer_s, brake_s
#     gear_s = int(data.gear)
#     speed_s = data.accel  # speed
#     steer_s = data.steering
#     brake_s = data.brake
#     rospy.loginfo(f"Received cmd: gear: {gear_s}, speed: {speed_s}, steer: {steer_s}, brake: {brake_s}")

def Parsing():
    global speed, gear, steer, encoder
    buffer = b""
    while not rospy.is_shutdown():
        # 헤더를 찾을 때까지 1바이트씩 읽기
        while len(buffer) < 3 or buffer[-3:] != START_BITS:
            byte = ser.read(1)
            if not byte:
                continue
            buffer += byte
            if len(buffer) > 3:
                buffer = buffer[-3:]

        # 헤더를 찾았으면 나머지 데이터 읽기
        remaining = ser.read(15)  # 18 - 3(START_BITS)
        if len(remaining) != 15:
            continue

        Pdata = buffer + remaining

        if Pdata.endswith(END_BITS):
            AorM = Pdata[3:4]
            ESTOP = Pdata[4:5]
            GEAR = Pdata[5:6]
            SPEED0 = Pdata[6:7]
            SPEED1 = Pdata[7:8]
            STEER0 = Pdata[8:9]
            STEER1 = Pdata[9:10]
            BRAKE = Pdata[10:11]
            ENC0 = Pdata[11:12]
            ENC1 = Pdata[12:13]
            ENC2 = Pdata[13:14]
            ENC3 = Pdata[14:15]
            ALIVE = Pdata[15:16]

            gear = Gear_Conv(GEAR)
            speed = Speed_Conv(SPEED0, SPEED1)
            steer = Steer_Conv(STEER0 + STEER1)
            encoder = Encoder_Conv(ENC0, ENC1, ENC2, ENC3)

            rospy.loginfo(f"Parsed data: gear={gear}, speed={speed}, steer={steer}, encoder={encoder}")
        else:
            rospy.logwarn("Invalid packet received")

        buffer = b""  # 버퍼 초기화

if __name__ == '__main__':
    rospy.init_node('ERP42_status')
    pub_steer_erp42 = rospy.Publisher('/ERP42_steer', Float32, queue_size=10)
    pub_steer_erp42_conv = rospy.Publisher('/ERP42_steer_conv', Float32, queue_size=10)
    pub_speed_erp42 = rospy.Publisher('/ERP42_velocity', Velocity, queue_size=10) #Velocity
    pub_gear_erp42 = rospy.Publisher('/ERP42_gear', Gear, queue_size=10)
    # rospy.Subscriber('/ctrl_cmd', CtrlCmd, cmd_callback)
    rate = rospy.Rate(20)

    ser = serial.Serial(rospy.get_param('PORT'), baudrate=115200, timeout=None, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
    
    # 시리얼 데이터 수신 스레드 시작
    serial_thread = threading.Thread(target=Parsing)
    serial_thread.daemon = True
    serial_thread.start()

    while ser.isOpen() and not rospy.is_shutdown():
        pub_speed_erp42.publish(speed)
        pub_steer_erp42.publish(steer)
        pub_steer_erp42_conv.publish(steer / 71)
        pub_gear_erp42.publish(gear)
        Send_to_ERP42(gear_s, 40, 0.2, brake_s)
        rate.sleep()