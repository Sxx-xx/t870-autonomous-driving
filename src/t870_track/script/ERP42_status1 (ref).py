#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from control_msgs.msg import Velocity, Gear
from morai_msgs.msg import CtrlCmd
import serial
import struct

# Set initial parameters
rospy.set_param('PORT', '/dev/ttyUSB0')
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
    SPEED1 = struct.pack("B", int(speed_s))
    SPEED0 = b"\x00"
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
    # 브레이크 값을 0에서 255 사이로 제한합니다.
    clamped_brake = max(0, min(255, int(brake_s)))
    return struct.pack("B", clamped_brake)


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
    return speed0_b[0] + (speed1_b[0] << 8)

def Steer_Conv(steer0_b, steer1_b):
    steer = (steer0_b[0] << 8) | steer1_b[0]
    if steer & 0x8000:
        steer -= 0x10000
    return steer

def Encoder_Conv(enc0_b, enc1_b, enc2_b, enc3_b):
    encoder = enc0_b[0] | (enc1_b[0] << 8) | (enc2_b[0] << 16) | (enc3_b[0] << 24)
    if encoder & 0x80000000:
        encoder -= 0x100000000
    return encoder

def Parsing():
    global speed, gear, steer, encoder

    Pdata = ser.read(18)
    if len(Pdata) == 18 and Pdata.startswith(START_BITS) and Pdata.endswith(END_BITS):
        GEAR = Pdata[5:6]
        SPEED0 = Pdata[6:7]
        SPEED1 = Pdata[7:8]
        STEER0 = Pdata[8:9]
        STEER1 = Pdata[9:10]
        ENC0 = Pdata[11:12]
        ENC1 = Pdata[12:13]
        ENC2 = Pdata[13:14]
        ENC3 = Pdata[14:15]

        gear = Gear_Conv(GEAR)
        speed = Speed_Conv(SPEED0, SPEED1)
        steer = Steer_Conv(STEER0, STEER1)
        encoder = Encoder_Conv(ENC0, ENC1, ENC2, ENC3)

gear_s = 0
speed_s = 0
steer_s = 0
brake_s = 0
# encoder_s = 0

def cmd_callback(data):
    global gear_s, speed_s, steer_s, brake_s, encoder_s
    gear_s = int(data.gear)
    speed_s = data.accel  # speed
    steer_s = data.steering
    brake_s = data.brake
    # encoder_s = data.encoder
    rospy.loginfo(f"Received cmd: gear: {gear_s}, speed: {speed_s}, steer: {steer_s}, brake: {brake_s}")

if __name__ == '__main__':
    rospy.init_node('ERP42_status')
    pub_steer_erp42 = rospy.Publisher('/ERP42_steer', Float32, queue_size=10)
    pub_steer_erp42_conv = rospy.Publisher('/ERP42_steer_conv', Float32, queue_size=10)
    pub_speed_erp42 = rospy.Publisher('/ERP42_velocity', Velocity, queue_size=10)
    pub_gear_erp42 = rospy.Publisher('/ERP42_gear', Gear, queue_size=10)
    pub_encoder_erp42 = rospy.Publisher('/ERP42_encoder',Float32, queue_size=10)
    rospy.Subscriber('/ctrl_cmd', CtrlCmd, cmd_callback)
    rate = rospy.Rate(20)

    ser = serial.Serial(rospy.get_param('PORT'), baudrate=115200, timeout=None, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

    while ser.isOpen() and not rospy.is_shutdown():
        Parsing()
        pub_speed_erp42.publish(speed)
        pub_steer_erp42.publish(steer)
        pub_steer_erp42_conv.publish(steer / 71)
        pub_gear_erp42.publish(gear)
        # pub_encoder_erp42.publish(encoder)
        Send_to_ERP42(gear_s, speed_s, steer_s, brake_s)
        rate.sleep()