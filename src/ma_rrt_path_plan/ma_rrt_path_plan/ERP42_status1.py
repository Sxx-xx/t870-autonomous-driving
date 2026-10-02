import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import Float32
from vehicle_msgs.msg import Velocity # Assuming vehicle_msgs.msg.Velocity
from morai_msgs.msg import CtrlCmd # Assuming morai_msgs.msg.CtrlCmd

class ERP42StatusNode(Node):
    def __init__(self):
        super().__init__('ERP42_status')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.START_BITS = b"\x53\x54\x58"
        self.END_BITS = b"\x0D\x0A"
        self.count_alive = 0

        self.ser = serial.Serial(self.port, self.baudrate, timeout=None, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        
        self.pub_steer_erp42 = self.create_publisher(Float32, '/ERP42_steer', 10)
        self.pub_steer_erp42_conv = self.create_publisher(Float32, '/ERP42_steer_conv', 10)
        self.pub_speed_erp42 = self.create_publisher(Velocity, '/ERP42_velocity', 10) # Assuming Velocity msg
        self.pub_gear_erp42 = self.create_publisher(Float32, '/ERP42_gear', 10) # Assuming Gear msg is Float32

        self.create_subscription(CtrlCmd, '/ctrl_cmd', self.cmd_callback, 10)
        
        self.timer = self.create_timer(1.0/40.0, self.timer_callback) # Original rate was 40Hz

        self.get_logger().info(f"Serial port {self.port} opened at {self.baudrate}")

        self.gear_s = 0
        self.speed_s = 0
        self.steer_s = 0
        self.brake_s = 0

        self.speed = 0
        self.gear = 0
        self.steer = 0
        self.encoder = 0

    def GetAorM(self):
        return b"\x01"

    def GetESTOP(self):
        return b"\x00"

    def GetGEAR(self, gear_s):
        return struct.pack("B", gear_s)

    def GetSPEED(self, speed_s):
        SPEED1 = struct.pack("B", int(speed_s))
        SPEED0 = b"\x00"
        return SPEED0, SPEED1

    def GetSTEER(self, steer_s):
        steer_s = int(steer_s * 71)
        steer_max = 2000
        steer_min = -2000

        if steer_s > steer_max:
            steer_s = steer_max
        elif steer_s < steer_min:
            steer_s = steer_min

        STEER = struct.pack(">h", steer_s)
        return STEER[0:1], STEER[1:2]

    def GetBRAKE(self, brake_s):
        return struct.pack("B", int(brake_s))

    def Send_to_ERP42(self, gear_s, speed_s, steer_s, brake_s):
        self.count_alive = (self.count_alive + 1) % 256

        AorM = self.GetAorM()
        ESTOP = self.GetESTOP()
        GEAR = self.GetGEAR(gear_s)
        SPEED0, SPEED1 = self.GetSPEED(speed_s)
        STEER0, STEER1 = self.GetSTEER(steer_s)
        BRAKE = self.GetBRAKE(brake_s)
        ALIVE = struct.pack("B", self.count_alive)

        packet = self.START_BITS + AorM + ESTOP + GEAR + SPEED0 + SPEED1 + STEER0 + STEER1 + BRAKE + ALIVE + self.END_BITS
        self.ser.write(packet)

    def Gear_Conv(self, gear_b):
        return gear_b[0]

    def Speed_Conv(self, speed0_b, speed1_b):
        return speed0_b[0] + (speed1_b[0] << 8)

    def Steer_Conv(self, steer0_b, steer1_b):
        steer = (steer0_b[0] << 8) | steer1_b[0]
        if steer & 0x8000:
            steer -= 0x10000
        return steer

    def Encoder_Conv(self, enc0_b, enc1_b, enc2_b, enc3_b):
        encoder = enc0_b[0] | (enc1_b[0] << 8) | (enc2_b[0] << 16) | (enc3_b[0] << 24)
        if encoder & 0x80000000:
            encoder -= 0x100000000
        return encoder

    def Parsing(self):
        Pdata = self.ser.read(18)
        if len(Pdata) == 18 and Pdata.startswith(self.START_BITS) and Pdata.endswith(self.END_BITS):
            GEAR = Pdata[5:6]
            SPEED0 = Pdata[6:7]
            SPEED1 = Pdata[7:8]
            STEER0 = Pdata[8:9]
            STEER1 = Pdata[9:10]
            ENC0 = Pdata[11:12]
            ENC1 = Pdata[12:13]
            ENC2 = Pdata[13:14]
            ENC3 = Pdata[14:15]

            self.gear = self.Gear_Conv(GEAR)
            self.speed = self.Speed_Conv(SPEED0, SPEED1)
            self.steer = self.Steer_Conv(STEER0, STEER1)
            self.encoder = self.Encoder_Conv(ENC0, ENC1, ENC2, ENC3)

    def cmd_callback(self, data):
        self.gear_s = int(data.gear)
        self.speed_s = data.accel
        self.steer_s = data.steering
        self.brake_s = data.brake
        self.get_logger().info(f"Received cmd: gear: {self.gear_s}, speed: {self.speed_s}, steer: {self.steer_s}, brake: {self.brake_s}")

    def timer_callback(self):
        if self.ser.isOpen():
            self.Parsing()
            self.pub_speed_erp42.publish(Velocity(velocity=float(self.speed))) # Assuming Velocity has a 'velocity' field
            self.pub_steer_erp42.publish(Float32(data=float(self.steer)))
            self.pub_steer_erp42_conv.publish(Float32(data=float(self.steer / 71.0)))
            self.pub_gear_erp42.publish(Float32(data=float(self.gear))) # Assuming Gear msg is Float32
            self.Send_to_ERP42(self.gear_s, self.speed_s, self.steer_s, self.brake_s)
        else:
            self.get_logger().warn("Serial port is not open.")

def main(args=None):
    rclpy.init(args=args)
    try:
        erp_status_node = ERP42StatusNode()
        rclpy.spin(erp_status_node)
    except serial.SerialException as e:
        erp_status_node.get_logger().error(f"Serial port error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            erp_status_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
