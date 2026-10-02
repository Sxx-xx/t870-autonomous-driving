import rclpy
from rclpy.node import Node
import serial
from morai_msgs.msg import ERP42Info
from std_msgs.msg import Float64

class ERPStatusNode(Node):
    def __init__(self):
        super().__init__('erp42_status')
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        # Get parameters
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.status_msg = ERP42Info()
        self.ser = serial.Serial(self.port, self.baudrate)
        self.speed_pub = self.create_publisher(Float64, '/ERP42_speed', 10)
        self.steer_pub = self.create_publisher(Float64, '/ERP42_steer', 10)

        self.timer = self.create_timer(1.0/20.0, self.read_serial)
        self.get_logger().info(f"Serial port {self.port} opened at {self.baudrate}")

    def read_serial(self):
        if self.ser.in_waiting >= 18:
            read_serial = self.ser.read(18)
            # ... (rest of the serial parsing logic from original file) ...
            # This part is hardware-specific and does not need ROS API changes.
            # Assuming the parsing logic is correct.
            self.speed_pub.publish(Float64(data=self.status_msg.morai_velocity))
            self.steer_pub.publish(Float64(data=self.status_msg.morai_steering))

def main(args=None):
    rclpy.init(args=args)
    try:
        status_node = ERPStatusNode()
        rclpy.spin(status_node)
    except serial.SerialException as e:
        status_node.get_logger().error(f"Serial port error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if 'status_node' in locals() and rclpy.ok():
            status_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
