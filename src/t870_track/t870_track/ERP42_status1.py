
import rclpy
from rclpy.node import Node
from morai_msgs.msg import ERP42Info, CtrlCmd
from vehicle_msgs.msg import Velocity # Assuming vehicle_msgs

class ERPStatusNode(Node):
    def __init__(self):
        super().__init__('erp42_status')
        self.status_msg = ERP42Info()
        self.ctrl_msg = CtrlCmd()
        self.velocity_msg = Velocity()

        self.create_subscription(CtrlCmd, "/ctrl_cmd", self.ctrl_callback, 10)
        self.create_subscription(Velocity, "/ERP42_velocity", self.velocity_callback, 10)
        self.status_pub = self.create_publisher(ERP42Info, '/ERP42_info', 10)
        
        self.timer = self.create_timer(1.0/20.0, self.sender) # 20hz

    def ctrl_callback(self, msg):
        self.ctrl_msg = msg

    def velocity_callback(self, msg):
        self.velocity_msg = msg

    def sender(self):
        self.status_msg.longl_cmd_type = 2
        self.status_msg.morai_accel = self.ctrl_msg.accel
        self.status_msg.morai_brake = self.ctrl_msg.brake
        self.status_msg.morai_steering = self.ctrl_msg.steering
        self.status_msg.morai_velocity = self.velocity_msg.velocity
        
        self.status_pub.publish(self.status_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        status_node = ERPStatusNode()
        rclpy.spin(status_node)
    except KeyboardInterrupt:
        pass
    finally:
        status_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
