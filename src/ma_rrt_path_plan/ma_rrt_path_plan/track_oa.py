import rclpy
from rclpy.node import Node
import time as tm
from math import pi, cos, sin
from vehicle_msgs.msg import Velocity # Assuming vehicle_msgs.msg.Velocity
from morai_msgs.msg import CtrlCmd # Assuming morai_msgs.msg.CtrlCmd
from std_msgs.msg import Float32
from nav_msgs.msg import Path, Odometry

class MakePathOANode(Node): # Renamed class to avoid conflict
    def __init__(self):
        super().__init__('ERP42_track_mission_oa')
        
        self.path_pub = self.create_publisher(Path, '/track_path', 10)
        self.create_subscription(Velocity, "/ERP42_velocity", self.velocity_callback, 10)
        self.create_subscription(Float32, "/ERP42_steer_conv", self.steer_callback, 10)

        self.position_x = 0.0
        self.position_y = 0.0
        self.steer_conv_msg = 0.0
        self.curvel_conv_msg = 0.0
        self.pre_time = self.get_clock().now().nanoseconds / 1e9 # Initialize with current time

        self.timer = self.create_timer(1.0/10.0, self.timer_callback) # Assuming a reasonable rate

    def velocity_callback(self, velocity):
        self.curvel_conv_msg = velocity.velocity # Assuming Velocity has a 'velocity' field

    def steer_callback(self, data):
        self.steer_conv_msg = data.data # Assuming Float32 has a 'data' field

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.pre_time
        v = self.curvel_conv_msg
        d = v * dt
        
        steer = self.steer_conv_msg
        delta = steer * (pi / 180.0)

        self.get_logger().info(f"delta : {delta}")

        if (-28 <= delta and delta < 0):
            delta = abs(delta)
            self.position_x += d * cos(delta)
            self.position_y += d * sin(delta)

        elif (delta == 0):
            self.position_x += d
            # self.position_y remains the same

        elif (0 < delta and delta <= 28):
            delta = delta
            self.position_x += d * cos(delta)
            self.position_y -= d * sin(delta)

        self.get_logger().info(f"point : ({self.position_x}, {self.position_y})")
        
        self.pre_time = current_time
        
        path_msg = Path()
        path_msg.header.frame_id = "map" # Assuming "map" as frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = self.position_x
        pose.pose.position.y = self.position_y
        pose.pose.orientation.w = 1.0 # Assuming no rotation
        path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MakePathOANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()