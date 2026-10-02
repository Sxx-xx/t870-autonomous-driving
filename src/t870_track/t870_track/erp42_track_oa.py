
import rclpy
from rclpy.node import Node
import threading
import math
from std_msgs.msg import Float64
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, ObjectStatusList
from vehicle_msgs.msg import WaypointsArray, Waypoint, Velocity
from .lib.utils_track import purePursuit_nogps as purePursuit
from .lib.utils_oa import ObstacleAvoidance

class ERPPlannerOA(Node):
    def __init__(self):
        super().__init__('erp42_track_oa')
        self.is_track = False
        self.is_obj = False
        self.is_ego = False

        self.ctrl_msg = CtrlCmd()
        self.track_msg = WaypointsArray()
        self.obj_msg = ObjectStatusList()
        self.ego_msg = EgoVehicleStatus()
        self.curvel_msg = Velocity()

        self.pure_pursuit = purePursuit()
        self.oa = ObstacleAvoidance()

        self.look_steering_point = Waypoint()
        self.look_velocity_point = Waypoint()

        self.ctrl_pub = self.create_publisher(CtrlCmd, '/ctrl_cmd', 10)
        self.steer_way_pub = self.create_publisher(Waypoint, '/steer_waypoint', 10)
        self.vel_way_pub = self.create_publisher(Waypoint, '/velocity_waypoint', 10)

        self.create_subscription(WaypointsArray, "/newwaypoints", self.points_callback, 10)
        self.create_subscription(ObjectStatusList, "/Object_topic", self.obj_callback, 10)
        self.create_subscription(EgoVehicleStatus, "/Ego_topic", self.ego_callback, 10)
        self.create_subscription(Velocity, "/ERP42_velocity", self.velocity_callback, 10)

        self.oa_thread = threading.Thread(target=self.oa_run)
        self.oa_thread.daemon = True
        self.oa_thread.start()

        self.get_logger().info("ERP42 Obstacle Avoidance Tracker Node Started")

    def points_callback(self, msg):
        self.track_msg = msg
        self.is_track = True

    def obj_callback(self, msg):
        self.obj_msg = msg
        self.is_obj = True

    def ego_callback(self, msg):
        self.ego_msg = msg
        self.is_ego = True

    def velocity_callback(self, msg):
        self.curvel_msg = msg

    def oa_run(self):
        rate = self.create_rate(20) # 20hz
        while rclpy.ok():
            if self.is_track and self.is_obj and self.is_ego:
                self.oa.get_ego_status(self.ego_msg)
                self.oa.get_obj_status(self.obj_msg)
                self.pure_pursuit.getVelStatus(self.curvel_msg)

                self.oa_path = self.oa.process(self.track_msg)
                
                self.ctrl_msg.steering, self.look_steering_point = self.pure_pursuit.steering_angle(self.oa_path)
                self.ctrl_msg.velocity, self.look_velocity_point = self.pure_pursuit.velocity_plan(self.oa_path)

                self.ctrl_pub.publish(self.ctrl_msg)
                self.steer_way_pub.publish(self.look_steering_point)
                self.vel_way_pub.publish(self.look_velocity_point)
            else:
                self.get_logger().info("Waiting for messages...", throttle_duration_sec=1)
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    try:
        planner_oa = ERPPlannerOA()
        rclpy.spin(planner_oa)
    except KeyboardInterrupt:
        pass
    finally:
        planner_oa.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
