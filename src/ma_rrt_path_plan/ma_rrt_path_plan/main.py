import rclpy
from rclpy.node import Node
from .MaRRTPathPlanNode import MaRRTPathPlanNode # Local import

def main(args=None):
    rclpy.init(args=args)
    node = MaRRTPathPlanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
