import rclpy
from rclpy.node import Node
from .MaRRTPathPlanNode_oa import MaRRTPathPlanNodeOA # Local import

def main(args=None):
    rclpy.init(args=args)
    node = MaRRTPathPlanNodeOA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
