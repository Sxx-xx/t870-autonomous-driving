# Converted content for erp42_planner_siyeon.py
# Similar conversion as erp42_planner_main.py
import rclpy
from rclpy.node import Node
from v2x_msgs.msg import Spat
# ... (other imports)

class ERPPlannerSiyeonNode(Node):
    def __init__(self):
        super().__init__('erp42_planner_siyeon')
        # ... (conversion of the entire script) ...

def main(args=None):
    rclpy.init(args=args)
    node = ERPPlannerSiyeonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()