# Converted content for erp42_planner_bigob.py
# Similar conversion as erp42_planner_main.py
import rclpy
from rclpy.node import Node
# ... (imports)

class ERPPlannerBigObNode(Node):
    def __init__(self):
        super().__init__('erp42_planner_bigob')
        # ... (conversion of the entire script) ...

def main(args=None):
    rclpy.init(args=args)
    node = ERPPlannerBigObNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()