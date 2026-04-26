import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        self.declare_parameter('close_thresh_m', 0.5)
        self.declare_parameter('safe_dist_m', 1.5)
        self.declare_parameter('nogo_thresh', 0.75)
        self.declare_parameter('roi_bottom_frac', 0.7)

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Image, '/depth/image',
                                            self.depth_cb, qos)
        self.pub = self.create_publisher(String, '/dir', 10)
        self.get_logger().info('nav_node started')

    def depth_cb(self, msg):
        # 每收到一張深度圖就立刻決策，不用 timer
        h, w = msg.height, msg.width
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        roi_top = int(h * (1.0 - self.get_parameter('roi_bottom_frac').value))
        arr = arr[roi_top:, :]

        close = self.get_parameter('close_thresh_m').value
        safe  = self.get_parameter('safe_dist_m').value
        cost  = np.clip((safe - arr) / (safe - close), 0.0, 1.0)

        third = w // 3
        l = float(np.mean(cost[:, :third]))
        c = float(np.mean(cost[:, third:2*third]))
        r = float(np.mean(cost[:, 2*third:]))

        nogo = self.get_parameter('nogo_thresh').value
        turn_threshold = 0.06

        if min(l, c, r) >= nogo:
            direction = 'STOP'
        elif l < c - turn_threshold and l <= r:
            direction = 'LEFT'
        elif r < c - turn_threshold and r < l:
            direction = 'RIGHT'
        else:
            direction = 'FORWARD'

        self.get_logger().info(
            f'decision: {direction}  L={l:.3f} C={c:.3f} R={r:.3f}')
        msg = String(); msg.data = direction
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
