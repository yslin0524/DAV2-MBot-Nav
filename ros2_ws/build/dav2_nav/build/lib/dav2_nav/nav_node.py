import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        self.declare_parameter('close_thresh_m', 0.6)
        self.declare_parameter('safe_dist_m', 2.0)
        self.declare_parameter('nogo_thresh', 0.6)
        self.declare_parameter('smooth_kernel', 31)
        self.declare_parameter('min_depth_m', 0.2)
        self.declare_parameter('max_depth_m', 20.0)
        self.declare_parameter('roi_top_frac', 0.0)
        self.declare_parameter('roi_bottom_frac', 0.7)
        self.declare_parameter('decision_period_s', 2.0)
        self.declare_parameter('min_frames_per_decision', 1)
        self.declare_parameter('republish_period_s', 0.5)

        self.close_thresh  = self.get_parameter('close_thresh_m').value
        self.safe_dist     = self.get_parameter('safe_dist_m').value
        self.nogo_thresh   = self.get_parameter('nogo_thresh').value
        self.kernel_size   = self.get_parameter('smooth_kernel').value
        self.min_depth     = self.get_parameter('min_depth_m').value
        self.max_depth     = self.get_parameter('max_depth_m').value
        self.roi_top       = self.get_parameter('roi_top_frac').value
        self.roi_bottom    = self.get_parameter('roi_bottom_frac').value
        self.decision_period = self.get_parameter('decision_period_s').value
        self.min_frames    = self.get_parameter('min_frames_per_decision').value
        self.republish_period = self.get_parameter('republish_period_s').value

        self.bridge = CvBridge()
        self.L_sum = 0.0
        self.C_sum = 0.0
        self.R_sum = 0.0
        self.n_frames = 0
        self.last_dir = 'STOP'

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.sub = self.create_subscription(Image, '/depth/image', self.on_depth, best_effort_qos)
        self.pub_dir = self.create_publisher(String, '/dir', reliable_qos)
        self.create_timer(self.decision_period, self.on_decision_tick)
        self.create_timer(self.republish_period, self.on_republish_tick)
        self.get_logger().info('nav_node started')

    def on_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, '32FC1').astype(np.float32)
            depth = np.nan_to_num(depth, nan=self.max_depth, posinf=self.max_depth)
            depth = np.clip(depth, self.min_depth, self.max_depth)

            # Cost map
            denom = max(self.safe_dist - self.close_thresh, 1e-6)
            cost = np.clip((self.safe_dist - depth) / denom, 0.0, 1.0)
            cost[depth < self.close_thresh] = 1.0

            # Smooth
            k = self.kernel_size | 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)).astype(np.float32)
            kernel /= kernel.sum()
            cost = cv2.filter2D(cost, -1, kernel)

            # ROI
            H, W = cost.shape
            r0 = int(self.roi_top * H)
            r1 = int(self.roi_bottom * H)
            roi = cost[r0:r1, :]

            # Split L/C/R
            L = roi[:, :W//3].mean()
            C = roi[:, W//3:2*W//3].mean()
            R = roi[:, 2*W//3:].mean()

            self.L_sum += L
            self.C_sum += C
            self.R_sum += R
            self.n_frames += 1
        except Exception as e:
            self.get_logger().warn(f'on_depth error: {e}')

    def on_decision_tick(self):
        if self.n_frames < self.min_frames:
            return
        L = self.L_sum / self.n_frames
        C = self.C_sum / self.n_frames
        R = self.R_sum / self.n_frames
        self.L_sum = self.C_sum = self.R_sum = 0.0
        self.n_frames = 0

        if L > self.nogo_thresh and C > self.nogo_thresh and R > self.nogo_thresh:
            self.last_dir = 'STOP'
        else:
            costs = {'FORWARD': C, 'LEFT': L, 'RIGHT': R}
            self.last_dir = min(costs, key=costs.get)

        self.get_logger().info(f'decision: {self.last_dir}  L={L:.3f} C={C:.3f} R={R:.3f}')
        msg = String(); msg.data = self.last_dir
        self.pub_dir.publish(msg)

    def on_republish_tick(self):
        msg = String(); msg.data = self.last_dir
        self.pub_dir.publish(msg)

def main():
    rclpy.init()
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
