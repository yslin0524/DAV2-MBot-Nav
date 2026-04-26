import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class DAV2Node(Node):
    def __init__(self):
        super().__init__('dav2_node')
        self.declare_parameter('encoder', 'vits')
        self.declare_parameter('checkpoint_path', 'checkpoints/depth_anything_v2_metric_hypersim_vits.pth')
        self.declare_parameter('max_depth', 20.0)
        self.declare_parameter('input_size', 518)
        self.declare_parameter('device', 'auto')

        encoder    = self.get_parameter('encoder').value
        ckpt_path  = self.get_parameter('checkpoint_path').value
        max_depth  = self.get_parameter('max_depth').value
        input_size = self.get_parameter('input_size').value
        device_str = self.get_parameter('device').value

        repo_dir = os.path.expanduser('~/DA-V2-for-RobotNav')
        sys.path.insert(0, os.path.join(repo_dir, 'metric_depth'))
        sys.path.insert(1, repo_dir)

        import torch
        from depth_anything_v2.dpt import DepthAnythingV2

        if device_str == 'auto':
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = device_str

        model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        }

        ckpt_full = os.path.join(repo_dir, ckpt_path)
        self.model = DepthAnythingV2(**{**model_configs[encoder], 'max_depth': max_depth})
        self.model.load_state_dict(torch.load(ckpt_full, map_location='cpu'))
        self.model = self.model.to(self.device).eval()
        self.input_size = input_size
        self.bridge = CvBridge()
        self.frame_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.sub = self.create_subscription(Image, '/image_raw', self.on_image, qos)
        self.pub = self.create_publisher(Image, '/depth/image', qos)
        self.get_logger().info(f'dav2_node started on {self.device}')

    def on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            depth = self.model.infer_image(bgr, self.input_size)
            depth_msg = self.bridge.cv2_to_imgmsg(depth.astype(np.float32), '32FC1')
            depth_msg.header = msg.header
            self.pub.publish(depth_msg)
            self.frame_count += 1
            if self.frame_count % 10 == 0:
                self.get_logger().info(f'Frame {self.frame_count} published')
        except Exception as e:
            self.get_logger().warn(f'Error: {e}')

def main():
    rclpy.init()
    node = DAV2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
