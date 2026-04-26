import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('stop_on_stale', 5.0)
        self.declare_parameter('control_rate_hz', 20.0)

        self.linear_speed  = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.stop_on_stale = self.get_parameter('stop_on_stale').value
        rate_hz            = self.get_parameter('control_rate_hz').value

        self.last_action   = 'STOP'
        self.last_dir_time = None

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.sub = self.create_subscription(String, '/dir', self.on_dir, reliable_qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self.create_timer(1.0 / rate_hz, self.on_tick)
        self.get_logger().info('controller_node started')

    def on_dir(self, msg):
        self.last_action = msg.data
        self.last_dir_time = self.get_clock().now()

    def on_tick(self):
        # Stale check
        if self.last_dir_time is not None:
            age = (self.get_clock().now() - self.last_dir_time).nanoseconds / 1e9
            if age > self.stop_on_stale:
                self.last_action = 'STOP'

        twist = Twist()
        if self.last_action == 'FORWARD':
            twist.linear.x = self.linear_speed
        elif self.last_action == 'LEFT':
            twist.angular.z = self.angular_speed
        elif self.last_action == 'RIGHT':
            twist.angular.z = -self.angular_speed

        self.pub.publish(twist)

    def publish_zero(self):
        self.pub.publish(Twist())

def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()
