import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('stale_timeout', 2.0)
        self.declare_parameter('run_duration', 30.0)

        self.linear_speed  = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.stale_timeout = self.get_parameter('stale_timeout').value
        self.run_duration  = self.get_parameter('run_duration').value

        self.current_dir = 'STOP'
        self.last_dir_time = self.get_clock().now()
        self.start_time = self.get_clock().now()
        self.done_logged = False

        self.sub = self.create_subscription(String, '/dir', self.dir_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.publish_cmd)
        self.get_logger().info(f'controller_node started — will run for {self.run_duration}s')

    def dir_cb(self, msg):
        self.current_dir = msg.data
        self.last_dir_time = self.get_clock().now()

    def publish_cmd(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= self.run_duration:
            self.pub.publish(Twist())
            if not self.done_logged:
                self.get_logger().info('30s run complete — stopped.')
                self.done_logged = True
            return

        stale = (self.get_clock().now() - self.last_dir_time).nanoseconds / 1e9
        if stale > self.stale_timeout:
            self.pub.publish(Twist())
            return

        twist = Twist()
        if self.current_dir == 'FORWARD':
            twist.linear.x = self.linear_speed
        elif self.current_dir == 'LEFT':
            twist.angular.z = self.angular_speed
        elif self.current_dir == 'RIGHT':
            twist.angular.z = -self.angular_speed
        self.pub.publish(twist)

    def publish_zero(self):
        self.pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()
