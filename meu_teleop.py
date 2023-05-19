import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')

        # Parameter setup
        self.linear_speed = self.declare_parameter('linear_speed', 0.5).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.5).value
        self.acceleration_factor = self.declare_parameter('acceleration_factor', 2.0).value

        # State to track if acceleration is active
        self.acceleration_active = False

        # Subscriber setup
        self.subscription = self.create_subscription(
            String,
            'keyboard_topic',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher setup
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        twist = Twist()

        speed_factor = self.acceleration_factor if self.acceleration_active else 1.0

        if msg.data.lower() == 'w':
            twist.linear.x = self.linear_speed * speed_factor
        elif msg.data.lower() == 's':
            twist.linear.x = -self.linear_speed * speed_factor
        elif msg.data.lower() == 'a':
            twist.linear.y = self.linear_speed * speed_factor
        elif msg.data.lower() == 'd':
            twist.linear.y = -self.linear_speed * speed_factor
        elif msg.data.lower() == 'q':
            twist.angular.z = self.angular_speed * speed_factor
        elif msg.data.lower() == 'e':
            twist.angular.z = -self.angular_speed * speed_factor
        elif msg.data.lower() == ' ':
            twist = Twist()
            self.get_logger().info('Emergency stop activated.')

        if msg.data.lower() in ['w', 's', 'a', 'd', 'q', 'e', ' ']:
            self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    teleop_node = TeleopNode()

    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass

    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
