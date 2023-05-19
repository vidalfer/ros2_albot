import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')

        # configurando os parametros
        self.linear_speed = self.declare_parameter('linear_speed', 0.5).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.5).value
        self.speed_increment = self.declare_parameter('speed_increment', 0.5).value

        # configurando o subscriber
        self.subscription = self.create_subscription(
            String,
            'keyboard_topic',
            self.listener_callback,
            10
        )
        self.subscription

        # configurando o publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Recebido: {msg.data}')

        twist = Twist()

        #incremento de velocidade ao apertar f
        if msg.data.lower() == 'f':
            self.linear_speed += self.speed_increment
            self.angular_speed += self.speed_increment
            self.get_logger().info(f'Velocidade aumentada. Velocidade linear: {self.linear_speed}, Velocidade Angular: {self.angular_speed}')

        if msg.data.lower() == 'w':
            twist.linear.x = self.linear_speed
        elif msg.data.lower() == 's':
            twist.linear.x = -self.linear_speed
        elif msg.data.lower() == 'a':
            twist.linear.y = self.linear_speed
        elif msg.data.lower() == 'd':
            twist.linear.y = -self.linear_speed
        elif msg.data.lower() == 'q':
            twist.angular.z = self.angular_speed
        elif msg.data.lower() == 'e':
            twist.angular.z = -self.angular_speed
        elif msg.data.lower() == ' ':
            twist = Twist()
            self.get_logger().info('Parada de emergência ativada.')

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
