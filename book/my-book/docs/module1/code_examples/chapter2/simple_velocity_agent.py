import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # For sending velocity commands

class SimpleVelocityAgent(Node):
    def __init__(self):
        super().__init__('simple_velocity_agent')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Simple Velocity Agent started.')

    def timer_callback(self):
        # Create a Twist message to command forward velocity
        msg = Twist()
        msg.linear.x = 0.2  # Move forward at 0.2 m/s
        msg.angular.z = 0.0 # No angular velocity

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing cmd_vel: linear.x=%.2f' % msg.linear.x)

def main(args=None):
    rclpy.init(args=args)
    agent = SimpleVelocityAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
