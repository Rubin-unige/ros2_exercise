

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtlePubSubNode(Node):

    def __init__(self):
        super().__init__('turtle_pub_sub_node')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.timer = self.create_timer(1.0, self.publish_velocity)
        
    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Publlishing vel')

    def pose_callback(self, msg):
        self.get_logger().info(f'Turtle position: x={msg.x}, y={msg.y}, theta={msg.theta}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePubSubNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()