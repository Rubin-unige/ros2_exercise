import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import time

class TurtleUserInterface(Node):
    def __init__(self):
        super().__init__('turtle_user_interface')

        # Publishers for turtle1 and turtle2
        self.pub_turtle1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Service clients for spawn and kill services
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Empty, '/kill')

        # Check if turtle2 exists and spawn if necessary
        if not self.check_if_turtle2_exists():
            self.spawn_turtle()

    def check_if_turtle2_exists(self):
        """Checks if turtle2 exists by subscribing to /turtle2/pose."""
        turtle2_exists = False

        def pose_callback(msg):
            nonlocal turtle2_exists
            turtle2_exists = True  # Message received; turtle2 exists

        # Subscribe to /turtle2/pose topic
        self.create_subscription(Pose, '/turtle2/pose', pose_callback, 10)

        # Short delay, wait for message
        timeout_time = self.get_clock().now() + rclpy.duration.Duration(seconds=1.0)
        while self.get_clock().now() < timeout_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)  # Spin in small increments

        return turtle2_exists

    def spawn_turtle(self):
        """Spawns turtle2 if it doesn't exist."""
        self.get_logger().info("Spawning Turtle2.")
        while not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Spawn service not available, waiting again...")

        request = Spawn.Request()
        request.x = 5.0
        request.y = 2.0
        request.theta = 0.0
        request.name = 'turtle2'
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Turtle2 spawned successfully.")
        else:
            self.get_logger().error("Failed to spawn turtle2.")

    def main_loop(self):

        while rclpy.ok():
            # Prompt user for turtle selection
            turtle_name = input("Enter the turtle you want to control (turtle1 or turtle2): ")

            # Validate turtle name
            if turtle_name not in ["turtle1", "turtle2"]:
                print("Invalid turtle name. Please enter 'turtle1' or 'turtle2'.")
                continue

            # Get linear velocity with range validation
            while True:
                try:
                    linear_x = float(input("Enter the linear velocity (between -5 and 5): "))
                    if -5 <= linear_x <= 5:
                        break
                    else:
                        print("Invalid input. Linear velocity must be between -5 and 5.")
                except ValueError:
                    print("Invalid input. Linear velocity must be a number.")

            # Get angular velocity with range validation
            while True:
                try:
                    angular_z = float(input("Enter the angular velocity (between -5 and 5): "))
                    if -5 <= angular_z <= 5:
                        break
                    else:
                        print("Invalid input. Angular velocity must be between -5 and 5.")
                except ValueError:
                    print("Invalid input. Angular velocity must be a number.")

            # Create and populate Twist message
            turtle_vel = Twist()
            turtle_vel.linear.x = linear_x
            turtle_vel.angular.z = angular_z

            # Publish to the selected turtle
            if turtle_name == "turtle1":
                self.pub_turtle1.publish(turtle_vel)
            elif turtle_name == "turtle2":
                self.pub_turtle2.publish(turtle_vel)

            # Move the turtle for 1 second
            time.sleep(1.0)

            # Stop the turtle
            turtle_vel.linear.x = 0.0
            turtle_vel.angular.z = 0.0
            if turtle_name == "turtle1":
                self.pub_turtle1.publish(turtle_vel)
            elif turtle_name == "turtle2":
                self.pub_turtle2.publish(turtle_vel)

            return


def main(args=None):
    rclpy.init(args=args)

    ui_node = TurtleUserInterface()
   
    while rclpy.ok():
        ui_node.main_loop()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
