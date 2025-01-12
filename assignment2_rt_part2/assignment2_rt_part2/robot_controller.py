import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from assignment2_rt_part2.srv import SetLinearVelocity
import math
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publisher for controlling the robot
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for robot's distance to the fixed point (10, 10)
        self.pub_distance = self.create_publisher(Float64, '/robot_distance', 10)

        # Declare a variable to hold the robot's linear velocity (to be used by both user input and service)
        self.linear_velocity = 0.0

        # Service to modify the linear velocity
        self.srv = self.create_service(SetLinearVelocity, '/set_linear_velocity', self.set_linear_velocity_callback)

        # Subscribe to the robot's odometry (Pose information)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Set a fixed point (10, 10)
        self.fixed_x = 10.0
        self.fixed_y = 10.0

    def control_robot(self):
        """Control the robot's movement based on user input."""
        while rclpy.ok():
            # Get linear velocity with range validation from user
            try:
                self.linear_velocity = float(input("Enter the linear velocity (between -5 and 5): "))
                if -5 <= self.linear_velocity <= 5:
                    break
                else:
                    print("Invalid input. Linear velocity must be between -5 and 5.")
            except ValueError:
                print("Invalid input. Linear velocity must be a number.")

        # Get angular velocity with range validation from user
        while True:
            try:
                angular_z = float(input("Enter the angular velocity (between -5 and 5): "))
                if -5 <= angular_z <= 5:
                    break
                else:
                    print("Invalid input. Angular velocity must be between -5 and 5.")
            except ValueError:
                print("Invalid input. Angular velocity must be a number.")

        # Create a Twist message and populate with user input
        robot_vel = Twist()
        robot_vel.linear.x = self.linear_velocity
        robot_vel.angular.z = angular_z

        # Publish to the cmd_vel topic to move the robot
        self.pub_cmd_vel.publish(robot_vel)
        self.get_logger().info(f"Moving robot with linear velocity {self.linear_velocity} and angular velocity {angular_z}")

        # Simulate robot movement (for 1 second)
        time.sleep(1)  # Sleep for 1 second

        # After moving, stop the robot
        robot_vel.linear.x = 0.0
        robot_vel.angular.z = 0.0
        self.pub_cmd_vel.publish(robot_vel)
        self.get_logger().info("Robot stopped.")

    def set_linear_velocity_callback(self, request, response):
        """Service callback to modify the robot's linear velocity."""
        # Update the robot's linear velocity from the service request
        self.linear_velocity = request.linear_velocity
        response.success = True
        response.message = f"Linear velocity set to {self.linear_velocity}"
        self.get_logger().info(f"Linear velocity updated to {self.linear_velocity}")
        return response

    def odom_callback(self, msg):
        """Callback to receive robot's odometry and calculate distance to fixed point."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Calculate the distance to the fixed point
        distance = self.compute_distance_to_fixed_point(self.x, self.y)

        # Publish the distance
        self.publish_distance(distance)

    def compute_distance_to_fixed_point(self, x_robot, y_robot):
        """Compute the distance to the fixed point (10, 10)."""
        return math.sqrt((x_robot - self.fixed_x)**2 + (y_robot - self.fixed_y)**2)

    def publish_distance(self, distance):
        """Publish the robot's distance to the fixed point (10, 10)."""
        # Create a Float64 message to publish the distance
        msg = Float64()
        msg.data = distance
        self.pub_distance.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    robot_controller = RobotController()

    # Spin the node to listen to service and handle odom callbacks
    try:
        while rclpy.ok():
            robot_controller.control_robot()
            rclpy.spin_once(robot_controller) 
    except KeyboardInterrupt:
        pass  

    # Clean up after node shutdown
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
