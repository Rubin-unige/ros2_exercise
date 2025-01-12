import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32  # Using Float32 to publish the distance
import math

# Global variables to store turtle positions
turtle1_x = 0.0
turtle1_y = 0.0
turtle2_x = 0.0
turtle2_y = 0.0

# Constants for distance and boundary checks
distance_threshold = 2.0
boundary_limit = 1.0
max_limit = 10.0

class TurtleDistanceMonitor(Node):
    def __init__(self):
        super().__init__('turtle_distance_monitor')

        # Subscribers for turtle positions
        self.create_subscription(Pose, "/turtle1/pose", self.turtle1_pose_callback, 10)
        self.create_subscription(Pose, "/turtle2/pose", self.turtle2_pose_callback, 10)

        # Publishers for controlling turtle velocities
        self.pub_turtle1 = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pub_turtle2 = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

        # Publisher for the distance (using Float32 for simplicity)
        self.pub_distance = self.create_publisher(Float32, "/turtle_distance_topic", 10)

    def turtle1_pose_callback(self, msg):
        """Callback for turtle1 pose."""
        global turtle1_x, turtle1_y
        turtle1_x = msg.x
        turtle1_y = msg.y

    def turtle2_pose_callback(self, msg):
        """Callback for turtle2 pose."""
        global turtle2_x, turtle2_y
        turtle2_x = msg.x
        turtle2_y = msg.y

    def is_near_boundary(self, x, y):
        """Check if the turtle is near the boundary."""
        return (x <= boundary_limit or x >= max_limit or y <= boundary_limit or y >= max_limit)

    def check_if_overshot_boundary(self, x, y):
        """Check if the turtle has overshot the boundary."""
        return (x > max_limit or x < boundary_limit or y > max_limit or y < boundary_limit)

    def stop_turtle(self, pub):
        """Stop the turtle by publishing zero velocity."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        pub.publish(stop_msg)

    def monitor_turtles(self):
        """Continuously monitor the turtles' distance and boundary status."""
        global turtle1_x, turtle1_y, turtle2_x, turtle2_y
        
        # Calculate the distance between turtles
        distance = math.sqrt((turtle2_x - turtle1_x) ** 2 + (turtle2_y - turtle1_y) ** 2)

        # Check if turtles are too close
        is_too_close = (distance <= distance_threshold)

        # Check if turtles are near boundaries
        turtle1_near_boundary = self.is_near_boundary(turtle1_x, turtle1_y)
        turtle2_near_boundary = self.is_near_boundary(turtle2_x, turtle2_y)

        # Publish the distance as a Float32 message
        dist_msg = Float32()
        dist_msg.data = distance
        self.pub_distance.publish(dist_msg)

        # If turtles are too close or near boundary, stop them
        if is_too_close or turtle1_near_boundary or turtle2_near_boundary:
            if is_too_close:
                self.stop_turtle(self.pub_turtle1)  # Stop turtle1
                self.stop_turtle(self.pub_turtle2)  # Stop turtle2
                if (distance < distance_threshold):
                    self.get_logger().warn("Turtles are really too close!!!")

            if turtle1_near_boundary:
                self.stop_turtle(self.pub_turtle1)  # Stop turtle1

            if turtle2_near_boundary:
                self.stop_turtle(self.pub_turtle2)  # Stop turtle2

        # After stopping the turtles, check if they have overshot the boundary
        turtle1_overshot = self.check_if_overshot_boundary(turtle1_x, turtle1_y)
        turtle2_overshot = self.check_if_overshot_boundary(turtle2_x, turtle2_y)

        # Handle overshoot
        if turtle1_overshot:
            self.get_logger().warn("Turtle1 is over the boundary after stopping!")

        if turtle2_overshot:
            self.get_logger().warn("Turtle2 is over the boundary after stopping!")

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the TurtleDistanceMonitor class
    turtle_distance_monitor = TurtleDistanceMonitor()

    # Start the node and continuously run a while loop
    while rclpy.ok():
        turtle_distance_monitor.monitor_turtles()
        rclpy.spin_once(turtle_distance_monitor)  # Allow ROS 2 to process other messages

    # Shutdown ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
