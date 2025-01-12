
import rclpy
from rclpy.node import Node
from turtlesim.srv import Kill, Spawn

class TurtleServiceNode(Node):

    def __init__(self):
        super().__init__('turtle_service_node')
        self.kill_client = self.create_client(Kill, 'kill')
        self.spawn_client = self.create_client(Spawn, 'spawn')

        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available')

        self.kill_turtle()
        self.spawn_turtle()

    def kill_turtle(self):
        request = Kill.Request()
        request.name = 'turtle1'
        future = self.kill_client.call_async(request)
        future.add_done_callback(self.kill_callback)

    def spawn_turtle(self):
        request = Spawn.Request()
        request.x = 5.0
        request.y = 2.0
        request.theta = 0.0
        request.name = 'turtle2'
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)

    def kill_callback(self, future):
        self.get_logger().info('turtle dead')

    def spawn_callback(self, future):
        self.get_logger().info('turtle spawned')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
