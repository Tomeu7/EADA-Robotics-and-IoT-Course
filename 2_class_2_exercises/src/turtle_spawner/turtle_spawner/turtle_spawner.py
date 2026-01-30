import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

# also you can spawn turtles with:
# Terminal 1: ros2 run turtlesim turtlesim_node
# Terminal 2: ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'my_new_turtle'}"

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        
        # 1. Create client to spawn turtle2
        self.client = self.create_client(Spawn, 'spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
        
        self.spawn_turtle()

        # 2. Publisher for turtle1 movement
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_circle)

    def spawn_turtle(self):
        request = Spawn.Request()
        request.x = 2.0
        request.y = 2.0
        request.name = 'turtle2'
        self.client.call_async(request)

    def move_circle(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
