# Session 1 exercises

## Exercise 1 — Hello World Node

### Start with activating the default jazzy system-wide ROS

```bash
source /opt/ros/jazzy/setup.bash
```

### Create a workspace

Go to the mounted directory

```bash
cd /home/user/exchange
```

Always when you start a new project you need to create a workspace

```bash
mkdir -p ros2_course_ws/src
cd ros2_course_ws
```

Create your first package

https://docs.ros.org/en/jazzy/How-To-Guides/Developing-a-ROS-2-Package.html#python-packages

```bash
cd src
ros2 pkg create  --build-type ament_python exercise1
```

Create your first node!!

```bash
cd exercise1/exercise1
touch exercise1.py
```

Edit exercise1.py. Please take some time to understand each line of comment, I have added comments after every line

```python
# Import the ROS 2 Python client library
import rclpy
# Import the Node class, the basic building block of ROS 2 applications
from rclpy.node import Node

# Define a class that represents our first ROS 2 node
class HelloWorld(Node):

    def __init__(self):
        # Initialize the parent class (Node) with the name 'hello_world_node'
        super().__init__('hello_world_node')

        # Create a timer that triggers every 1.0 seconds (1 Hz)
        # When the timer triggers, the method 'timer_callback' is executed
        self.timer = self.create_timer(1.0, self.timer_callback)


    # This function runs every time the timer expires (once per second)
    def timer_callback(self):
        # Print a log message to the ROS console
        # This is similar to print(), but integrated into ROS 2 logging
        self.get_logger().info('Hello World from ROS 2 Jazzy!')



# The main() function is the entry point of the program
def main(args=None):

    # Initialize the ROS 2 communication system
    rclpy.init(args=args)

    # Create an instance of our node class
    node = HelloWorld()

    # Keep the node active so it can process timer callbacks and events
    # This is like a "while True" loop, but controlled by ROS 2
    rclpy.spin(node)

    # Destroy the node after shutdown (cleanup)
    node.destroy_node()

    # Shutdown the ROS 2 communication system
    rclpy.shutdown()

# If this file is executed directly (not imported), call main()
if __name__ == '__main__':
    main()
```

Register the new node

```bash
cd ~/ros2_course_ws/src/exercise1
nano setup.py
```

Replace or add inside entry_points={...}:

```python
entry_points={
    'console_scripts': [
        'exercise1 = exercise1.exercise1:main',
    ],
},
```

Compile

```bash
cd /home/user/exchange/ros2_course_ws/
colcon build
source install/setup.bash
```

Execute

```bash
terminator # terminator - to open two launchers
ros2 run exercise1 exercise1
```

## Exercise 2

# Exercise to test publisher/subscriber

Create packages

```bash
cd /home/user/exchange/ros2_course_ws/src/
ros2 pkg create --build-type ament_python exercise2_subscriber
ros2 pkg create --build-type ament_python exercise2_publisher
```

Create publisher Node. Remember to set up setup.py

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'greetings', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello from the Publisher node!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Create Subscriber Node. Remeber to set up setup.py

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'greetings',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Build and test
```bash
# Test nodes as well
ros2 node list # should appear empty
ros2 run exercise2_publisher exercise2_publisher
# Another terminal
ros2 node list # /simple_publisher
ros2 run exercise2_subscriber exercise2_subscriber
# Another terminal
ros2 node list
# /simple_publisher
# /simple_subscriber
ros2 topic list
# /greetings
# /parameter_events
# /rosout
rqt_graph
```

## Exercise 3 Tutlesim basics

Terminal 1
```bash
ros2 run turtlesim turtlesim_node
```

Terminal 2
```bash
ros2 node list
ros2 topic list
```

Identify:

+ /turtle1/cmd_vel
+ /turtle1/pose

Publish velocity commands manually:

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 2.0}, angular: {z: 1.0}}"
```

What do you see?

## Exercise 4 (take home)

## Overview

**Goal:** Control turtlesim with a ROS 2 Python node and monitor it like an IoT device.

You will create **3 ROS 2 Python nodes**:
1) `turtle_controller.py` → publishes `/turtle1/cmd_vel` (`Twist`)  
2) `telemetry_publisher.py` → publishes `/telemetry` (`String`)  
3) `dashboard_subscriber.py` → subscribes `/telemetry` and prints a report

## Setup

Terminal 1:
```bash
ros2 run turtlesim turtlesim_node
```
Terminal 2:
```
cd /home/user/exchange/ros2_course_ws/src
ros2 pkg create --build-type ament_python exercise4
```

## Part A: Turtle controller node

Requirements

```bash
cd exercise4/exercise4
touch turtle_controller.py
```

Starter code

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.step)  # 10 Hz control loop

        # ---- MODIFY THESE LINES (choose your shape) ----
        self.shape = 'square'  # options: 'square', 'circle', 'infinity'
        self.linear_speed = 2.0
        self.angular_speed = 1.57   # ~90 deg/s (rad/s) for square turning
        # ------------------------------------------------

        self.state = 'FORWARD'   # used for square/infinity
        self.t = 0.0            # elapsed time in current state (seconds)
        self.segment_time = 2.0 # MODIFY: forward duration for square segment
        self.turn_time = 1.0    # MODIFY: turn duration for 90 degrees
        self.step_count = 0     # used as "progress"

    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def step(self):
        dt = 0.1
        self.t += dt

        if self.shape == 'circle':
            # ---- MODIFY THESE LINES ----
            # Circle: constant velocity for some time, then stop
            self.publish_cmd(self.linear_speed, 1.0)  # set angular speed
            if self.t > 10.0:  # set duration
                self.stop()
                rclpy.shutdown()
            # ----------------------------

        elif self.shape == 'square':
            # Square: FORWARD for segment_time, TURN for turn_time, repeat 4 times
            if self.state == 'FORWARD':
                self.publish_cmd(self.linear_speed, 0.0)
                if self.t >= self.segment_time:
                    self.state = 'TURN'
                    self.t = 0.0
            elif self.state == 'TURN':
                self.publish_cmd(0.0, self.angular_speed)
                if self.t >= self.turn_time:
                    self.state = 'FORWARD'
                    self.t = 0.0
                    self.step_count += 1

            # ---- MODIFY THESE LINES ----
            if self.step_count >= 4:   # 4 turns = 1 square
                self.stop()
                rclpy.shutdown()
            # ----------------------------

        elif self.shape == 'infinity':
            # Infinity: simple approach = 1 circle left, then 1 circle right
            # You can implement this using states and time thresholds.
            # ---- TODO: MODIFY / COMPLETE THIS SECTION ----
            self.get_logger().info('TODO: implement infinity (figure-eight)')
            self.stop()
            rclpy.shutdown()
            # ---------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

What to modify
+ Add at least one status rule
+ Ensure the dashboard prints neatly

## Part B: Turtle controller node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TelemetryPublisher(Node):
    def __init__(self):
        super().__init__('telemetry_publisher')
        self.pub = self.create_publisher(String, '/telemetry', 10)
        self.timer = self.create_timer(1.0, self.publish_telemetry)  # 1 Hz

        # ---- MODIFY THESE LINES ----
        self.shape = 'square'
        self.speed = 2.0
        self.step = 0
        # ----------------------------

    def publish_telemetry(self):
        msg = String()
        # MODIFY THESE LINES  TO PUBLISH A STRING MESSAGE WITH step, speed and shape

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

What to modify
+ Add at least one status rule
+ Ensure the dashboard prints neatly

## Part C: Dashboard Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DashboardSubscriber(Node):
    def __init__(self):
        super().__init__('dashboard_subscriber')
        self.sub = self.create_subscription(String, '/telemetry', self.callback, 10)

    def callback(self, msg: String):
        # MODIFY THESE LINES  TO PRINT DIFFERENT METRICS

def main(args=None):
    rclpy.init(args=args)
    node = DashboardSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

What to modify
+ Add at least one status rule
+ Ensure the dashboard prints neatly

### Extra setup:

Register nodes in setup.py

Edit:

```bash
cd /home/user/exchange/ros2_course_ws/src/exercise4
nano setup.py
```

Inside entry_points, add:
```python
entry_points={
    'console_scripts': [
        'turtle_controller = exercise4.turtle_controller:main',
        'telemetry_publisher = exercise4.telemetry_publisher:main',
        'dashboard_subscriber = exercise4.dashboard_subscriber:main',
    ],
},
```

Build and run
```bash
cd /home/user/exchange/ros2_course_ws
colcon build
source install/setup.bash
```

Run in three terminals:
```bash
ros2 run exercise4 turtle_controller
ros2 run exercise4 telemetry_publisher
ros2 run exercise4 dashboard_subscriber
```

### Submit:

1. Your 3 Python files
2. A video of turtlesim drawing the shape
3. A screenshot of the dashboard output
4. A text of what you did (5–10 lines)