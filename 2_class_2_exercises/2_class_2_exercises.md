# Session 2 exercises

In codespaces I recommend you to work on the folder /workspaces/EADAros2codespaces/2_class_2_exercices/.

# I. Service

## Exercise 1

Start the turtlesim node

Terminal 1:
```bash
ros2 run turtlesim turtlesim_node
```

Test the following commands

Terminal 2:
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 1.0, y: 1.0, theta: 0.0, name: 'Speedy'}"
```

```bash
ros2 service list
```

## Exercise 2

Start creating the turtle_rgb package to create a service interface. In this case, we choose ament_cmake.

```bash
cd src
ros2 pkg create turtle_rgb --build-type ament_cmake
```

Create file:
```bash
turtle_rgb/srv/SetRGB.srv
```

```bash
uint8 r
uint8 g
uint8 b
---
bool success
```

Modify CMakeLists.txt. Below the first find_package.

```bash
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetRGB.srv"
)
```

Modify package.xml. Below ament_cmake.
```bash
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Build and verify. Be sure to be in your workspace directory.
```bash
colcon build --packages-select turtle_rgb
source install/setup.bash
ros2 interface show turtle_rgb/srv/SetRGB
```

If it works the .srv definition should appear in the terminal.

## Exercise 3

A service server is started in turtle_rgb_server.

```bash
cd src
ros2 pkg create turtle_rgb_server --build-type ament_python
```

Paste the code into a node turtle_rgb_server.py

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Your custom service type generated from srv/SetRGB.srv
from turtle_rgb.srv import SetRGB


class TurtleRGBServer(Node):
    def __init__(self):
        super().__init__('turtle_rgb_server')

        self.service = self.create_service(
            SetRGB,
            '/set_rgb',
            self.set_rgb_callback
        )

        self.get_logger().info("Service /set_rgb ready.")

    def set_rgb_callback(self, request, response):
        # Just acknowledge we received the request
        self.get_logger().info(
            f"Received RGB request: r={request.r}, g={request.g}, b={request.b}"
        )

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TurtleRGBServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Don't forget to modify setup.py

```
entry_points={
    'console_scripts': [
        'turtle_rgb_server = turtle_rgb_server.turtle_rgb_server:main',
    ],
},
```

Build and test:
```bash
colcon build --packages-select turtle_rgb
source install/setup.bash
# Terminal 1
ros2 run turtle_rgb_server turtle_rgb_server
# Terminal 2
ros2 service call /set_rgb turtle_rgb/srv/SetRGB "{r: 255, g: 0, b: 0}"
```


# II. Actions

## Part I - explanation

We will reuse **turtlesim**, even though it does **not provide actions by default**.

This is intentional:  
we will **implement our own action** to understand how actions work.

Terminal 1:
```bash
ros2 run turtlesim turtlesim_node
```

Turtlesim implements services and topics straight away (already seen in previous session).

Topics published by default:
+ /turtle1/pose
+ /turtle1/cmd_vel


## Part 2 - Reach the Edge and Return to the Center (Exercise 4, take home)

We will create an action that makes the turtle:

+ Move forward until it reaches the edge of the world
+ Stop
+ Turn back
+ Return to the center
+ Finish successfully (or stop if canceled)

In turtlesim:

+ World limits: approximately 0 → 11
+ Center position: (5.5, 5.5)
+ Edge threshold: 0.5

Codebase Locations

Start your development using the templates in these folders:

+ Action Definition: action_msg/action/ReachEdgeAndReturn.action
+ Server Code: reach_edge_action_server/reach_edge_action_server.py

```bash
colcon build
source install/setup.bash
# Terminal 1
ros2 run turtlesim turtlesim_node
# Terminal 2
ros2 run reach_edge_action_server reach_edge_action_server 
```

### Submission (4 points)

Please submit a zip file or repository link containing the following:

+ Include your completed reach_edge_action_server.py file.
+ A screen recording showing the turtlesim window and the terminal.

Command to run: ros2 action send_goal /reach_edge_and_return action_msg/action/ReachEdgeAndReturn {}
The video should show the turtle hitting the edge, turning, returning to the center, and the terminal displaying "Goal finished with status: SUCCEEDED."

Provide a short paragraph (3–5 sentences) explaining what an action is and why is it useful.

## III. Transforms

These exercises focus on transforms.

> **Reminder:**  TF and nodes describe *how things move*.

### Part 1 - Inspect with rviz

#### 1. Open rviz

Terminal 1:
```bash
rviz2
```

+ Change the Fixed Frame (under Global Options) from map to world.
+ Click Add (bottom left) and select TF.

#### 2. Publish static transform

Remember tf2_ros static_transform_publisher followS:

```bash
ros2 run tf2_ros static_transform_publisher \
  x y z qx qy qz qw \
  parent_frame child_frame
```

Terminal 2:
```bash
ros2 run tf2_ros static_transform_publisher 1.0 0.0 0.0 0 0 0 1 world example
```

In RViz, you will now see an axis named example exactly 1 meter away from the world center.

#### 3. Inspecting
```bash
ros2 run tf2_ros tf2_echo world example
```
Expected result:

```bash
ros2 run tf2_ros tf2_echo world example
[INFO] [1769686794.389857237] [tf2_echo]: Waiting for transform world ->  example: Invalid frame ID "world" passed to canTransform argument target_frame - frame does not exist
At time 0.0
- Translation: [1.000, 0.000, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.000]
- Rotation: in RPY (degree) [0.000, -0.000, 0.000]
- Matrix:
  1.000  0.000  0.000  1.000
  0.000  1.000  0.000  0.000
  0.000  0.000  1.000  0.000
  0.000  0.000  0.000  1.000
```

### Part 2 - Use TFs for turtle1 to follow turtle2 (Exercise 5 - take home) 

Now we move from static points to moving robots. We will use the TF tree to calculate the steering commands for a second turtle.

1. build and source the workspace of 2_class_2_exercices

2. Launch the Environment We will use a single launch file to start the simulator, the spawner (which moves Turtle 1), and the TF broadcasters for both turtles.

Terminal 1:
```bash
ros2 launch turtle_spawner turtle_spawner.launch.py
```

In this code, you will see two turtles:
+ The first is following a circle path and broadcasting a TF.
+ The second is static and broadcasting a TF.

3. The Exercise: Complete the Follower Open src/turtle_follower/turtle_follower/turtle_follower.py. You need to complete the logic in the on_timer function.
 
Your goal is to use the TF Buffer to find out where Turtle 1 is located relative to Turtle 2.

+ Open turtle_follower.py.
+ Locate the lookup_transform function.
+ Ensure the frames are set correctly: turtle2 (you) and turtle1 (the leader).
+ Observe how trans.transform.translation.x is used directly to drive the motors.

Result: When you run the launch file and the follower node, Turtle 2 will use the TF tree as its "eyes" to track and follow Turtle 1.

### Submission (4 points)

Please submit a zip file or a repository link containing the following: 

+ Your completed turtle_follower node (final working version).
+ A short screen recording showing the turtlesim window.
+ A short description (3–5 sentences) explaining what your turtle_follower node does.

### IV. URDF and Xacro

These exercises focus on robot description using **URDF** and **xacro**.  

> **Reminder:** URDF/xacro describe *what the robot looks like*.  


---

## Exercise 6 part 1 — Load a URDF model and change parameters

Load a simple robot model in RViz and modify its geometry and appearance.

1. The URDF File: urdf/simple_box.urdf.xml

Create a folder with a file.

---

### Provided file: `simple_box.urdf.xml`

```xml
<robot name="simple_box">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

</robot>
```

Run the model:

2. Prerequisites (Install missing tools)

If not already installed, run this to enable the GUI and coordinate transforms:
Bash

sudo apt update
sudo apt install ros-humble-joint-state-publisher-gui

3. The Launcher: display.launch.py

Create this file in your exercise folder. It automates the robot_state_publisher and ensures RViz sees your robot.

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define absolute path to your URDF
    urdf_path = '/workspaces/EADAros2codespaces/2_class_2_exercises/urdf/simple_box.urdf.xml'
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1. Publishes the URDF to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        # 2. Required for multi-link robots
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        # 3. STATIC TRANSFORM: Connects map to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        # 4. Opens RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])
```

4. Running the model
```Bash
# Navigate to your exercise folder
cd /workspaces/EADAros2codespaces/2_class_2_exercises/

# Run the launcher
ros2 launch display.launch.py
```

After launching the file, follow these steps to visualize the model:

+ Set Fixed Frame: Change Fixed Frame to base_link (or map if using a static transform).

+ Add RobotModel: Click Add (bottom left) and select RobotModel.

+ Link Topic: Expand RobotModel, click Description Topic, and select /robot_description.

## Exercise 6 part 2 — Introduction to Xacro

### The Point
Use variables and math to make your robot model modular.

### Setup
1. Add a new file: `simple_box.xacro`.
2. Use `${variable_name}` to call your properties.
3. Use the updated `display_xacro.launch.py` that includes the `xacro` python library.

```python
import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define absolute path to your URDF
    xacro_file = '/workspaces/EADAros2codespaces/2_class_2_exercises/urdf/simple_box.xacro'
    
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()

    return LaunchDescription([
        # 1. Publishes the xacro to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}]
        ),
        # 2. Required for multi-link robots
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        # 3. STATIC TRANSFORM: Connects map to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        # 4. Opens RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])
```

## Exercise 6 part 3 — Add two links (either Xacro or xml)

### Instructions

Create the File: In your urdf_tutorial/urdf/ folder, create a file named human_body.urdf.xacro.

1. Define the Torso: Use a base_link as the torso (root of your tree).
2. Add the Head:
3. Create a head_link (use a <sphere> geometry).
+ Connect it to the torso with a fixed joint named neck_joint.
4. Add the Limbs:
+ Add two additional links (arms or legs) using <cylinder> geometries to complete the body.
+ Ensure each new link has a unique name and a corresponding joint connecting it to the base_link.
+ Visualize: Launch the model in RViz2 to verify the structure.

### Submission (4 points)

To complete the submission, provide the following:

1. The Code: Your completed .urdf.xacro or .urdf file showing the head and all limbs.
2. The Screenshot: A single image of RViz2 containing:
3. The RobotModel showing the 3D human shape.