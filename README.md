# ROS2 Turtlesim-Catch_Them_All Simulation
## Description
In this project, A simulation of the popular ROS2 turtlesim executable was created. A master turtle was made to leverage the P controller to catch all other turtles spawned on the screen. To achieve this, 3 nodes were utilized and 3 custom interfaces were written (2 msgs + 1 srv).

![Img: Turtlesim CTA](https://github.com/LateefAkinola/ROS2-Turtlesim-Catch_Them_All-Project/assets/105966848/2e9768d5-569b-489e-84d3-d3ed4aa8f32a)

![Catch Them All Demo](https://github.com/LateefAkinola/ROS2-Turtlesim-Catch_Them_All-Project/assets/105966848/eb6e6cb0-801a-4af1-866f-2b7acbf743b7)

### The nodes:
- [x] The **turtlesim_node** from the turtlesim package
- [x] The **tutle_controller**: A custom node to control the turtle (named “turtle1”) which is already existing in the turtlesim_node.
- [x] The **tutle_spawner**: A custom node to spawn turtles on the window, and to manage which turtle is still “alive” (on the screen).

### The custom interfaces:
- [x] **Turtle.msg** and **TurtleArray.msg** to send the list of turtles (name + coordinates) on the **/alive_turtles** topic
- [x] **CatchTurtle.srv** to send the name of the turtle which was caught. The client is the turtle_controller node and the server is the turtle_spawner node

## Installation
```bash
# Create your workspace directory
mkdir turtlesimcta_ws/
cd turtlesimcta_ws/

# Create your workspace directory
mkdir src/
cd src/
```
```bash
# Clone the repo
git clone https://github.com/LateefAkinola/Mini-MobileArmBot_with_ROS2.git
```
```bash
# Navigate back to the turtlesimcta_ws/ directory
cd ..
```
```bash
# Build
colcon build

# Source the WS
source install/setup.bash
```

## Testing
```bash
# Execute the Launch File
ros2 launch turtlesim_cta_bringup turtlesim_cta_app.launch.py 
```

## Future Work
This project can be further enhanced by:
- [x] Using a PID controller for better efficiency;
- [x] Adding one or more master_turtles, etc.

Thank you :smile:
