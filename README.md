# ROS2 Turtlesim-Catch_Them_All Simulation
## Description
In this project, a simulation of the popular ROS2 Turtlesim executable was developed. A master turtle was programmed to use a P controller to catch all other turtles that appear on the screen. The implementation involved utilizing three nodes and creating three custom interfaces, consisting of two messages and one service.

![Img: Turtlesim CTA](https://github.com/LateefAkinola/ROS2-Turtlesim-Catch_Them_All-Project/assets/105966848/2e9768d5-569b-489e-84d3-d3ed4aa8f32a)

### The nodes:
- [x] The `turtlesim_node` from the turtlesim package
- [x] The `tutle_controller`: A custom node to control the turtle (named “turtle1”) which is already existing in the turtlesim_node.
- [x] The `tutle_spawner`: A custom node to spawn turtles on the window, and to manage which turtle is still “alive” (on the screen).

### The custom interfaces:
- `Messages`:
- [x] `Turtle.msg` and `TurtleArray.msg` to send the list of turtles (name + coordinates) on the `/alive_turtles` topic
- `Service`:
- [x] `CatchTurtle.srv` to send the name of the turtle which was caught. The client is the turtle_controller node and the server is the turtle_spawner node

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
git clone https://github.com/LateefAkinola/ROS2-Turtlesim-Catch_Them_All-Project.git
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
