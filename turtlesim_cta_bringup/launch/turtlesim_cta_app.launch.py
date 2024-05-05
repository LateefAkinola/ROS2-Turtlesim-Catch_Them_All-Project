from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    turtle_controller = Node(
        package="turtlesim_cta_py",
        executable="turtle_controller",
        parameters=[
            {"catch_nearest_turtle": True}
        ]
    )
    turtle_spawner = Node(
        package="turtlesim_cta_py",
        executable="turtle_spawner",
        parameters=[
            {"spawning_frequency": 1.3},
            {"turtle_name_prefix": "Lat_turtle"}
        ]
    )
    
    ld.add_action(turtlesim_node)
    ld.add_action(turtle_controller)
    ld.add_action(turtle_spawner)
    return ld