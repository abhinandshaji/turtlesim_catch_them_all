from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # for the turtle sim node to be ran
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim"
    )

    turtle_client_node = Node(
        package="my_cpp_pkg",
        executable="turtle_client_node",
        name="turtle_client_node"
    )

    turtle_control_node = Node(
        package="my_cpp_pkg",
        executable="turtle_control_node",
        name="turtle_control_node"
    )

    turtle_kill_client_node = Node(
        package="my_cpp_pkg",
        executable="turtle_kill_client_node",
        name="turtle_kill_client_node"
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_client_node)
    ld.add_action(turtle_control_node)
    ld.add_action(turtle_kill_client_node)
    return ld