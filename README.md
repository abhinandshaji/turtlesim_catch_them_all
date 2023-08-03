# turtlesim_catch_them_all

This project was done as project for a ROS2 humble course.

objective: Use turtlesim to spawn several turtles in certain intervals of time and make the first turtle catch all the turtle. Once the first turtle catches the newly spawned turtle it is killed and the turtlesim is cleared.

src:
Contains all the packages which are necessary. Place src in the root directory.
my_cpp_pkg contains cpp codes of all the nodes we are running.
turtlesim_bringup contains the launch file for launching all the nodes 

All the executables: turtle_client_node, turtle_control_node, turtle_kill_client_node, turtlesim_node
