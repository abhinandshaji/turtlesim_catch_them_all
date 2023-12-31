# turtlesim_catch_them_all

https://github.com/abhinandshaji/turtlesim_catch_them_all/assets/91078614/8f7d8ead-0612-45cf-9d27-3221b3f08fc2

This project was done as project for a ROS2 humble course.

objective: Use turtlesim to spawn several turtles in certain intervals of time and make the first turtle catch all the turtle. Once the first turtle catches the newly spawned turtle it is killed and the turtlesim is cleared.

src:
Contains all the packages which are necessary. Place src in the root directory.
my_cpp_pkg contains cpp codes of all the nodes we are running.
turtlesim_bringup contains the launch file for launching all the nodes 

All the executables: turtle_client_node, turtle_control_node, turtle_kill_client_node, turtlesim_node

Build the packages in the root directory using 
```
colcon build 
```

Source setup in the root directory using 
```
source install/setup.bash
```

Run this command in the terminal where we have sourced the setup files to run the launch file
```
ros2 launch turtlesim_bringup turtlesim_catch_them_all.launch.py
```
---
The PID control used in the [turtle_control_node](https://github.com/abhinandshaji/turtlesim_catch_them_all/blob/main/src/my_cpp_pkg/src/turtle_control_node.cpp) has been tested in matlab simulation.

![image](https://github.com/abhinandshaji/turtlesim_catch_them_all/assets/91078614/de699822-a1fa-46fc-8c9d-eb1ccd8d4d24)

The results found that the system was critically damped system. This ensures the turtle moves to its position without any overshoot.




