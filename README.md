# move_base_abstract

A ROS package that is designed to serve as a drop-in replacement for move_base, to provide abstracted navigation for faster, if less realistic, simulations.
This package works with Gazebo or stage_ros as the simulator, and tells the simulator to set the pose of a robot to a given goal pose after a delay in ROS time equal to a constant average robot speed (m/s) multiplied by the distance (m) between the robot's current pose and its goal pose.

To use move_base_abstract, include the package in your catkin workspace, build and source the workspace, and then call `rosrun move_base_abstract move_base_abstract_actionserver`. This node must be running, instead of running any move_base nodes, and the node can be namespaced to support the use of multiple robots, each with their own instance of the actionserver.
