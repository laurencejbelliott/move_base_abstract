# move_base_abstract

A ROS package that is designed to serve as a drop-in replacement for move_base, to provide abstracted navigation for faster, if less realistic, ROS simulation.
See our extended abstract, 'Towards an Abstract Lightweight Multi-robot ROS Simulator for Rapid Experimentation', published in the [proceedings of the TAROS '24 conference](https://taros-conference.org/wp-content/uploads/2023/09/taros2023_all_abstracts.pdf) for more detail.

This package works with Gazebo or stage_ros as the simulator, and tells the simulator to set the pose of a robot to a given goal pose after a delay in ROS time equal to a constant average robot speed (m/s) multiplied by the distance (m) between the robot's current pose and its goal pose.

There is a minimalist bespoke simulator provided in the `rasberry` branch of move_base_abstract, offering reduced computational overhead even when compared to stage_ros. 

## Setup

To use move_base_abstract, include the package in your catkin workspace, in the root folder of your workspace run
`rosdep install --from-paths src --ignore-src --rosdistro noetic -y .`
to automate installation of dependencies from apt. 

Then build and source the workspace with

`catkin build`

`source devel/setup.bash`

## Running the move_base_abstract ActionServer

To run an instance of the `move_base_abstract_actionserver` node, call

`rosrun move_base_abstract move_base_abstract_actionserver`

This node must be running, instead of running any move_base nodes, and can be namespaced to support the use of multiple robots, each using their own instance of the actionserver. You should then be able to interface with the node using a ROS ActionClient. This means that you provide Goals in the form of MoveBaseGoal actions, receive updates on the status of the goal, and can preempt a goal before its completion to cancel it. The [tutorial for sending goals to the move_base navigation stack](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals) should also apply to move_base_abstract given its design as a drop-in alternative to move_base.

## Optional Arguments for move_base_abstract_actionserver
When running a move_base_abstract_actionserver node, you may pass the following arguments to set the values of parameters detailed in the table below: 

| Index | Name | Description | Data Type | Default Value |
|---|---|---|---|---|
| 1 | robot_name | Name of robot. | string | None, or the namespace of the node if run within a namespace |
| 2 | simulator | Name of simulator (either 'stage_ros' or 'gazebo'). | string | 'stage_ros' |
| 3 | max_speed | Maximum speed (m/s) of robot (must be > 0). | float | 0.5 |
| 4 | start_pos_x | Starting x-position of robot. | float | 0.0 |
| 5 | start_pos_y | Starting y-position of robot. | float | 0.0 |
| 6 | pose_update_freq | Frequency (Hz) at which robot's pose is published. | float | 0.5 |
| 7 | acceleration | The accleration (m/s^2) of the robot (until its speed reaches max_speed). 0 = constant max. speed. | float | 0.0 |

E.g.

To start move_base_abstract_actionserver with
* robot_name: "robot_0"
* simulator: "stage_ros"
* max_speed: 2.0
* start_pos_x: 0.0
* start_pos_y: 0.0
* pose_update_freq: 0.1
* accleration: 0.1

In CLI:

`rosrun move_base_abstract_actionserver "robot_0" "stage_ros" 2.0 0.0 0.0 0.1 0.1`

In roslaunch XML:

`<node pkg="move_base_abstract" type="move_base_abstract_actionserver" respawn="false" name="move_base_node" output="screen"
    args='"robot_0" "stage_ros" 2.0 0.0 0.0 0.1 0.1'/>`
