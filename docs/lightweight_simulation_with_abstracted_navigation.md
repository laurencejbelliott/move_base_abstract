A bespoke lightweight simulation of the RASberry system can be run that uses an abstraction of 'move_base' navigation called [move_base_abstract](https://github.com/laurencejbelliott/move_base_abstract) (specifically the `rasberry` branch). move_base_abstract is a ROS package that serves as a drop-in replacement for the standard move_base navigation, but enables much faster simulation speeds as it essentially teleports the robot to a goal pose, over a period of time calculated from the robot's average speed, and the distance between the robot's current pose and the goal pose. Ths calculation uses ROS time, and move_base_abstract provides a node, `/sim_time_controller`, which can be used to control the rate at which ROS time progresses. This requires the `use_sim_time` parameter to be set to `true` in [TMuLE](https://github.com/LCAS/RASberry/wiki/Using-tmule-(suggested-way-of-launching)) config files, after roscore is run, but before other nodes are run. These features are not yet merged with `LCAS/RASberry`'s main branch, but are available in the `rasberry_transportation` branch of [Iranaphor's RASberry fork](https://github.com/Iranaphor/RASberry/), and should be merged in the near future.

This lightweight simulation is useful for testing the system across long periods of simulated time when full 3D visualisation of the environment and physics simulation of robots are not necessary. It has been designed to work with RASberry's topological navigation and virtual robot systems.

## Running a Simulation of Transportation Robots at the Riseholme Polytunnel

### Configure `.rasberryrc`
Edit the contents of `~/.rasberryrc` to match the following example. Be sure to change the values of `VPN_BASE_ADDR` and `MQTT_BROKER_IP` to match your own network configuration.

```bash
#Define information about the workspace
export RAS_WS=$HOME/rasberry_ws
export WORKSPACE_DIRECTORY=$RAS_WS
export RAS_REPO=$RAS_WS/src/RASberry

#Define information about the overall active system
export FARM=riseholme
export FIELD=polytunnel

#Define information for use-case
export RAS_PRIMARY_TMULE=$RAS_REPO/rasberry_core/new_tmule/server-virtual.tmule
export SCENARIO_NAME=combined_${FARM}_${FIELD}

#Networing variables
export VPN_BASE_ADDR="x.x.x.x"
export MQTT_BROKER_IP="x.x.x.x"
export WEBSOCKET_URL="ws://${MQTT_BROKER_IP}:8128/rasberry/ws"
export MQTT_ROHI_NAMESPACE="rohi3"
export OVERRIDE_SIM_TIME="true"
```
Once this is done, run `source ~/.rasberryrc` to load the new configuration.

### Launch the "Virtual Server" TMuLE Session
Simply enter the command `rtm` into a terminal to bring up a dialog for selecting a TMuLE config file to launch. Use the arrow keys to select `server_virtual` session, and press `Enter` repeatedly to launch the session.

This session will need to be left running in the background, launching nodes necessary for networking, multi-robot coordination, rviz, etc., and the `/sim_time_controller` node, which is used to control the rate at which ROS time progresses. See the `rviz` GUI window for visualisation of the topological map and robot poses.

### Launch the "Virtual Robot(s)" TMuLE Session(s)
Once the "Virtual Server" session has finished launching and is now running in the background (you can check this with `tmux ls`), enter the command `rtm` again to bring up the TMuLE session selection dialog. Use the arrow keys to select the `transportation_robot_virtual` session, and press the `Enter` key repeatedly to launch the session with the default parameters. This will launch a single virtual robot, namespaced as `thorvald_001`.

Once this session has finished launching, you can launch additional virtual robots by running the command `rtm` again, selecting the `transportation_robot_virtual` session, and pressing the `Enter` key repeatedly to launch the session with the default parameters. This will launch an additional virtual robot, namespaced as `thorvald_002`, but it will appear with the same pose as the virtual `thorvald_001`. You may wish to change the starting pose by watching the dialog each time you press the `Enter` key and entering value before pressing the `Enter` key when prompted for `X Pos` and `Y Pos`. This process can be repeated as many times as desired to launch additional virtual robots.
Topological navigation, multi-robot coordination, and the RoHi Dashboard (provided that it is configured to communicate with the MQTT broker of the "virtual server") should all work as they do with the real robots.

As with any tmule session, you can list the currently running `server_virtual` and `transportation_robot_virtual_thorvald_xx` tmule sessions with `tmux ls`, and attach to a running tmule session with `tmux a -t <session_name>`, to view the processes and ROS nodes running therein.