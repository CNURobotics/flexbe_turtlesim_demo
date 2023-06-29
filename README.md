# Flexbe Turtlesim Demonstration

This repo contains all flexbe_turtlesim_demo-specific states and behaviors to
provide a simple demonstration of FlexBE using the ROS [Turtlesim] packages.

FlexBE includes both an *Onboard* robot control behavior executive and an 
Operator Control Station (*OCS*) for supervisory control and *collaborative autonomy*.


## Installation

In addition to the standard FlexBE flexbe_app and flexbe_behavior_engine packages,
clone the following repo into your ROS workspace:

    git clone https://github.com/FlexBE/flexbe_turtlesim_demo.git  # if not already present

Make sure that the branches are consistent (e.g. `git checkout ros2-devel`)

Install any required dependencies.

  * `rosdep update`
  * `rosdep install --from-paths src --ignore-src`


Build your workspace:

  `colcon build`

After sourcing the new setup as normally required, you must download the required `nwjs` binaries
*before* you can run the FlexBE App:

  `ros2 run flexbe_app nwjs_install`

  > Note: These are installed in the `install` folder.  If the `install` folder is deleted, then the `nwjs` binaries
  will need to be reinstalled with this script.

## Usage

Launch the Turtlesim node, FlexBE UI App, and Flexible Behavior engine

Quick start:

For each command, we assume the ROS environment is set up in the terminal using `setup.bash` after a build.

First, launch TurtleSim:
  
`ros2 run  turtlesim turtlesim_node`

> Note: Unlike regular simulations such as `Gazebo`, `TurtleSim` does NOT 
> publish a `\clock` topic to ROS.  Therefore, do NOT set `use_sim_time:=True` with these demonstrations!
> Without a `clock`, nothing gets published and so the system will appear hung; therefore TurtleSim should 
> use the real wallclock time.

Then start FlexBE using one (and only one) of the follow three blocks:

* FlexBE Quickstart of behaviors with full autonomy 

`ros2 launch flexbe_onboard behavior_onboard.launch.py use_sim_time:=False`

`ros2 run flexbe_widget be_launcher -b "Simple FlexBE Turtlesim Demo" --ros-args --remap name:="behavior_launcher" -p use_sim_time:=False`

  This will launch the `Simple FlexBE Turtlesim Demo`, which will move the turtle through a series of motions to generate 
  a figure 8 pattern in full autonomy mode.

<img src="img/turtlesim_figure8.png" alt="Turtlesim figure 8 under FlexBE 'Simple FlexBE Turtlesim Demo' behavior." width="250">

 > Note: Clicking on any image will give the high resolution view.


  We will discuss *"Attaching"* the user interface later.

  For now, just `Ctrl-C` to end the `behavior_onboard` and `be_launcher` nodes, and move on to the next demos.

Ensure that a `turtlesim` window is open via the `ros2 run  turtlesim turtlesim_node`.

There are 3 approaches to launching the full FlexBE suite for operator supervised autonomy-base control.
Use one (and only one of the approaches):

1) FlexBE Quickstart

`ros2 launch flexbe_app flexbe_full.launch.py use_sim_time:=False`

  This starts all of FlexBE including both the *OCS* and *Onboard* software.

2) Launch the *OCS* and *Onboard* separately:

`ros2 launch flexbe_app flexbe_ocs.launch.py use_sim_time:=False`

`ros2 launch flexbe_onboard behavior_onboard.launch.py use_sim_time:=False`

  This allows running the *Onboard* software *on board* the robot, and the *OCS* software on a separate machine.

3) Launch each FlexBE component separately:
  * *Onboard*

  `ros2 launch flexbe_onboard behavior_onboard.launch.py use_sim_time:=False`

  ----
  * *OCS*

  `ros2 run flexbe_mirror behavior_mirror_sm --ros-args --remap __node:="behavior_mirror" -p use_sim_time:=False`
  
  `ros2 run flexbe_app run_app --ros-args --remap name:="flexbe_app" -p use_sim_time:=False`
  
  `ros2 run flexbe_widget be_launcher --ros-args --remap name:="behavior_launcher" -p use_sim_time:=False`

The *OCS* components can be run on a separate computer from the *onboard* components.

## Controling Behaviors Via FlexBE UI

Using the FlexBE UI application *Behavior Dashboard*, select *Load Behavior* from the upper middle tool bar, and 
select the `flexbe_turtlesim_demo_flexbe_behaviors` package from dropdown, and the `Simple FlexBE Turtlesim Demo`.

<img src="img/loading_behavior.png" alt="Loading behavior via FlexBE UI Dashboard" width="350">
<img src="img/editor_view.png" alt="State machine editor view" width="350">

The *Statemachine Editor* tab is used to inspect or edit existing behaviors, or build new ones.  
The `Simple FlexBE Turtlesim Demo` behavior is shown above.



The `flexbe_turtlesim_demo_flexbe_states` package includes custom state examples for:

  * `clear_turtlesim_state` - clear the turtlesim window using a *blocking* service call
  * `teleport_absolute_state` - go to home position using a *non-blocking* service call
  * `timed_cmd_vel_state` - publish command velocity using a specified desired update rate

    > NOTE: The desired state update rate is only best effort.  FlexBE is NOT a real time controller, and
    > is generally suited for lower rate periodic monitoring that does not require precise timing.

----

The *Runtime Control* tab allows the operator to launch behaviors on the onboard syste, and monitor their execution.

<img src="img/execute_view.png" alt="Ready to launch loaded behavior." width="350">
<img src="img/monitoring_view.png" alt="Monitoring running behavior." width="350">

Click on the "Eight" transition to make one loop in the figure 8 pattern.  After completion it will bring you back 
to the *Operator Decision* state.  From there you can choose "Home" to recenter your turtle, or "Clear" to 
clear the path trace, or "Eight" to do another loop, or "Quit" to complete the statemachine behavior and exit the 
runtime control.

FlexBE supports variable autonomy levels, so choosing "Full" autonomy allows the system to automatically choose to 
repeat the "Eight" transition.

<img src="img/full_autonomy_loops.png" alt="Autonomous behavior in Full autonomy." width="500">

This is based on the settings in the *Operator Decision State.  By allowing a transition to "Eight" with only "High" autonomy, 
setting the executive to "Full" autonomy allows the automatic transition.

<img src="img/decision_state_settings.png" alt="Settings for operator decision state." width="350">
<img src="img/hfsm_container_sub_sm.png" alt="HFSM sub state machine." width="350">

FlexBE supports Hierarchical Finite State Machines (HFSM) so that the "Container" state is actually a (simple) state machine that executes the figure 8 pattern using the provided FlexBE state implementations 
such as the [Timed Cmd Velocity State](flexbe_turtlesim_demo_flexbe_states/flexbe_turtlesim_demo_flexbe_states/timed_cmd_vel_state.py) 
which publishes a fixed command velocity as a [Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/TwistStamped.html) (forward speed and turning rate) for a given time duration.  The specific parameters are set in the FlexBE Editor by clicking on a particular state; the "Left Turn" state values are shown below.

<img src="img/timed_cmd_vel.png" alt="State parameters." width="250">




----

See the [FlexBE tutorials] for more  information about loading and launch behaviors.

This package also includes a simple `Example Behavior` that is described in the FlexBE tutorials.


## Publications

Please use the following publications for reference when using FlexBE:

- Philipp Schillinger, Stefan Kohlbrecher, and Oskar von Stryk, ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics"](http://dx.doi.org/10.1109/ICRA.2016.7487442), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

- Joshua Zutell, David C. Conner and Philipp Schillinger, ["ROS 2-Based Flexible Behavior Engine for Flexible Navigation ,"](http://dx.doi.org/10.1109/SoutheastCon48659.2022.9764047), IEEE SouthEastCon, April 2022.

-----

[Turtlesim]:https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
[FlexBE tutorials]:http://wiki.ros.org/flexbe/Tutorials
