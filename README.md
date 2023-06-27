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

First, launch TurtleSim:
  * `ros2 run  turtlesim turtlesim_node`

> Note: Unlike regular simulations such as `Gazebo`, `TurtleSim` does NOT 
> publish a clock topic.  Therefore, do NOT set `use_sim_time:=True` with these demonstrations!
> Without a clock, nothing gets published and so the system will appear hung.

Then start FlexBE using one (and only one) of the follow three blocks:

* FlexBE Quickstart
    * `ros2 launch flexbe_app flexbe_full.launch.py use_sim_time:=False`

  This starts all of FlexBE including both the O
* Launch the *OCS* and *Onboard* separately:
    * `ros2 run  turtlesim turtlesim_node use_sim_time:=False`
    * `ros2 launch flexbe_app flexbe_ocs.launch.py use_sim_time:=False`
    * `ros2 launch flexbe_onboard behavior_onboard.launch.py use_sim_time:=False`

* Launch each FlexBE component separately:
  * *Onboard*
    * `ros2 launch flexbe_onboard behavior_onboard.launch.py use_sim_time:=False`
    ----
  * *OCS*
    * `ros2 run flexbe_mirror behavior_mirror_sm --ros-args --remap __node:="behavior_mirror" -p use_sim_time:=False`
    * `ros2 run flexbe_app run_app --ros-args --remap name:="flexbe_app" -p use_sim_time:=False`
    * `ros2 run flexbe_widget be_launcher --ros-args --remap name:="behavior_launcher" -p use_sim_time:=False`

The *OCS* components can be run on a separate computer from the *onboard* components.

Using the FlexBE UI application, load the `Simple FlexBE Turtlesim Demo` behavior from the
`flexbe_turtlesim_demo_flexbe_behaviors` package.

Alternatively, you can preload and execute the behavior using the `flexbe_widget be_launcher` by 
replacing the last command above with:
  * `ros2 run flexbe_widget be_launcher --ros-args --remap name:="behavior_launcher" -p use_sim_time:=False -b "Simple FlexBE Turtlesim Demo"`

  If following this approach, then you need to `Attach` the *OCS UI* to the running behavior.

Examine the behavior in the FlexBE editor window.

The `flexbe_turtlesim_demo_flexbe_states` includes custom state examples for:

  * `clear_turtlesim_state` - clear the turtlesim window using a blocking service call
  * `teleport_absolute_state` - go to home position using a non-blocking service call
  * `timed_cmd_vel_state` - publish command velocity using a specified desired update rate

    > NOTE: The desired state update rate is only best effort.  FlexBE is NOT a real time controller, and
    > is generally suited for lower rate periodic monitoring that does not require precise timing.

See the [FlexBE tutorials] for more  information about loading and launch behaviors.

  > NOTE: The tutorials are somewhat dated and
  > have not been updated for ROS 2; however, the
  > basic functionality is the same.

This package also includes the `Example Behavior` that is described in the tutorials.


## Publications

Please use the following publications for reference when using FlexBE:

- Philipp Schillinger, Stefan Kohlbrecher, and Oskar von Stryk, ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics"](http://dx.doi.org/10.1109/ICRA.2016.7487442), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

- Joshua Zutell, David C. Conner and Philipp Schillinger, ["ROS 2-Based Flexible Behavior Engine for Flexible Navigation ,"](http://dx.doi.org/10.1109/SoutheastCon48659.2022.9764047), IEEE SouthEastCon, April 2022.


-----

[Turtlesim]:https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
[FlexBE tutorials]:http://wiki.ros.org/flexbe/Tutorials
