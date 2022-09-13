# flexbe_turtlesim_demo_behaviors

This repo contains all flexbe_turtlesim_demo-specific states and behaviors to
provide a simple demonstration of FlexBE using the ROS [Turtlesim] packages.


## Installation

In addition to the standard FlexBE flexbe_app and flexbe_behavior_engine packages,
clone the following repo into your ROS workspace:

    `git clone https://github.com/FlexBE/flexbe_turtlesim_demo_behaviors.git`  # if not already present

Make sure that the branches are consistent (e.g. `git checkout ros2-devel`)

Install any required dependencies.

    `rosdep update `
    `rosdep install --from-paths src --ignore-src`


Build your workspace:

    `colcon build`

After sourcing the new setup as normally required, you must download the required `nwjs` binaries
*before* you can run the FlexBE App:

    `ros2 run flexbe_app nwjs_install`

  > Note: These are installed in the `install` folder.  If the `install` folder is deleted, then the `nwjs` binaries
  will need to be reinstalled with this script.

## Usage

Launch the Turtlesim node, FlexBE UI App, and Flexible Behavior engine
    `ros2 launch flexbe_turtlesim_demo_behaviors flexbe_turtlesim_demo.launch.py`
    `ros2 launch flexbe_app flexbe_full.launch.py`

You can also replace the latter launch with
    `ros2 launch flexbe_app flexbe_ocs.launch.py`
    `ros2 launch flexbe_onboard behavior_onboard.launch.py`  
This allows the FlexBE behavior engine to run on a separate computer from the UI.


Load the `Simple FlexBE Turtlesim Demo` behavior from the FlexBE UI, and begin
execution.  See the [FlexBE tutorials] for more  information about loading and launch behaviors.



## Publications

Please use the following publications for reference when using FlexBE:

- Philipp Schillinger, Stefan Kohlbrecher, and Oskar von Stryk, ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics"](http://dx.doi.org/10.1109/ICRA.2016.7487442), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

- Joshua Zutell, David C. Conner and Philipp Schillinger, ["ROS 2-Based Flexible Behavior Engine for Flexible Navigation ,"](http://dx.doi.org/10.1109/SoutheastCon48659.2022.9764047), IEEE SouthEastCon, April 2022.



-----

[Turtlesim]:https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
[FlexBE tutorials]:http://wiki.ros.org/flexbe/Tutorials
