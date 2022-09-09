import os
import sys
import ament_index_python
import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import launch_testing_ros
import pytest

@pytest.mark.rostest
def generate_test_description():
    path_to_test = os.path.dirname(__file__)
    flexbe_testing_dir = get_package_share_directory('flexbe_testing')

    pkg = DeclareLaunchArgument(
        "pkg",
        default_value="flexbe_states")
    path = DeclareLaunchArgument(
        "path",
        default_value=path_to_test)

    flexbe_testing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(flexbe_testing_dir + "/launch/flexbe_testing.launch"),
        launch_arguments={
            'compact_format': "False",
            'package': LaunchConfiguration("pkg"),
            "testcases": LaunchConfiguration("path") + "/example_state.test\n" +
                         LaunchConfiguration("path") + "/teleport_absolute_state.test\n" +
                         LaunchConfiguration("path") + "/timed_twist_state.test"

        }.items()
    )

    return (
        launch.LaunchDescription([
            pkg,
            path,
            flexbe_testing
        ]),
        {
            'flexbe_testing': flexbe_testing,
        }
    )
