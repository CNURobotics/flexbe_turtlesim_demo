#!/usr/bin/env python
from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from turtlesim.action import RotateAbsolute

import math


class RotateTurtleState(EventState):
    """
    Actionlib actions are the most common basis for state implementations.

    These are used to interface with longer running more computationally intensive calculations in a
    non-blocking manner.

    State implementations should be lightweight and non-blocking so that they execute() completes within the
    desired update period.

    The ROS 2 action library provides a non-blocking, high-level interface for robot capabilities.

    Elements define here for UI
    Parameters
    -- timeout             Maximum time allowed (seconds)
    -- action_topic        Name of action to invoke

    Outputs
    <= rotation_complete   Only a few dishes have been cleaned.
    <= failed              Failed for some reason.
    <= canceled            User canceled before completion.
    <= timeout             The action has timed out.

    User data
    ># angle     float     Desired rotational angle in (degrees) (Input)
    #> duration  float     Amount time taken to complete rotation (seconds) (Output)

    """

    def __init__(self, timeout, action_topic="/turtle1/rotate_absolute"):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['rotation_complete', 'failed', 'canceled', 'timeout'],
                         input_keys=['angle'],
                         output_keys=['duration'])

        self._timeout = Duration(seconds=timeout)
        self._topic = action_topic

        # Create the action client when building the behavior.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior.
        ProxyActionClient.initialize(RotateTurtleState._node)

        self._client = ProxyActionClient({self._topic: RotateAbsolute})  # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._error = False
        self._return = None  # Retain return value in case the outcome is blocked by operator

    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            return 'failed'

        if self._return is not None:
            # Return prior outcome in case transition is blocked by autonomy level
            return self._return

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            # feedback.feedback.remaining to access feedback
            userdata.duration = self._node.get_clock().now() - self._start_time
            Logger.loginfo('Rotation complete')
            self._return = 'rotation_complete'
            return 'rotation_complete'

        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
            # Checking for timeout after we check for goal response
            self._return = 'timeout'
            return 'timeout'

        # If the action has not yet finished, no outcome will be returned and the state stays active.
        return None

    def on_enter(self, userdata):

        # make sure to reset the error state since a previous state execution might have failed
        self._error = False
        self._return = None

        # Recording the start time to set rotation duration output
        self._start_time = self._node.get_clock().now()

        goal = RotateAbsolute.Goal()

        if isinstance(userdata.angle, (float, int)):
            goal.theta = (userdata.angle * math.pi) / 180
        else:
            self._error = True
            Logger.logwarn("Input is %s. Expects an int or a float.", type(userdata.angle).__name__)

        # Send the goal.
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure,
            # it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn('Failed to send the RotateAbsolute command:\n%s' % str(e))
            self._error = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
