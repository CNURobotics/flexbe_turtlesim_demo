#!/usr/bin/env python
from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from turtlesim.action import RotateAbsolute


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
    <= canceled            User canceled before completion

    User data
    ># angle     float     Desired rotational angle in (degrees) (Input)
    #> duration  float     Amount time taken to complete rotation (seconds) (Output)

    """

    def __init__(self, timeout, action_topic="/turtle1/rotate_absolute"):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['rotation_complete', 'failed', 'canceled'],
                                                 input_keys=['angle'],
                                                 output_keys=['cleaned'])
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

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            dishes_cleaned = result.total_dishes_cleaned

            # In this example, we also provide the amount of cleaned dishes as output key.
            userdata.cleaned = dishes_cleaned

            # Based on the result, decide which outcome to trigger.
            if dishes_cleaned > self._dishes_to_do:
                return 'cleaned_enough'
            else:
                return 'cleaned_some'

        # If the action has not yet finished, no outcome will be returned and the state stays active.

    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # As documented above, we get the specification of which dishwasher to use as input key.
        # This enables a previous state to make this decision during runtime and provide the ID as its own output key.
        dishwasher_id = userdata.dishwasher

        # Create the goal.
        goal = DoDishesGoal()
        goal.dishwasher_id = dishwasher_id

        # Send the goal.
        self._error = False  # make sure to reset the error state since a previous state execution might have failed
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure,
            # it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn('Failed to send the DoDishes command:\n%s' % str(e))
            self._error = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
