#!/usr/bin/env python

"""Demo state."""
import rclpy

from flexbe_core import EventState, Logger


class ExampleState(EventState):
    """
    Example for a state to demonstrate which functionality is available for state implementation.

    All FlexBE states should inherit from EventState.

    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    The UI parses this description for data about the state to diplay.

    List parameter values with double hyphens
    -- target_time     float     Time which needs to have passed since the behavior started.

    List labeled outcomes using the double arrow notation (must match constructor)
    <= continue        Given time has passed.
    <= failed          Example for a failure outcome.

    List input and output user data that is passes along using <TODO>
    """

    def __init__(self, target_time):
        """Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments."""
        super().__init__(outcomes=['continue', 'failed'])

        # Store state parameter for later use.
        self._target_time = rclpy.Duration(target_time)

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

    def execute(self, userdata):
        """
        Execute this method periodically while the state is active.

        Main purpose is to check state conditions and trigger a corresponding outcome.
        If no outcome is returned, the state will stay active.
        """
        try:
            if ExampleState._node.get_clock().now() - self._start_time > self._target_time:
                return 'continue'  # One of the outcomes declared above.
        except Exception:  # pylint:disable=W0703
            return 'failed'

    def on_enter(self, userdata):
        """
        Call this method when the state becomes active.

        That is, when a transition from another state to this one is taken.
        It is primarily used to start actions which are associated with this state.

        The following code is just for illustrating how the behavior logger works.
        Text logged by the behavior logger is sent to the operator and displayed in the GUI.
        """
        time_to_wait = (self._target_time - (ExampleState._node.get_clock().now() - self._start_time)).to_sec()

        if time_to_wait > 0:
            Logger.loginfo('Need to wait for %.1f seconds.' % time_to_wait)

    def on_exit(self, userdata):
        """
        Call this method when an outcome is returned and another state gets active.

        It can be used to stop possibly running processes started by on_enter.
        Nothing to do in this example.
        """

    def on_start(self):
        """
        Call this method when the behavior is started.

        If possible, it is generally better to initialize used resources in the constructor
        because if anything failed, the behavior would not even be started.
        In this example, we use this event to set the correct start time.
        """
        self._start_time = ExampleState._node.get_clock().now()

    def on_stop(self):
        """
        Call this method whenever the behavior stops execution, also if it is cancelled.

        Use this event to clean up things like claimed resources.
        Nothing to do in this example.
        """
