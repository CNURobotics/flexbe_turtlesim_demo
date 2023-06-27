#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2022-2023
#  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
#  Christopher Newport University
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#    3. Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
#       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#       POSSIBILITY OF SUCH DAMAGE.
###############################################################################

from rclpy.duration import Duration
from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher

from geometry_msgs.msg import Twist

# Based on flexible_navigation : flex_nav_flexbe_states: TimedTwistState
# but removes TwistStamped handling


class TimedCmdVelState(EventState):
    """
    This state publishes an open loop constant Twist command based on parameters.

    -- target_time          float     Time which needs to have passed since the behavior started.
    -- velocity             float     Body velocity (m/s)
    -- rotation_rate        float     Angular rotation (radians/s)
    -- cmd_topic            string    Topic name of the robot velocity command (default: 'cmd_vel')
    -- desired_rate         float     Desired state update rate (default: 50 Hz)
    <= done                 Given time has passed.
    """

    def __init__(self, target_time, velocity, rotation_rate, cmd_topic='cmd_vel', desired_rate=50):
        """
        Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        NOTE: Uses desired update rate added to ROS 2 version of FlexBE flexbe_core.ros_state
         This state should run faster than default 10 Hz state update
         The outcomes must come last in kwargs list due to FlexBE UI parsing!
        """
        super().__init__(desired_rate=desired_rate, outcomes=['done'])

        # Store state parameter for later use.
        self._target_time = Duration(seconds=target_time)

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

        self._done = None  # Track the outcome so we can detect if transition is blocked

        self._twist = Twist()
        self._twist.linear.x = velocity
        self._twist.angular.z = rotation_rate
        self._cmd_topic = cmd_topic

        # FlexBE uses "proxies" for publishers, subscribers, and service callers
        # so that all states in a behavior can share a single subscription/publisher
        ProxyPublisher.initialize(TimedCmdVelState._node)  # the class must know the behavior node
        self._pub = ProxyPublisher()
        self._pub.createPublisher(cmd_topic, Twist)

    def execute(self, userdata):
        """
        Call this method periodically while the state is active.

        If no outcome is returned, the state will stay active.
        """
        if (self._done):
            # We have completed the state, and therefore must be blocked by autonomy level
            # Stop the robot, but and return the prior outcome
            if self._cmd_topic:
                self._pub.publish(self._cmd_topic, Twist())

            return self._done

        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._target_time.nanoseconds:
            # Normal completion, do not bother repeating the publish
            # We won't bother publishing a 0 command unless blocked (above)
            # so that we can chain multiple motions together
            self._done = 'done'
            return 'done'

        # Normal operation
        if self._cmd_topic:
            # Logger.localinfo(f"{self._name} : {self._twist}")  # For initial debugging
            self._pub.publish(self._cmd_topic, self._twist)

        return None

    def on_enter(self, userdata):
        """
        Call this method when the state becomes active.

        i.e. a transition from another state to this one is taken.
        """
        self._start_time = self._node.get_clock().now()
        self._done = None  # reset the completion flag
