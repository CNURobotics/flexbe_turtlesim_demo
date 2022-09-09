#!/usr/bin/env python

###############################################################################
#  Copyright (c) 2022
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

import rclpy
from rclpy.duration import Duration
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher

from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import TeleportAbsoluteRequest, TeleportAbsoluteResult

class TeleportAbsoluteState(EventState):
    '''
    This state teleports the Turtlesim turtle using TeleportAbsolute service.

    -- turtle_name   string     Turtle name (default: `turtle1`)
    -- x             float      x position (default: 0.0)
    -- y             float      y position (default: 0.0)
    -- theta         float      yaw orientation angle (default: 0.0)
    -- call_timeout  float      Timeout for completion (default: 3.0 seconds)
    -- wait_timeout  float      Duration to wait for service to become available (default: 3.0 seconds)
    -- service_name  string     Service name (default: `teleport_absolute`)
    <= done             Service call returned result as expected
    <= call_timeout     Service call did not return result successfully
    <= unavailable      Service is unavailable
    '''

    def __init__(self, turtle_name='turtle1', x=0.0, y=0.0, theta=0.0, call_timeout=3.0, wait_timeout=3.0):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(TeleportAbsoluteState, self).__init__(outcomes = ['done', 'call_timeout', 'unavailable'])

        ProxyServiceCaller._initialize(TeleportAbsoluteState._node)

        # Store state parameters for later use.
        self._call_timeout = Duration(seconds=call_timeout)
        self._wait_timeout = Duration(seconds=wait_timeout)

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None
        self._return     = None # Track the outcome so we can detect if transition is blocked
        self._service_called = False

        self._srv_topic = f'/{turtle_name}/{service_name}'
        self._srv_result = None

        self._srv_request =
        self._error = None

        # Set up the proxy now, but do not wait on the service just yet
        self._srv = ProxyServiceCaller({self._srv_topic: TeleportAbsolute}, wait_duration=0.0)


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # If no outcome is returned, the state will stay active.

        if self._return:
            # We have completed the state, and therefore must be blocked by autonomy level
            return self._return

        if self._service_called:
            # Waiting for result.
            # We will do this in a non-blocking way
            if self._srv.done(self._srv_topic):
                result = self._srv.result(self._srv_topic)
                Logger.logerr(f"{self._name}: Service {self_srv_topic} returned {result}!")
                self._return = 'done'
            else:

                if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._call_timeout.nanoseconds:
                    # Failed to return call in timely manner
                    self._return = 'call_timeout'
                    Logger.logerr(f"{self._name}: Service {self._srv_topic} call timed out!")
        else:
            # Waiting for service to become available in non-blocking manner
            if self._srv.is_available(self._srv_topic, wait_duration=None):
                Logger.logerr(f"{self._name}: Service {self._srv_topic} is now available - making service call!")
                self._do_service_call()
                # Process the result on next execute call (so some delay)
            else:
                if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._wait_timeout.nanoseconds:
                    # Failed to return call in timely manner
                    self._return = 'unavailable'
                    Logger.logerr(f"{self._name}: Service {self._srv_topic} is unavailable!")

        return self._return

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        self._start_time = self._node.get_clock().now()
        self._return     = None # reset the completion flag
        self._service_called = False
        if self._srv.is_available(self._srv_topic, wait_duration=None):
            Logger.logerr(f"{self._name}: Service {self._srv_topic} is available!")
            self._do_service_call()
        else:
            Logger.logwarn(f"{self._name}: Service {self._srv_topic} is not yet available ...")


    def _do_service_call(self):
        """
        Make the service call using async non-blocking
        """
        try:
            self._srv_result = self._srv.call_async(self._srv_topic, self._srv_request, wait_duration=None)
            self._start_time = self._node.get_clock().now()  # Reset timer for call timeout
            self._service_called = True
       except Exception as e:
            Logger.logerr(f"{self._name}: Service {self._srv_topic} exception {type(e)} - {str(e)}")
