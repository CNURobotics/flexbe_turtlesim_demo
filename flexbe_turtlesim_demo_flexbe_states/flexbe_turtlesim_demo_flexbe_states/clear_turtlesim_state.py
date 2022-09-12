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
from flexbe_core.proxy import ProxyServiceCaller

from std_srvs.srv import Empty

class ClearTurtlesimState(EventState):
    '''
    This state clears the Turtlesim window using the /clear service.

    This approach using the blocking call on enter.
    This is generally NOT advised if there are any potential concurrent operations.

    -- service_name   string    Service name (default: `/clear`)
    -- wait_timeout  float      Duration to wait for service to become available (default: 3.0 seconds)
    <= done             Service call returned result as expected
    <= unavailable      Service is unavailable
    '''

    def __init__(self, service_name='/clear', wait_timeout=3.0):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ClearTurtlesimState, self).__init__(outcomes = ['done', 'failed', 'unavailable'])

        ProxyServiceCaller._initialize(ClearTurtlesimState._node)

        # Store state parameters for later use.
        self._wait_timeout = Duration(seconds=wait_timeout)

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None
        self._return     = None # Track the outcome so we can detect if transition is blocked
        self._service_called = False

        self._srv_topic = service_name
        self._srv_result = None

        self._srv_request = Empty.Request()

        self._error = None

        # Set up the proxy now, but do not wait on the service just yet
        self._srv = ProxyServiceCaller({self._srv_topic: Empty}, wait_duration=0.0)


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # If no outcome is returned, the state will stay active.

        if self._return:
            # We have completed the state, and therefore must be blocked by autonomy level
            Logger.loginfo(f"{self._name}: returning existing value {self._return} .")
            return self._return

        if self._service_called:
            # Called from on_enter
            Logger.loginfo(f"{self._name}: Service called  - check result {self._srv_result} .")
            if self._srv_result is None:
                Logger.loginfo(f"{self._name}: Service {self._srv_topic} failed to return result!")
                self._return = 'failed'
            else:
                self._return = 'done'

        else:
            # Waiting for service to become available in non-blocking manner
            Logger.loginfo(f"{self._name}: Service not called - check if available {self._srv_topic} ...")
            if self._srv.is_available(self._srv_topic, wait_duration=0.0):
                Logger.localinfo(f"{self._name}: Service {self._srv_topic} is now available - making service call to clear!")
                self._do_service_call()
                if self._srv_result is None:
                    Logger.loginfo(f"{self._name}: Service {self._srv_topic} failed to return result!")
                    self._return = 'failed'
                else:
                    self._return = 'done'
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
        self._srv_result = None
        self._service_called = False
        try:
            if self._srv.is_available(self._srv_topic, wait_duration=0.0):
                Logger.loginfo(f"{self._name}: Service {self._srv_topic} is available ...")
                self._do_service_call()
            else:
                Logger.logwarn(f"{self._name}: Service {self._srv_topic} is not yet available ...")
        except Exception as exc:
            Logger.logerr(f"{self._name}: Service {self._srv_topic} exception {type(exc)} - {str(exc)}")


    def _do_service_call(self):
        """
        Make the service call using synchronous blocking call
        """
        try:
            Logger.loginfo(f"{self._name}: Calling service {self._srv_topic} ...")
            self._service_called = True
            self._srv_result = self._srv.call(self._srv_topic, self._srv_request, wait_duration=0.0)
        except Exception as e:
            Logger.logerr(f"{self._name}: Service {self._srv_topic} exception {type(e)} - {str(e)}")
            self._srv_result = None
