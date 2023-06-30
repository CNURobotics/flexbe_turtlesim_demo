#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Example State Behavior.

Created on Thursday 30-June-2023
@author: David Conner
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_states.log_state import LogState
from flexbe_turtlesim_demo_flexbe_states.example_state import ExampleState as flexbe_turtlesim_demo_flexbe_states__ExampleState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class ExampleStateBehaviorSM(Behavior):
    """
    Define Example State Behavior.

    This is a simple example for a behavior using custom example_state that logs each function in life cycle.
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Example State Behavior'

        # parameters of this behavior
        self.add_parameter('waiting_time', 3)

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        LogState.initialize_ros(node)
        flexbe_turtlesim_demo_flexbe_states__ExampleState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        start_msg = "Demo started!"
        done_msg = "Demo finished!"
        # x:823 y:238, x:836 y:155
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:52 y:78
            OperatableStateMachine.add('Start',
                                        LogState(text=start_msg, severity=Logger.REPORT_HINT),
                                        transitions={'done': 'A'},
                                        autonomy={'done': Autonomy.Low})

            # x:562 y:190
            OperatableStateMachine.add('Done',
                                        LogState(text=done_msg, severity=Logger.REPORT_HINT),
                                        transitions={'done': 'finished'},
                                        autonomy={'done': Autonomy.Off})

            # x:224 y:81
            OperatableStateMachine.add('A',
                                        flexbe_turtlesim_demo_flexbe_states__ExampleState(target_time=self.waiting_time),
                                        transitions={'done': 'Done', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
