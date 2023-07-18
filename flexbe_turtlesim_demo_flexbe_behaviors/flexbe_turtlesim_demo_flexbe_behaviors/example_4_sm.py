#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Example behavior that contains a (sub-)behavior.

This behavior contains the Example 3 behavior as a state.

Created on Wed Jul 05 2023
@author: David Conner
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_states.log_state import LogState
from flexbe_turtlesim_demo_flexbe_behaviors.example_3_sm import Example3SM

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class Example4SM(Behavior):
    """
    Example behavior that contains a (sub-)behavior.

    This behavior contains the Example 3 behavior as a state.
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Example 4'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        LogState.initialize_ros(node)
        self.add_behavior(Example3SM, 'Example 3', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        # x:718 y:163, x:712 y:285
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:61 y:124
            OperatableStateMachine.add('A',
                                       LogState(text="Enter top-level", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Example Concurrent Behavior'},
                                       autonomy={'done': Autonomy.Off})

            # x:529 y:153
            OperatableStateMachine.add('B',
                                       LogState(text="top-level finished", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'finished'},
                                       autonomy={'done': Autonomy.Off})

            # x:519 y:267
            OperatableStateMachine.add('C',
                                       LogState(text="top-level failed", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'failed'},
                                       autonomy={'done': Autonomy.Off})

            # x:208 y:179
            OperatableStateMachine.add('Example Concurrent Behavior',
                                       self.use_behavior(Example3SM, 'Example 3'),
                                       transitions={'finished': 'B', 'failed': 'C'},
                                       autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
