#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define FlexBE Turtlesim Demo.

Created on Fri Sep 09 2022
@author: David Conner
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_turtlesim_demo_flexbe_behaviors.turtlesim_input_state_behavior_sm import TurtlesimInputStateBehaviorSM
from flexbe_turtlesim_demo_flexbe_states.clear_turtlesim_state import ClearTurtlesimState
from flexbe_turtlesim_demo_flexbe_states.teleport_absolute_state import TeleportAbsoluteState
from flexbe_turtlesim_demo_flexbe_states.timed_cmd_vel_state import TimedCmdVelState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class FlexBETurtlesimDemoSM(Behavior):
    """
    Define FlexBE Turtlesim Demo.

    FlexBE demonstration using ROS Turtlesim from ROS tutorials.

    Demonstrates several FlexBE capabilities including synchronous and async service calls,
    action interfaces, and  operator input.
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'FlexBE Turtlesim Demo'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        ClearTurtlesimState.initialize_ros(node)
        LogState.initialize_ros(node)
        OperatorDecisionState.initialize_ros(node)
        TeleportAbsoluteState.initialize_ros(node)
        TimedCmdVelState.initialize_ros(node)
        self.add_behavior(TurtlesimInputStateBehaviorSM, 'Turtlesim Input State Behavior', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        cmd_vel = '/turtle1/cmd_vel'
        # x:1028 y:227
        _state_machine = OperatableStateMachine(outcomes=['finished'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        # x:975 y:134, x:130 y:365
        _sm_container_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

        with _sm_container_0:
            # x:130 y:43
            OperatableStateMachine.add('Forward0',
                                       TimedCmdVelState(target_time=3.99, velocity=0.5, rotation_rate=0.0,
                                                        cmd_topic=cmd_vel, desired_rate=50),
                                       transitions={'done': 'LeftTurn'},
                                       autonomy={'done': Autonomy.Off})

            # x:414 y:314
            OperatableStateMachine.add('Forward1',
                                       TimedCmdVelState(target_time=7.99, velocity=0.5, rotation_rate=0.0,
                                                        cmd_topic=cmd_vel, desired_rate=50),
                                       transitions={'done': 'RightTurn'},
                                       autonomy={'done': Autonomy.Off})

            # x:663 y:38
            OperatableStateMachine.add('Forward2',
                                       TimedCmdVelState(target_time=3.99, velocity=0.5, rotation_rate=0.0,
                                                        cmd_topic=cmd_vel, desired_rate=50),
                                       transitions={'done': 'finished'},
                                       autonomy={'done': Autonomy.Off})

            # x:179 y:211
            OperatableStateMachine.add('LeftTurn',
                                       TimedCmdVelState(target_time=5.77, velocity=0.5, rotation_rate=0.667,
                                                        cmd_topic=cmd_vel, desired_rate=50),
                                       transitions={'done': 'Forward1'},
                                       autonomy={'done': Autonomy.Off})

            # x:625 y:189
            OperatableStateMachine.add('RightTurn',
                                       TimedCmdVelState(target_time=5.77, velocity=0.5, rotation_rate=-0.667,
                                                        cmd_topic=cmd_vel, desired_rate=50),
                                       transitions={'done': 'Forward2'},
                                       autonomy={'done': Autonomy.Off})

        with _state_machine:
            # x:60 y:200
            OperatableStateMachine.add('ClearEntry',
                                       ClearTurtlesimState(service_name='/clear', wait_timeout=3.0),
                                       transitions={'done': 'Home', 'failed': 'Home', 'unavailable': 'Unavailable'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'unavailable': Autonomy.Off})

            # x:911 y:133
            OperatableStateMachine.add('ClearFailed',
                                       LogState(text="Failed to clear Turtlesim window!", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Operator'},
                                       autonomy={'done': Autonomy.Off})

            # x:694 y:20
            OperatableStateMachine.add('ClearLog',
                                       LogState(text="Clear turtlesim window ...", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'ClearWindow'},
                                       autonomy={'done': Autonomy.Off})

            # x:893 y:15
            OperatableStateMachine.add('ClearWindow',
                                       ClearTurtlesimState(service_name='/clear', wait_timeout=3.0),
                                       transitions={'done': 'Operator', 'failed': 'ClearFailed', 'unavailable': 'ClearFailed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'unavailable': Autonomy.Off})

            # x:731 y:485
            OperatableStateMachine.add('Container',
                                       _sm_container_0,
                                       transitions={'finished': 'StopCmd', 'failed': 'FailedEight'},
                                       autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

            # x:400 y:449
            OperatableStateMachine.add('FailedEight',
                                       LogState(text="Failed Eight pattern", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Operator'},
                                       autonomy={'done': Autonomy.Off})

            # x:615 y:300
            OperatableStateMachine.add('Finished Eight-pattern',
                                       LogState(text="Finished 8 pattern", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Operator'},
                                       autonomy={'done': Autonomy.Off})

            # x:459 y:4
            OperatableStateMachine.add('GoHome',
                                       LogState(text="Go to home position", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Home'},
                                       autonomy={'done': Autonomy.Off})

            # x:178 y:77
            OperatableStateMachine.add('Home',
                                       TeleportAbsoluteState(turtle_name='turtle1', x=5.544, y=5.544, theta=0.0,
                                                             call_timeout=3.0, wait_timeout=3.0,
                                                             service_name='teleport_absolute'),
                                       transitions={'done': 'AtHome', 'call_timeout': 'ServiceCallFailed',
                                                    'unavailable': 'Unavailable'},
                                       autonomy={'done': Autonomy.Off, 'call_timeout': Autonomy.Off,
                                                 'unavailable': Autonomy.Off})

            # x:651 y:133
            OperatableStateMachine.add('Operator',
                                       OperatorDecisionState(outcomes=["Home", "Eight", "Quit", "Clear", "Rotate"],
                                                             hint="Eight", suggestion="Eight"),
                                       transitions={'Home': 'GoHome', 'Eight': 'Container', 'Quit': 'finished',
                                                    'Clear': 'ClearLog', 'Rotate': 'Turtlesim Input State Behavior'},
                                       autonomy={'Home': Autonomy.Full, 'Eight': Autonomy.High, 'Quit': Autonomy.Full,
                                                 'Clear': Autonomy.Full, 'Rotate': Autonomy.Low})

            # x:895 y:338
            OperatableStateMachine.add('RotateFailed',
                                       LogState(text="Rotate failed", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Operator'},
                                       autonomy={'done': Autonomy.Off})

            # x:992 y:292
            OperatableStateMachine.add('RotateLog',
                                       LogState(text="Rotation complete", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Operator'},
                                       autonomy={'done': Autonomy.Off})

            # x:449 y:171
            OperatableStateMachine.add('ServiceCallFailed',
                                       LogState(text="Failed to find TeleportAbsolute service for turtlesim",
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Operator'},
                                       autonomy={'done': Autonomy.Off})

            # x:561 y:391
            OperatableStateMachine.add('StopCmd',
                                       TimedCmdVelState(target_time=0.01, velocity=0.0, rotation_rate=0.0,
                                                        cmd_topic=cmd_vel, desired_rate=50),
                                       transitions={'done': 'Finished Eight-pattern'},
                                       autonomy={'done': Autonomy.Off})

            # x:896 y:439
            OperatableStateMachine.add('Turtlesim Input State Behavior',
                                       self.use_behavior(TurtlesimInputStateBehaviorSM, 'Turtlesim Input State Behavior'),
                                       transitions={'finished': 'RotateLog', 'failed': 'RotateFailed'},
                                       autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

            # x:428 y:277
            OperatableStateMachine.add('Unavailable',
                                       LogState(text="Service is unavailable - Start turtlesim?",
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Operator'},
                                       autonomy={'done': Autonomy.Off})

            # x:461 y:80
            OperatableStateMachine.add('AtHome',
                                       LogState(text="Turtle is home!", severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Operator'},
                                       autonomy={'done': Autonomy.Off})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
