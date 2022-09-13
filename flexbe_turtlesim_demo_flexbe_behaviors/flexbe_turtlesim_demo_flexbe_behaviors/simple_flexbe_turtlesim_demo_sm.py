#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_turtlesim_demo_flexbe_states.clear_turtlesim_state import ClearTurtlesimState
from flexbe_turtlesim_demo_flexbe_states.teleport_absolute_state import TeleportAbsoluteState
from flexbe_turtlesim_demo_flexbe_states.timed_cmd_vel_state import TimedCmdVelState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Sep 09 2022
@author: David Conner
'''
class SimpleFlexBETurtlesimDemoSM(Behavior):
	'''
	Simple demonstration of FlexBE using ROS Turtlesim from ROS tutorials
	'''


	def __init__(self, node):
		super(SimpleFlexBETurtlesimDemoSM, self).__init__()
		self.name = 'Simple FlexBE Turtlesim Demo'

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

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		cmd_vel = '/turtle1/cmd_vel'
		# x:959 y:233
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]

		# x:975 y:134, x:130 y:365
		_sm_container_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_container_0:
			# x:130 y:43
			OperatableStateMachine.add('Forward0',
										TimedCmdVelState(target_time=3.99, velocity=0.5, rotation_rate=0.0, cmd_topic=cmd_vel, desired_rate=50),
										transitions={'done': 'LeftTurn'},
										autonomy={'done': Autonomy.Off})

			# x:414 y:314
			OperatableStateMachine.add('Forward1',
										TimedCmdVelState(target_time=7.99, velocity=0.5, rotation_rate=0.0, cmd_topic=cmd_vel, desired_rate=50),
										transitions={'done': 'RightTurn'},
										autonomy={'done': Autonomy.Off})

			# x:663 y:38
			OperatableStateMachine.add('Forward2',
										TimedCmdVelState(target_time=3.99, velocity=0.5, rotation_rate=0.0, cmd_topic=cmd_vel, desired_rate=50),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:179 y:211
			OperatableStateMachine.add('LeftTurn',
										TimedCmdVelState(target_time=5.77, velocity=0.5, rotation_rate=0.667, cmd_topic=cmd_vel, desired_rate=50),
										transitions={'done': 'Forward1'},
										autonomy={'done': Autonomy.Off})

			# x:625 y:189
			OperatableStateMachine.add('RightTurn',
										TimedCmdVelState(target_time=5.77, velocity=0.5, rotation_rate=-0.667, cmd_topic=cmd_vel, desired_rate=50),
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

			# x:756 y:454
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'Finished Eight-pattern', 'failed': 'FailedEight'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:431 y:455
			OperatableStateMachine.add('FailedEight',
										LogState(text="Failed Eight pattern", severity=Logger.REPORT_HINT),
										transitions={'done': 'Operator'},
										autonomy={'done': Autonomy.Off})

			# x:572 y:397
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
										TeleportAbsoluteState(turtle_name='turtle1', x=5.544, y=5.544, theta=0.0, call_timeout=3.0, wait_timeout=3.0, service_name='teleport_absolute'),
										transitions={'done': 'AtHome', 'call_timeout': 'ServiceCallFailed', 'unavailable': 'Unavailable'},
										autonomy={'done': Autonomy.Off, 'call_timeout': Autonomy.Off, 'unavailable': Autonomy.Off})

			# x:651 y:133
			OperatableStateMachine.add('Operator',
										OperatorDecisionState(outcomes=["Home", "Eight", "Quit", "Clear"], hint="Eight", suggestion="Eight"),
										transitions={'Home': 'GoHome', 'Eight': 'Container', 'Quit': 'finished', 'Clear': 'ClearLog'},
										autonomy={'Home': Autonomy.Full, 'Eight': Autonomy.High, 'Quit': Autonomy.Full, 'Clear': Autonomy.Full})

			# x:449 y:171
			OperatableStateMachine.add('ServiceCallFailed',
										LogState(text="Failed to find TeleportAbsolute service for turtlesim", severity=Logger.REPORT_HINT),
										transitions={'done': 'Operator'},
										autonomy={'done': Autonomy.Off})

			# x:463 y:273
			OperatableStateMachine.add('Unavailable',
										LogState(text="Service is unavailable - Start turtlesim?", severity=Logger.REPORT_HINT),
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
