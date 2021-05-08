#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from spear_flexbe_states.navigate_through_gate.set_goal_to_gate_normal_offset_state import SetGoalToGateNormalOffsetState
from spear_flexbe_behaviors.proportional_drive_sm import ProportionaldriveSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Feb 03 2020
@author: Mitchell
'''
class DrivethroughgateSM(Behavior):
	'''
	Drive through a gate
	'''


	def __init__(self):
		super(DrivethroughgateSM, self).__init__()
		self.name = 'Drive through gate'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(ProportionaldriveSM, 'Proportional drive')
		self.add_behavior(ProportionaldriveSM, 'Proportional drive_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:833 y:326, x:415 y:381
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['leg_number'])
		_state_machine.userdata.leg_number = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:98 y:100
			OperatableStateMachine.add('SetGoalToGateFront',
										SetGoalToGateNormalOffsetState(offset=2.0),
										transitions={'ok': 'Proportional drive', 'err': 'failed'},
										autonomy={'ok': Autonomy.Off, 'err': Autonomy.Off},
										remapping={'leg_number': 'leg_number', 'target_pose': 'target_pose'})

			# x:315 y:98
			OperatableStateMachine.add('Proportional drive',
										self.use_behavior(ProportionaldriveSM, 'Proportional drive'),
										transitions={'ok': 'SetGoalToBack'},
										autonomy={'ok': Autonomy.Inherit},
										remapping={'target_pose': 'target_pose'})

			# x:574 y:88
			OperatableStateMachine.add('SetGoalToBack',
										SetGoalToGateNormalOffsetState(offset=-2.0),
										transitions={'ok': 'Proportional drive_2', 'err': 'failed'},
										autonomy={'ok': Autonomy.Off, 'err': Autonomy.Off},
										remapping={'leg_number': 'leg_number', 'target_pose': 'target_pose'})

			# x:793 y:87
			OperatableStateMachine.add('Proportional drive_2',
										self.use_behavior(ProportionaldriveSM, 'Proportional drive_2'),
										transitions={'ok': 'finished'},
										autonomy={'ok': Autonomy.Inherit},
										remapping={'target_pose': 'target_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
