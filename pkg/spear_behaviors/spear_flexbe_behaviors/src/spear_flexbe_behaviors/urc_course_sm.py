#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from spear_flexbe_behaviors.drive_to_post_sm import DrivetopostSM
from spear_flexbe_behaviors.drive_through_gate_sm import DrivethroughgateSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Feb 03 2020
@author: Mitchell
'''
class URCcourseSM(Behavior):
	'''
	Complete the URC test course world
	'''


	def __init__(self):
		super(URCcourseSM, self).__init__()
		self.name = 'URC course'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(DrivetopostSM, 'Leg 1 - drive to post')
		self.add_behavior(DrivetopostSM, 'Leg 2 - drive to post')
		self.add_behavior(DrivetopostSM, 'Leg 3 - drive to post')
		self.add_behavior(DrivethroughgateSM, 'Leg 4 - drive through gate')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:636, x:388 y:202
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.ln1 = 1
		_state_machine.userdata.ln2 = 2
		_state_machine.userdata.ln3 = 3
		_state_machine.userdata.ln4 = 4

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Leg 1 - drive to post',
										self.use_behavior(DrivetopostSM, 'Leg 1 - drive to post'),
										transitions={'finished': 'Leg 2 - drive to post', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'leg_number': 'ln1'})

			# x:30 y:138
			OperatableStateMachine.add('Leg 2 - drive to post',
										self.use_behavior(DrivetopostSM, 'Leg 2 - drive to post'),
										transitions={'finished': 'Leg 3 - drive to post', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'leg_number': 'ln2'})

			# x:30 y:236
			OperatableStateMachine.add('Leg 3 - drive to post',
										self.use_behavior(DrivetopostSM, 'Leg 3 - drive to post'),
										transitions={'finished': 'Leg 4 - drive through gate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'leg_number': 'ln3'})

			# x:30 y:334
			OperatableStateMachine.add('Leg 4 - drive through gate',
										self.use_behavior(DrivethroughgateSM, 'Leg 4 - drive through gate'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'leg_number': 'ln4'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
