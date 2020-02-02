#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from spear_flexbe_states.test_state import TestState
from spear_flexbe_behaviors.test___inner_sm import TestinnerSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Feb 02 2020
@author: Mitchell
'''
class TestSM(Behavior):
	'''
	Test importing, global variables, and processes
	'''


	def __init__(self):
		super(TestSM, self).__init__()
		self.name = 'Test'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(TestinnerSM, 'Test - inner')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:102 y:115
			OperatableStateMachine.add('Test',
										TestState(),
										transitions={'outcome1': 'Test - inner', 'outcome2': 'finished', 'outcome3': 'failed'},
										autonomy={'outcome1': Autonomy.Off, 'outcome2': Autonomy.Off, 'outcome3': Autonomy.Off})

			# x:423 y:116
			OperatableStateMachine.add('Test - inner',
										self.use_behavior(TestinnerSM, 'Test - inner'),
										transitions={'finished': 'Test', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
