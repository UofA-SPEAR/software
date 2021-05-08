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
from spear_flexbe_states.test_state_2 import TestState2
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Feb 02 2020
@author: Mitchell
'''
class TestinnerSM(Behavior):
	'''
	An inner behavior to test nested behaviors
	'''


	def __init__(self):
		super(TestinnerSM, self).__init__()
		self.name = 'Test - inner'

		# parameters of this behavior

		# references to used behaviors

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
			# x:108 y:140
			OperatableStateMachine.add('Inner test',
										TestState(),
										transitions={'outcome1': 'Another inner test state', 'outcome2': 'finished', 'outcome3': 'failed'},
										autonomy={'outcome1': Autonomy.Off, 'outcome2': Autonomy.Off, 'outcome3': Autonomy.Off})

			# x:382 y:132
			OperatableStateMachine.add('Another inner test state',
										TestState2(),
										transitions={'outcome1': 'Inner test', 'outcome2': 'finished', 'outcome3': 'failed'},
										autonomy={'outcome1': Autonomy.Off, 'outcome2': Autonomy.Off, 'outcome3': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
