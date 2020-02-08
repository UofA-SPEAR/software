#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from spear_flexbe_states.timed_move_state import TimedMoveState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jan 18 2020
@author: Mitchell Epp
'''
class TimedmovementSM(Behavior):
	'''
	Demo using timed movement states
	'''


	def __init__(self):
		super(TimedmovementSM, self).__init__()
		self.name = 'Timed movement'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:324 y:460, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:178 y:120
			OperatableStateMachine.add('Forward',
										TimedMoveState(linear=1, angular=0, duration=2),
										transitions={'ok': 'Stop'},
										autonomy={'ok': Autonomy.Off})

			# x:335 y:207
			OperatableStateMachine.add('FrontLeft',
										TimedMoveState(linear=1, angular=5, duration=1),
										transitions={'ok': 'Stop2'},
										autonomy={'ok': Autonomy.Off})

			# x:342 y:119
			OperatableStateMachine.add('Stop',
										TimedMoveState(linear=0, angular=0, duration=1),
										transitions={'ok': 'FrontLeft'},
										autonomy={'ok': Autonomy.Off})

			# x:331 y:288
			OperatableStateMachine.add('Stop2',
										TimedMoveState(linear=0, angular=0, duration=1),
										transitions={'ok': 'Forward'},
										autonomy={'ok': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
