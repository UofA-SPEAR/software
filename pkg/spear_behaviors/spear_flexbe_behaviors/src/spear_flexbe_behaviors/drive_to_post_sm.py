#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from spear_flexbe_states.navigate_to_post.set_goal_to_post_state import SetGoalToPostState
from spear_flexbe_behaviors.proportional_drive_sm import ProportionaldriveSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Feb 02 2020
@author: Mitchell
'''
class DrivetopostSM(Behavior):
	'''
	Drive to a post
	'''


	def __init__(self):
		super(DrivetopostSM, self).__init__()
		self.name = 'Drive to post'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(ProportionaldriveSM, 'Proportional drive')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:281 y:482, x:130 y:477
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['leg_number'])
		_state_machine.userdata.leg_number = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:104 y:105
			OperatableStateMachine.add('SetGoal',
										SetGoalToPostState(),
										transitions={'ok': 'Proportional drive', 'err': 'failed'},
										autonomy={'ok': Autonomy.Off, 'err': Autonomy.Off},
										remapping={'leg_number': 'leg_number', 'target_pose': 'target_pose'})

			# x:206 y:287
			OperatableStateMachine.add('Proportional drive',
										self.use_behavior(ProportionaldriveSM, 'Proportional drive'),
										transitions={'ok': 'finished'},
										autonomy={'ok': Autonomy.Inherit},
										remapping={'target_pose': 'target_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
