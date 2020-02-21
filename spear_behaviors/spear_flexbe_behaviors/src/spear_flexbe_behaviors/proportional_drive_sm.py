#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from spear_flexbe_states.proportional_drive.split_pose_state import SplitPoseState
from spear_flexbe_states.proportional_drive.turn_correction_state import TurnCorrectionState
from spear_flexbe_states.proportional_drive.proportional_drive_state import ProportionalDriveState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Feb 02 2020
@author: Mitchell
'''
class ProportionaldriveSM(Behavior):
	'''
	Drive directly to a target pose, ignoring obstacles
	'''


	def __init__(self):
		super(ProportionaldriveSM, self).__init__()
		self.name = 'Proportional drive'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365
		_state_machine = OperatableStateMachine(outcomes=['ok'], input_keys=['target_pose'])
		_state_machine.userdata.target_pose = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:81 y:35
			OperatableStateMachine.add('SplitTargetPose',
										SplitPoseState(),
										transitions={'ok': 'ProportionalDrive'},
										autonomy={'ok': Autonomy.Off},
										remapping={'target_pose': 'target_pose', 'target_position': 'target_position', 'target_orientation': 'target_orientation'})

			# x:88 y:249
			OperatableStateMachine.add('CorrectTurn',
										TurnCorrectionState(),
										transitions={'ok': 'ok'},
										autonomy={'ok': Autonomy.Off},
										remapping={'target_orientation': 'target_orientation'})

			# x:79 y:145
			OperatableStateMachine.add('ProportionalDrive',
										ProportionalDriveState(),
										transitions={'ok': 'CorrectTurn'},
										autonomy={'ok': Autonomy.Off},
										remapping={'target_position': 'target_position'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
