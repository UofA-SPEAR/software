#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from spear_flexbe_states.navigate_state import NavigateState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jan 08 2020
@author: Mitchell
'''
class WaypointnavigationSM(Behavior):
	'''
	Autonomously navigate to various waypoints.
	'''


	def __init__(self):
		super(WaypointnavigationSM, self).__init__()
		self.name = 'Waypoint navigation'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:496 y:328, x:511 y:211
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.target_pose_1 = {'x': 2, 'y': 2, 'frame_id': 'map'}
		_state_machine.userdata.target_pose_2 = {'x': -2, 'y': 2, 'frame_id': 'map'}
		_state_machine.userdata.target_pose_3 = {'x': -2, 'y': -2, 'frame_id': 'map'}
		_state_machine.userdata.target_pose_4 = {'x': 2, 'y': -2, 'frame_id': 'map'}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Waypoint #1',
										NavigateState(),
										transitions={'ok': 'Waypoint #2', 'err': 'failed'},
										autonomy={'ok': Autonomy.Off, 'err': Autonomy.Off},
										remapping={'target_pose': 'target_pose_1'})

			# x:164 y:123
			OperatableStateMachine.add('Waypoint #2',
										NavigateState(),
										transitions={'ok': 'Waypoint #3', 'err': 'failed'},
										autonomy={'ok': Autonomy.Off, 'err': Autonomy.Off},
										remapping={'target_pose': 'target_pose_2'})

			# x:165 y:232
			OperatableStateMachine.add('Waypoint #3',
										NavigateState(),
										transitions={'ok': 'Waypoint #4', 'err': 'failed'},
										autonomy={'ok': Autonomy.Off, 'err': Autonomy.Off},
										remapping={'target_pose': 'target_pose_3'})

			# x:30 y:316
			OperatableStateMachine.add('Waypoint #4',
										NavigateState(),
										transitions={'ok': 'Waypoint #1', 'err': 'failed'},
										autonomy={'ok': Autonomy.Off, 'err': Autonomy.Off},
										remapping={'target_pose': 'target_pose_4'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
