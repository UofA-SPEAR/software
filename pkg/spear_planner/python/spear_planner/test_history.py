#!/usr/bin/env python3
from unittest import TestCase, TestSuite
from spear_planner.history import Checkpoint, ContainerHistory, ContainerSaveState, ResumableStateMachine

from smach.state_machine import StateMachine
from smach.state import CBState
from smach.user_data import UserData
from uuid import uuid4
from datetime import datetime

PKG = 'spear_planner'


class TestResumableStateMachine(TestCase):
    def setUp(self) -> None:
        self._trace = []

    def simple_sm(self):
        def execute(name):
            self._trace.append(name)
            return 'ok'

        sm = ResumableStateMachine(outcomes=['ok'])
        with sm:
            StateMachine.add('A', CBState(lambda ud: execute('A'), outcomes=['ok']), {'ok': 'B'})
            StateMachine.add('B', CBState(lambda ud: execute('B'), outcomes=['ok']), {'ok': 'C'})
            StateMachine.add('C', CBState(lambda ud: execute('C'), outcomes=['ok']))
        return sm

    def sm_with_userdata(self):
        def execute(name, outcome):
            self._trace.append(name)
            return outcome

        sm = ResumableStateMachine(outcomes=['ok'], input_keys=['key'])
        with sm:
            StateMachine.add('A', CBState(lambda ud: execute('A', ud['key']), outcomes=['B'], io_keys=['key']), {'B': 'B'})
            StateMachine.add('B', CBState(lambda ud: execute('B', ud['key']), outcomes=['ok'], input_keys=['key']))
        return sm

    def nested_sm(self):
        def execute(name, outcome):
            self._trace.append(name)
            return outcome

        resumable_inner_sm = ResumableStateMachine(outcomes=['ok'])
        inner_sm = StateMachine(outcomes=['ok'])
        outer_sm = ResumableStateMachine(outcomes=['ok'])

        with outer_sm:
            StateMachine.add('A', resumable_inner_sm, {'ok': 'B'})
            StateMachine.add('B', inner_sm, {'ok': 'C'})
            StateMachine.add('C', CBState(lambda ud: execute('C', 'ok'), outcomes=['ok']))
        with resumable_inner_sm:
            StateMachine.add('A', CBState(lambda ud: execute('A.A', 'ok'), outcomes=['ok']), {'ok': 'B'})
            StateMachine.add('B', CBState(lambda ud: execute('A.B', 'ok'), outcomes=['ok']))
        with inner_sm:
            StateMachine.add('A', CBState(lambda ud: execute('B.A', 'ok'), outcomes=['ok']), {'ok': 'B'})
            StateMachine.add('B', CBState(lambda ud: execute('B.B', 'ok'), outcomes=['ok']))

        return outer_sm

    def trace(self):
        return self._trace

    @staticmethod
    def make_ud(**kwargs):
        ud = UserData()
        ud._data.update(kwargs)
        return ud

    def test_no_resume(self):
        """
        Execution works as normal without resuming
        """
        sm = self.simple_sm()

        outcome = sm.execute()
        self.assertEqual(outcome, 'ok')
        self.assertEqual(self.trace(), ['A', 'B', 'C'])

    def test_resume_to_start(self):
        """
        Can resume to initial state
        """
        sm = self.simple_sm()

        outcome = sm.execute_from(Checkpoint(uuid4(), datetime.now(), [
            ContainerSaveState(label='A', userdata=UserData())
        ]))
        self.assertEqual(outcome, 'ok')
        self.assertEqual(self.trace(), ['A', 'B', 'C'])

    def test_resume_to_middle(self):
        """
        Can resume to an intermediate state
        """
        sm = self.simple_sm()

        outcome = sm.execute_from(Checkpoint(uuid4(), datetime.now(), [
            ContainerSaveState(label='B', userdata=UserData())
        ]))
        self.assertEqual(outcome, 'ok')
        self.assertEqual(self.trace(), ['B', 'C'])

    def test_resume_to_end(self):
        """
        Can resume to final state
        """
        sm = self.simple_sm()

        outcome = sm.execute_from(Checkpoint(uuid4(), datetime.now(), [
            ContainerSaveState(label='C', userdata=UserData())
        ]))
        self.assertEqual(outcome, 'ok')
        self.assertEqual(self.trace(), ['C'])

    def test_resume_with_ud(self):
        """
        Can resume with certain userdata
        """
        sm = self.sm_with_userdata()
        outcome = sm.execute_from(Checkpoint(uuid4(), datetime.now(), [
            ContainerSaveState(label='B', userdata=self.make_ud(key='ok'))
        ]))
        self.assertEqual(outcome, 'ok')

    def test_resume_from_with_ud_input_shadow(self):
        """
        Before executing a new state, the input keys from the parent userdata
        are copied into the child userdata. We want resuming to function as if
        it happens *after* this step, when in reality it happens before, with
        some trickery to (hopefully) make the difference invisible.

        Input key values from parent userdata should not overwrite values in
        child userdata upon resume.
        """
        sm = self.sm_with_userdata()
        outcome = sm.execute_from(Checkpoint(uuid4(), datetime.now(), [
            ContainerSaveState(label='B', userdata=self.make_ud(key='ok'))
        ]), self.make_ud(key='err'))
        self.assertEqual(outcome, 'ok')

    def test_nested_sm(self):
        """
        Can resume to nested state-machines
        """
        sm = self.nested_sm()
        outcome = sm.execute_from(Checkpoint(uuid4(), datetime.now(), [
            ContainerSaveState(label='A', userdata=UserData()),
            ContainerSaveState(label='B', userdata=UserData()),
        ]))
        self.assertEqual(outcome, 'ok')
        self.assertEqual(self.trace(), ['A.B', 'B.A', 'B.B', 'C'])

        # Cannot resume to non-resumable sm
        self.assertRaises(ValueError, lambda: sm.execute_from(Checkpoint(uuid4(), datetime.now(), [
            ContainerSaveState(label='B', userdata=UserData()),
            ContainerSaveState(label='B', userdata=UserData()),
        ])))

        # Cannot resume to nonexistant state
        self.assertRaises(ValueError, lambda: sm.execute_from(Checkpoint(uuid4(), datetime.now(), [
            ContainerSaveState(label='A', userdata=UserData()),
            ContainerSaveState(label='X', userdata=UserData()),
        ])))

        # Cannot resume with spec nested deeper than state-machine is
        self.assertRaises(ValueError, lambda: sm.execute_from(Checkpoint(uuid4(), datetime.now(), [
            ContainerSaveState(label='A', userdata=UserData()),
            ContainerSaveState(label='B', userdata=UserData()),
            ContainerSaveState(label='C', userdata=UserData()),
        ])))


class TestContainerHistory(TestCase):
    def make_sm(self):
        inner_sm_1 = ResumableStateMachine(outcomes=['ok'], input_keys=['key'], output_keys=['key'])
        inner_sm_2 = ResumableStateMachine(outcomes=['ok'], input_keys=['key'], output_keys=['key'])
        outer_sm = ResumableStateMachine(outcomes=['ok'], input_keys=['key'], output_keys=['key'])

        def execute(ud):
            ud['key'] += 1
            return 'ok'

        def make_state():
            return CBState(execute, outcomes=['ok'], io_keys=['key'])

        with outer_sm:
            StateMachine.add('A', inner_sm_1, {'ok': 'B'})
            StateMachine.add('B', inner_sm_2, {'ok': 'C'})
            StateMachine.add('C', make_state())

        with inner_sm_1:
            StateMachine.add('A', make_state(), {'ok': 'B'})
            StateMachine.add('B', make_state())

        with inner_sm_2:
            StateMachine.add('A', make_state(), {'ok': 'B'})
            StateMachine.add('B', make_state())

        return outer_sm

    def test_history(self):
        sm = self.make_sm()
        history = ContainerHistory(sm)

        parent_ud = UserData()
        parent_ud.key = 0
        outcome = sm.execute(parent_ud)

        self.assertEqual(outcome, 'ok')
        self.assertEqual(parent_ud.key, 5)

        labels = ['.'.join(s.label for s in c.save_states) for c in history._checkpoints]
        self.assertEqual(labels, ['A', 'A.A', 'A.B', 'B', 'B.A', 'B.B', 'C'])

        uds = [tuple(s.userdata.key for s in c.save_states) for c in history._checkpoints]
        self.assertEqual(uds, [
            (0,),
            (0, 0),
            (0, 1),
            (2,),
            (2, 2),
            (2, 3),
            (4,),
        ])

    def test_resume_from_history(self):
        sm = self.make_sm()
        history = ContainerHistory(sm)

        parent_ud = UserData()
        parent_ud.key = 0
        outcome = sm.execute(parent_ud)

        self.assertEqual(outcome, 'ok')
        self.assertEqual(parent_ud.key, 5)

        checkpoints = list(history.checkpoints)
        for checkpoint in checkpoints:
            parent_ud.key = 0
            outcome = sm.execute_from(checkpoint, parent_ud)
            self.assertEqual(outcome, 'ok')
            self.assertEqual(parent_ud.key, 5)


class TestHistory(TestSuite):
    def __init__(self):
        super().__init__(tests=(TestResumableStateMachine(), TestContainerHistory()))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_history', TestHistory)
    # import unittest
    # unittest.main()
