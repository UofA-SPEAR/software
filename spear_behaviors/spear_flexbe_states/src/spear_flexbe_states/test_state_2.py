from os import getpid

from flexbe_core import Logger, EventState

from test_import import GLOBAL_VARIABLE


class TestState2(EventState):
    def __init__(self):
        super(TestState2, self).__init__(outcomes=['outcome1', 'outcome2', 'outcome3'])

    def on_enter(self, userdata):
        Logger.loginfo('Process id: {}'.format(getpid()))
        Logger.loginfo('Global variable: {}'.format(GLOBAL_VARIABLE))

    def execute(self, ud):
        pass
