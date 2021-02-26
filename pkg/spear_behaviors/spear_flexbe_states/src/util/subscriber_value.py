from rospy import Subscriber, logwarn, sleep

from typing import TypeVar

T = TypeVar('T')


class SubscriberValue:
    def __init__(self, topic, data_class):  # type: (str, T) -> None
        self._topic = topic
        self._subscriber = Subscriber(topic, data_class, self._callback, queue_size=1)
        self._value = None  # type: T

    def _callback(self, message):  # type: (T) -> None
        self._value = message

    @property
    def value(self):  # type: () -> T
        while self._value is None:
            logwarn('Waiting for topic {}'.format(self._topic))
            sleep(1)
        return self._value
