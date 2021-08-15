#!/usr/bin/env python2
from __future__ import division
from typing import Callable, Generic, TypeVar

import pygame

from canros.msg import uavcan__equipment__actuator__Command, uavcan__equipment__actuator__ArrayCommand
import rospy

T = TypeVar("T")


class OneWayBinding:
    def __init__(self, get_fn):  # type: (Callable[[], T]) -> None
        self._get_fn = get_fn

    @property
    def value(self):
        return self._get_fn()

    @classmethod
    def from_member(cls, object_,
                    name):  # type: (object, str) -> OneWayBinding
        return cls(get_fn=lambda: getattr(object_, name))


class TwoWayBinding(Generic[T]):
    def __init__(
            self, get_fn,
            set_fn):  # type: (Callable[[], T], Callable[[T], None]) -> None
        self._get_fn = get_fn
        self._set_fn = set_fn

    @property
    def value(self):
        return self._get_fn()

    @value.setter
    def value(self, x):
        self._set_fn(x)

    @classmethod
    def from_member(cls, object_,
                    name):  # type: (object, str) -> TwoWayBinding
        return cls(get_fn=lambda: getattr(object_, name),
                   set_fn=lambda value: setattr(object_, name, value))


class KeypressedSpeedView:
    def __init__(self, keys, model, scale):
        self._positive_key, self._negative_key = keys
        self._model = model
        self._scale = scale

    @staticmethod
    def _key_pressed(key):  # type: (str) -> bool
        index = pygame.__dict__['K_' + key]
        return pygame.key.get_pressed()[index]

    def tick(self):
        if self._key_pressed(self._positive_key):
            self._model.value = self._scale
        elif self._key_pressed(self._negative_key):
            self._model.value = -self._scale
        else:
            self._model.value = 0


class CanrosJointSpeedController:
    def __init__(self, actuator_id):  # type: (int) -> None
        self._actuator_id = actuator_id
        self._speed = 0

    def command(self):
        command = uavcan__equipment__actuator__Command()
        command.actuator_id = self._actuator_id
        command.command_value = self._speed
        command.command_type = command.COMMAND_TYPE_SPEED
        return command

    def binding(self):
        return TwoWayBinding.from_member(self, '_speed')


def main():
    rospy.init_node('arm_manual')
    rospy.sleep(0)

    pygame.init()
    pygame.display.set_mode((100, 100))
    pygame.display.set_caption('Manual arm controller')

    commands_publisher = rospy.Publisher(
        '/canros/msg/uavcan/equipment/actuator/ArrayCommand',
        uavcan__equipment__actuator__ArrayCommand,
        queue_size=1)

    shoulder_yaw_controller = CanrosJointSpeedController(16)
    wrist_pitch_controller = CanrosJointSpeedController(13)
    wrist_roll_controller = CanrosJointSpeedController(14)
    grab_controller = CanrosJointSpeedController(15)

    controllers = [
        shoulder_yaw_controller, wrist_pitch_controller, wrist_roll_controller,
        grab_controller
    ]

    views = [
        KeypressedSpeedView('ad', shoulder_yaw_controller.binding(), 0.02),
        KeypressedSpeedView('ki', wrist_pitch_controller.binding(), 0.1),
        KeypressedSpeedView('lj', wrist_roll_controller.binding(), 0.1),
        KeypressedSpeedView('uo', grab_controller.binding(), 0.1),
    ]

    rate = rospy.Rate(10)
    running = True
    while running and not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        commands = uavcan__equipment__actuator__ArrayCommand()
        commands.commands = [
            controller.command() for controller in controllers
        ]
        commands_publisher.publish(commands)

        for view in views:
            view.tick()

        rate.sleep()


if __name__ == '__main__':
    main()
