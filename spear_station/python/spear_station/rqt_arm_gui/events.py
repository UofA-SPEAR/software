#!/usr/bin/env python2
"""This file handles global events (e.g. keyboard key presses)"""

from python_qt_binding.QtCore import QObject, QEvent, QTimer, Slot, Signal
from python_qt_binding.QtGui import QKeySequence


class _SpinBoxUpdaterStates():
    """The possible states for a `_SpinBoxUpdater` to be in.

    Treat this like an enum. Python 2 doesn't have Python 3's proper `Enum`
    type, so for now we have to believe in pinkie promises. :-)
    """
    NEUTRAL = 0
    INCREMENTING = 1
    DECREMENTING = 2


class _SpinBoxUpdater(QObject):
    """Takes care of updating a QDoubleSpinBox at a certain rate.

    Each object created with this class will have a QTimer that it will use
    to update a QDoubleSpinBox at some specified rate. The rate can be set
    using a standard QT Socket, as can whether or not it should be incrementing,
    decrementing, or doing nothing.

    We do it this way for Threading Reasons (TM).
    """
    value_changed = Signal(float)

    def __init__(self,
                 spin_box,
                 rate,
                 state=_SpinBoxUpdaterStates.NEUTRAL,
                 *args,
                 **kwargs):
        super(_SpinBoxUpdater, self).__init__(*args, **kwargs)

        self._spin_box = spin_box
        self.value_changed.connect(self._spin_box.setValue)

        self._rate = rate  # in Hz
        self._period = (1 / rate) * 1000  # in milliseconds

        self._state = state

        self._timer = QTimer(self)
        self._timer.setInterval(self._period)
        self._timer.timeout.connect(self._timer_fired)

        self._timer.start()

    def rate(self):
        return self._rate

    def period(self):
        return self._period

    @Slot(float)
    def set_rate(self, rate):
        self._rate = rate  # in Hz
        self._period = (1 / rate) * 1000  # in milliseconds
        self._timer.setInterval(self._period)

    def state(self):
        return self._state

    @Slot(int)
    def set_state(self, state):
        """@param state: MUST be a `_SpinBoxUpdaterStates` enum value!!!"""
        if state == _SpinBoxUpdaterStates.INCREMENTING or state == _SpinBoxUpdaterStates.DECREMENTING:
            self._state = state
            self._timer.start()
        elif state == _SpinBoxUpdaterStates.NEUTRAL:
            self._state = state
            self._timer.stop()

    @Slot()
    def _timer_fired(self):
        if self._state == _SpinBoxUpdaterStates.INCREMENTING:
            self.value_changed.emit(self._spin_box.value() + 1)
        elif self._state == _SpinBoxUpdaterStates.DECREMENTING:
            self.value_changed.emit(self._spin_box.value() - 1)


class IndivJointsKeyboardEvents(QObject):
    """Keyboard events for individual joint control.

    @param joint_spin_boxes_and_keys: A list of tuples. Each tuple has the
    format:

        (QDoubleSpinBox, { 'incr': String, 'decr': String })

    The first item in the tuple, the `QDoubleSpinBox`, is the spin box to be
    updated. The second tuple is a dictionary, with keys:

    - `'incr': String`: The key to press to increment the spin box
    - `'decr': String`: The key to press to decrement the pin box

    @param rate: The rate to increment or decrement by, in degrees per second.
    """
    rate_updated = Signal(float)

    def __init__(self, joint_spin_boxes_and_keys, rate, *args, **kwargs):
        super(IndivJointsKeyboardEvents, self).__init__(*args, **kwargs)
        self._updaters = [
            _SpinBoxUpdater(spin_box, rate)
            for spin_box, _ in joint_spin_boxes_and_keys
        ]
        self._keys = [{
            'incr': QKeySequence.fromString(keys['incr']),
            'decr': QKeySequence.fromString(keys['decr'])
        } for _, keys in joint_spin_boxes_and_keys]
        self._rate = rate

        # Whenever the rate is updated, notify each _SpinBoxUpdater.
        for updater in self._updaters:
            self.rate_updated.connect(updater.set_rate)

    def eventFilter(self, obj, event):
        e_type = event.type()

        if (e_type == QEvent.KeyPress
                or e_type == QEvent.KeyRelease) and not event.isAutoRepeat():
            key = QKeySequence(event.key())

            if e_type == QEvent.KeyPress:
                for keys, updater in zip(self._keys, self._updaters):
                    incr_pressed = keys['incr'].matches(
                        key) == QKeySequence.ExactMatch
                    decr_pressed = keys['decr'].matches(
                        key) == QKeySequence.ExactMatch

                    is_incrementing = updater.state(
                    ) == _SpinBoxUpdaterStates.INCREMENTING
                    is_decrementing = updater.state(
                    ) == _SpinBoxUpdaterStates.DECREMENTING

                    if (incr_pressed
                            and is_decrementing) or (decr_pressed
                                                     and is_incrementing):
                        # Both increment and decrement keys are being pressed,
                        # so do nothing.
                        pass
                    elif incr_pressed:
                        updater.set_state(_SpinBoxUpdaterStates.INCREMENTING)
                    elif decr_pressed:
                        updater.set_state(_SpinBoxUpdaterStates.DECREMENTING)
            elif e_type == QEvent.KeyRelease:
                for keys, updater in zip(self._keys, self._updaters):
                    if (keys['incr'].matches(key) == QKeySequence.ExactMatch
                        ) or (keys['decr'].matches(key) ==
                              QKeySequence.ExactMatch):
                        updater.set_state(_SpinBoxUpdaterStates.NEUTRAL)

        return QObject.eventFilter(self, obj, event)

    def rate(self):
        return self._rate

    @Slot(float)
    def set_rate(self, rate):
        self._rate = rate
        self.rate_updated.emit(rate)
