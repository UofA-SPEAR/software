# The defualt QSlider widget only allows for integer steps. This file provides
# a subclassed QSlider that allows for double (i.e. floating point) values.

from python_qt_binding.QtWidgets import QSlider
from python_qt_binding.QtCore import Signal, Slot


class DoubleSlider(QSlider):
    """
    An extended QSlider that allows for double value output, up to two decimal
    places.

    Literally takes the values from QSlider.valueChanged Signals, divides by
    100.0, and returns the result by emitting a doubleValueChanged Signal.

    Also, it adds a Slot called setDoubleValue, which will take whatever value
    it receives, multiply by 100.0, and use QSlider.setValue to set itself to
    that new integer value.

    If we did it any other way, we'd have to manually handle the drawing and
    events that come with making a proper slider component. This makes
    things much simpler. And two decimals of precision should be good enough for
    arm angles... hopefully :-)
    """

    # Custom doubleValueChanged Signal
    doubleValueChanged = Signal(float)

    def __init__(self, *args, **kwargs):
        super(DoubleSlider, self).__init__(*args, **kwargs)

        # Tie together the built-in valueChange Signal and our custom
        # notifyValueChanged Slot
        self.valueChanged.connect(self.notifyValueChanged)

    @Slot(int)
    def notifyValueChanged(self, value):
        doubleValue = value / 100.0
        self.doubleValueChanged.emit(doubleValue)

    @Slot(float)
    def setDoubleValue(self, value):
        integerValue = value * 100.0
        self.setValue(integerValue)
