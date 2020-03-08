#!/usr/bin/env python2

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QTabWidget, QDoubleSpinBox

from double_slider import DoubleSlider
from joint_props import joint_props
from events import IndivJointsKeyboardEvents


class ArmGuiPlugin(Plugin):
    def __init__(self, context):
        super(ArmGuiPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ArmGuiPlugin')

        # Create QTabWidget
        self._widget = QTabWidget()
        # Get path to UI file which should be in the "resource" folder of this
        # package
        ui_file = os.path.join(rospkg.RosPack().get_path('spear_station'),
                               'resource', 'ArmGuiPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget, {'DoubleSlider': DoubleSlider})
        # Give QObjects reasonable names
        self._widget.setObjectName('ArmGuiPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Cache references to joint sliders, spin boxes.
        self.joint_sliders = [
            self._widget.findChild(DoubleSlider, joint['name'] + 'Slider')
            for joint in joint_props
        ]

        self.joint_spin_boxes = [
            self._widget.findChild(QDoubleSpinBox, joint['name'] + 'SpinBox')
            for joint in joint_props
        ]

        self.rate_box = self._widget.findChild(QDoubleSpinBox, 'rateBox')

        # Install event filters (how we handle keyboard events in QT)
        self.indiv_joints_keyboard_events = IndivJointsKeyboardEvents(
            zip(self.joint_spin_boxes, [v['keyboard'] for v in joint_props]),
            self.rate_box.value())

        self.rate_box.valueChanged.connect(
            self.indiv_joints_keyboard_events.set_rate)
        self._widget.installEventFilter(self.indiv_joints_keyboard_events)

        self.connect_sliders_and_widgets()

    def connect_sliders_and_widgets(self):
        """Display slider changes on spinboxes, and vice-versa."""
        for slider, spin_box in zip(self.joint_sliders, self.joint_spin_boxes):
            slider.doubleValueChanged.connect(spin_box.setValue)
            spin_box.valueChanged.connect(slider.setDoubleValue)

    def shutdown_plugin(self):
        # TODO: unregister all publishers, timers, etc. here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO: save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO: restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
    #     # Comment in to signal that a plugin has a way to configure.
    #     # This will enable a setting button (gear icon) in each dock widget
    #     # title bar.
    #     # Usually used to open a modal configuration dialog.
