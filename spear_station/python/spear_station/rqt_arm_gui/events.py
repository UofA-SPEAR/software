"""This file handles global events (e.g. keyboard key presses)"""

from python_qt_binding.QtCore import QObject, QEvent


class IndivJointsKeyboardEvents(QObject):
    """Keyboard events for individual joint control."""
    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress and not event.isAutoRepeat():
            print('Key pressed: ' + event.text())
        elif event.type() == QEvent.KeyRelease and not event.isAutoRepeat():
            print('Key released: ' + event.text())
        return QObject.eventFilter(self, obj, event)
