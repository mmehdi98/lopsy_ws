#!/usr/bin/env python3
"""Qt slider window, styled like joint_state_publisher_gui, publishing left/right rod tensions on /rod_tensions."""

import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from lopsy_interfaces.msg import RodTensions

RESOLUTION = 0.1  # newtons per slider tick


class TensionControl(QWidget):
    """One rod: bold name and numeric box on top, slider underneath."""

    def __init__(self, name, initial, max_value, parent=None):
        super().__init__(parent)

        label = QLabel(name)
        font = label.font()
        font.setBold(True)
        label.setFont(font)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(0.0, max_value)
        self.spin.setSingleStep(RESOLUTION)
        self.spin.setDecimals(1)
        self.spin.setSuffix(' N')
        self.spin.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.spin.setFixedWidth(100)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, int(round(max_value / RESOLUTION)))

        top = QHBoxLayout()
        top.addWidget(label)
        top.addStretch()
        top.addWidget(self.spin)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        layout.addLayout(top)
        layout.addWidget(self.slider)

        # Keep slider and box in sync without each one re-triggering the other.
        self.slider.valueChanged.connect(self._on_slider)
        self.spin.valueChanged.connect(self._on_spin)
        self.set_value(initial)

    def value(self):
        return self.spin.value()

    def set_value(self, newtons):
        self.spin.setValue(newtons)          # clamps to [0, max]
        self._on_spin(self.spin.value())

    def _on_slider(self, tick):
        self.spin.blockSignals(True)
        self.spin.setValue(tick * RESOLUTION)
        self.spin.blockSignals(False)

    def _on_spin(self, newtons):
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(newtons / RESOLUTION)))
        self.slider.blockSignals(False)


class TensionSliderWindow(QWidget):

    def __init__(self, max_tension, left_initial, right_initial):
        super().__init__()
        self.setWindowTitle('Rod tensions')
        self.setMinimumWidth(420)

        self.left = TensionControl('Left tension', left_initial, max_tension)
        self.right = TensionControl('Right tension', right_initial, max_tension)

        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setFrameShadow(QFrame.Shadow.Sunken)

        zero_button = QPushButton('Zero both')
        zero_button.clicked.connect(self.zero)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(14)
        layout.addWidget(self.left)
        layout.addWidget(self.right)
        layout.addStretch()
        layout.addWidget(separator)
        layout.addWidget(zero_button)

    def zero(self):
        self.left.set_value(0.0)
        self.right.set_value(0.0)


class TensionPublisher(Node):

    def __init__(self):
        super().__init__('tension_slider_gui')
        # Read once at startup; set them from the launch file (use decimals, e.g. 50.0).
        self.max_tension = self.declare_parameter('max_tension', 50.0).value
        self.left_initial = self.declare_parameter('left_tension', 0.0).value
        self.right_initial = self.declare_parameter('right_tension', 0.0).value
        self.publish_rate = self.declare_parameter('publish_rate', 20.0).value
        self.publisher = self.create_publisher(RodTensions, 'rod_tensions', 10)

    def publish(self, left, right):
        msg = RodTensions()
        msg.left_tension = left
        msg.right_tension = right
        self.publisher.publish(msg)


def main():
    # Handle signals ourselves so Ctrl-C / launch shutdown closes the window cleanly.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TensionPublisher()

    app = QApplication(sys.argv)
    window = TensionSliderWindow(node.max_tension, node.left_initial, node.right_initial)

    # The publish timer also hands control back to Python regularly,
    # which is what lets the signal handlers below run while Qt is in its event loop.
    timer = QTimer()
    timer.timeout.connect(lambda: node.publish(window.left.value(), window.right.value()))
    timer.start(max(1, int(1000.0 / node.publish_rate)))

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())

    window.show()
    run = getattr(app, 'exec', None) or app.exec_  # Qt5/Qt6 compatible
    try:
        run()
    finally:
        timer.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()