
import sys
import pdb
import math
import os
import types

import numpy as np

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

# Must be on path
from multiview import to_percent

class LabeledSlider(QWidget):

    valueChanged = pyqtSignal(float)
    
    def __init__(self,
                 min_val = 0.0,
                 max_val = 100.0,
                 steps = 1001,
                 parent=None):
        
        super().__init__(parent)

        # Members
        self.min_val = float(min_val)
        self.max_val = float(max_val)
        self.fmt = '{0:8.2f}'

        # Set up widgets
        self.label = QLabel()
        self.label.setFixedWidth(60)
        
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setTickPosition(QSlider.NoTicks)
        self.slider.setMinimum(0)
        self.slider.setMaximum(steps - 1)
        self.set_number_steps(steps)
        
        self.label.setText(self.fmt.format(self.min_val))

        # Layout
        hbox = QHBoxLayout()
        hbox.addWidget(self.slider)
        hbox.addWidget(self.label)
        self.setLayout(hbox)

        # Wiring
        self.slider.valueChanged.connect(self.value_changed_)

    def set_number_steps(self, value):
        self.slider.setMaximum(int(value) - 1)
        
    def set_value(self, value):
        """Update the slider with 'value', setting the label as well. Value 
        should be between 'self.min_val' and 'self.max_val'. (It is clamped.) 
        The value is printed to the label using format 'self.fmt'."""
        percent = to_percent(value, self.min_val, self.max_val)
        s_value = percent * self.slider_precision_() + self.slider.minimum()
        old_state = self.slider.blockSignals(True)
        self.slider.setValue(s_value)
        self.slider.blockSignals(old_state)
        self.label.setText(self.fmt.format(value))

    def value(self):
        """Read the value off the slider."""
        value = self.slider.value()
        slider_min = self.slider.minimum()
        slider_max = self.slider.maximum()
        percent = to_percent(value, slider_min, slider_max)
        return percent * (self.max_val - self.min_val) + self.min_val

    def value_changed_(self, value):
        """(Private) sets the label text, and emits the slider value."""
        r_value = self.value()
        self.label.setText(self.fmt.format(r_value))
        self.valueChanged.emit(r_value)
        
    def slider_precision_(self):
        """Returns the number of positions on the slider"""
        return self.slider.maximum() - self.slider.minimum() + 1
        
