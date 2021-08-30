
import sys
import pdb
import math
import os
import types
import copy

import numpy as np

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

# Must be on path
import multiview as mv

from .labeled_slider import LabeledSlider

class QRelDial(QDial):

    onStartDial = pyqtSignal()
    onEndDial = pyqtSignal()

    def __init__(self, parent = None):
        super().__init__(parent)
    
    def mousePressEvent(self, event):
        self.onStartDial.emit()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self.onEndDial.emit()
        super().mouseReleaseEvent(event)
    
class LabeledDial(QWidget):

    valueChanged = pyqtSignal(float)
    onStartDial = pyqtSignal()
    onEndDial = pyqtSignal()

    def __init__(self,
                 min_val = -math.pi,
                 max_val =  math.pi,
                 steps = 360 * 10,
                 parent = None):

        super().__init__(parent)

        # Members
        self.min_val = float(min_val)
        self.max_val = float(max_val)
        self.fmt = '{0:8.2f}'
        self.value0 = 0.0
        
        # Set up widgets
        self.label = QLabel()
        self.label.setFixedWidth(60)

        self.label = QLabel()
        self.label.setFixedWidth(60)
        
        self.dial = QRelDial()
        self.dial.setWrapping(True)
        self.dial.setOrientation(Qt.Horizontal)
        self.dial.setMinimum(0)
        self.dial.setMaximum(steps - 1)
        self.set_number_steps(steps)

        # Initialization
        self.set_value(0.0)
        self.update_text()

        # Layout
        vbox = QVBoxLayout()
        vbox.addWidget(self.dial)
        vbox.addWidget(self.label)
        self.setLayout(vbox)

        # Wiring
        self.dial.onStartDial.connect(self.on_start_dial_)
        self.dial.onEndDial.connect(self.on_end_dial_)
        self.dial.valueChanged.connect(self.value_changed_)

    def on_start_dial_(self):
        self.value0 = self.value()
        self.onStartDial.emit()

    def on_end_dial_(self):
        self.onEndDial.emit()

    def set_number_steps(self, value):
        step = (self.max_val - self.min_val) / (int(value) - 1)
        self.dial.setSingleStep(step)
        
    def set_value(self, value):
        """Update the dial with 'value', setting the label as well. Value 
        should be between 'self.min_val' and 'self.max_val'. (It is clamped.) 
        The value is printed to the label using format 'self.fmt'."""
        percent = mv.to_percent(value, self.min_val, self.max_val)
        s_value = percent * self.dial_precision_() + self.dial.minimum()
        old_state = self.dial.blockSignals(True)
        self.value0 = value
        self.dial.setValue(s_value)
        self.dial.blockSignals(old_state)
        self.label.setText(self.fmt.format(value))

    def value(self):
        """Read the value off the dial."""
        value = self.dial.value()
        dial_min = self.dial.minimum()
        dial_max = self.dial.maximum()
        percent = mv.to_percent(value, dial_min, dial_max)
        return percent * (self.max_val - self.min_val) + self.min_val

    def update_text(self):
        r_value = self.value()
        self.label.setText(self.fmt.format(r_value * 180.0 / math.pi))
        
    def value_changed_(self, value):
        """(Private) sets the label text, and emits the dial value."""
        self.update_text()
        v0 = self.value0
        v = self.value()
        min_v = self.min_val
        rang = self.max_val - self.min_val
        r_value = (min_v + (v - v0)) % rang - min_v
        self.valueChanged.emit(r_value)
        
    def dial_precision_(self):
        """Returns the number of positions on the dial"""
        return self.dial.maximum() - self.dial.minimum() + 1

class ThreeDControls(QWidget):

    valueChanged = pyqtSignal()
    
    def __init__(self, label = '', parent=None):
        
        super().__init__(parent)

        # Members
        self.position  = mv.Vector3(0, 0, 0)
        self.rots      = mv.Vector3(0, 0, 0)
        self.scale     = 1.0
        self.rotation  = mv.Quaternion()
        self.rotation0 = mv.Quaternion()
        self.euclidean_transform = mv.EuclideanTransform()
        
        self.min_dist  = -10.0
        self.max_dist  =  10.0

        self.posX_wgt  = LabeledSlider(self.min_dist, self.max_dist, 1000)
        self.posY_wgt  = LabeledSlider(self.min_dist, self.max_dist, 1000)
        self.posZ_wgt  = LabeledSlider(self.min_dist, self.max_dist, 1000)
        self.rotX_wgt  = LabeledDial()
        self.rotY_wgt  = LabeledDial()
        self.rotZ_wgt  = LabeledDial()
        self.scale_wgt = LabeledSlider(0.5, 2.0, 1000)
        self.reset_wgt = QPushButton('Reset')
        
        # Layout
        hbox = QHBoxLayout()
        hbox.addWidget(self.rotX_wgt)
        hbox.addWidget(self.rotY_wgt)
        hbox.addWidget(self.rotZ_wgt)
        rot_wgt = QWidget()
        rot_wgt.setLayout(hbox)
        
        layout = QFormLayout()
        layout.setVerticalSpacing(0)
        layout.addWidget(QLabel(label))
        layout.addRow('pos-x',               self.posX_wgt)
        layout.addRow('pos-y',               self.posY_wgt)
        layout.addRow('pos-z',               self.posZ_wgt)
        layout.addRow('rot',                 rot_wgt)
        layout.addRow('scale',               self.scale_wgt)
        layout.addRow('',                    self.reset_wgt)        
        self.setLayout(layout)

        # Initialization
        self.model_to_widgets()
        
        # Wiring
        self.rotX_wgt  .valueChanged.connect(self.rotationX_changed_)
        self.rotY_wgt  .valueChanged.connect(self.rotationY_changed_)
        self.rotZ_wgt  .valueChanged.connect(self.rotationZ_changed_)

        self.rotX_wgt  .onStartDial .connect(self.on_start_rot_)
        self.rotY_wgt  .onStartDial .connect(self.on_start_rot_)
        self.rotZ_wgt  .onStartDial .connect(self.on_start_rot_)
        self.rotX_wgt  .onEndDial   .connect(self.on_end_rot_)
        self.rotY_wgt  .onEndDial   .connect(self.on_end_rot_)
        self.rotZ_wgt  .onEndDial   .connect(self.on_end_rot_)
        
        self.posX_wgt  .valueChanged.connect(self.value_changed_)
        self.posY_wgt  .valueChanged.connect(self.value_changed_)
        self.posZ_wgt  .valueChanged.connect(self.value_changed_)
        self.scale_wgt .valueChanged.connect(self.value_changed_)
        self.reset_wgt .pressed     .connect(self.reset_pressed_)

    def on_start_rot_(self):
        self.rotation0 = mv.Quaternion(self.rotation)

    def on_end_rot_(self):
        self.rotation0 = mv.Quaternion(self.rotation)

    def on_rotation_change_(self, aa):
        axis = self.rotation0.rotate(mv.Vector3(aa.x, aa.y, aa.z))
        q = mv.Quaternion()
        q.axis_angle = mv.Vector4(axis.x, axis.y, axis.z, aa.w)
        self.rotation = q * self.rotation0
        self.valueChanged.emit()
        
    def rotationX_changed_(self, value):
        self.on_rotation_change_(mv.Vector4(1, 0, 0, value))
        
    def rotationY_changed_(self, value):
        self.on_rotation_change_(mv.Vector4(0, 1, 0, value))
        
    def rotationZ_changed_(self, value):
        self.on_rotation_change_(mv.Vector4(0, 0, 1, value))
        
    def value_changed_(self, value):
        """(Private) sets the label text, and emits the slider value."""
        self.widgets_to_model()
        self.valueChanged.emit()

    def reset_pressed_(self):
        self.position = mv.Vector3(0, 0, 0)
        self.rots     = mv.Vector3(0, 0, 0)
        self.scale    = 1.0
        self.euclidean_transform = mv.EuclideanTransform()
        self.rotX_wgt.set_value(0)
        self.rotY_wgt.set_value(0)
        self.rotZ_wgt.set_value(0)
        self.rotation  = mv.Quaternion()
        self.rotation0 = mv.Quaternion()
        self.model_to_widgets()

    def set_euclidean_transform(self, et):
        self.position = et.translation
        self.scale    = et.scale
        self.rotation = et.rotation
        self.euclidean_transform = et
        self.model_to_widgets()
        
    def set_position(self, vec3):        
        self.position = vec3
        self.model_to_widgets()

    def set_scale(self, value):        
        self.scale = value
        self.model_to_widgets()

    def widgets_to_model(self):
        
        self.position.x = float(self.posX_wgt.value())
        self.position.y = float(self.posY_wgt.value())
        self.position.z = float(self.posZ_wgt.value())
        self.scale      = float(self.scale_wgt.value())

        self.euclidean_transform.translation = self.position
        self.euclidean_transform.rotation    = self.rotation
        self.euclidean_transform.scale       = self.scale
        
    def model_to_widgets(self):

        # self.quaternion should already be set
        
        self.posX_wgt.set_value(self.position.x)
        self.posY_wgt.set_value(self.position.y)
        self.posZ_wgt.set_value(self.position.z)

        self.scale_wgt.set_value(self.scale)
    
