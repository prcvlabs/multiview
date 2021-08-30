
import sys
import pdb
import math
import cv2

import numpy as np

from PyQt5.QtWidgets import QWidget, QApplication, QSizePolicy
from PyQt5.QtGui import QPainter, QImage
from PyQt5.QtCore import Qt, QRect

from .qt_helpers import *

class ImageViewer(QWidget):
    
    # ------------------------------------------------------------- Construction
    
    def __init__(self, parent = None):

        super().__init__(parent)

        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Public members        
        self.zoom = 1.0
        self.offset = np.array([0, 0], np.int)
        self.is_pannable = True

        # Callbacks
        self.on_mouse_press = None
        self.post_paint_func = None

        # Private
        self.mouse_is_panning_ = False
        self.mouse_down_pos_ = np.array([0, 0], np.int)
        self.qim_ = None

    # --------------------------------------------------------------------------

    def set_image(self, qim):

        if isinstance(qim, (np.ndarray, np.generic)):
            self.qim_ = opencv_to_qimage(qim)
        else:
            self.qim_ = qim
        self.update()

    def reset(self):

        self.zoom = 1.0
        self.offset = np.array([0, 0], np.int)
        self.update()

    def mouse_to_image_coord(self, x):

        return (np.array(x, np.int) - self.offset) / self.zoom

    # -------------------------------------------------------- Private Utilities
    
    def start_pan_(self, pos):

        self.mouse_down_pos_ = np.array(pos, np.int32) - self.offset
        self.setMouseTracking(True)
        self.mouse_is_panning_ = True

    def stop_pan_(self):

        self.setMouseTracking(False)
        self.mouse_is_panning_ = False
        self.update();

    # -------------------------------------------------------------- Mouse Press

    def mousePressEvent(self, event):

        if not self.on_mouse_press is None:
            if self.on_mouse_press(event):
                return

        if(self.qim_ is None):
            return # Nothing to do

        is_left_btn = bool(event.button() & Qt.LeftButton) 
        shift_down = bool(event.modifiers() & Qt.ShiftModifier)
        ctrl_down = bool(event.modifiers() & Qt.ControlModifier)
        alt_down = bool(event.modifiers() & Qt.AltModifier)
        meta_down = bool(event.modifiers() & Qt.MetaModifier)
        has_modifier = shift_down or ctrl_down or alt_down or meta_down
        
        if is_left_btn and not has_modifier and self.is_pannable:
            self.start_pan_([event.x(), event.y()])

    # --------------------------------------------------------------- Mouse Move

    def mouseMoveEvent(self, event):

        if not self.mouse_is_panning_:
            return # Should never happen

        shift_down = bool(event.modifiers() & Qt.ShiftModifier)
        ctrl_down = bool(event.modifiers() & Qt.ControlModifier)
        alt_down = bool(event.modifiers() & Qt.AltModifier)
        meta_down = bool(event.modifiers() & Qt.MetaModifier)
        has_modifier = shift_down or ctrl_down or alt_down or meta_down

        if has_modifier:
            self.stop_pan_()
        else:
            event_pos = np.array([event.x(), event.y()], np.int)
            self.offset = event_pos - self.mouse_down_pos_

        self.update()

    # ------------------------------------------------------------ Mouse Release
            
    def mouseReleaseEvent(self, event):

        if self.mouse_is_panning_:
            self.stop_pan_()

    # -------------------------------------------------------------- Wheel Event

    def wheelEvent(self, event):

        # Make sure no painting is going on
        self.stop_pan_()
        
        ctrl_down = bool(event.modifiers() & Qt.ControlModifier)
        delta = event.angleDelta().y()
        scale = 0.05 / 120.0
        min_zoom = 0.1
        max_zoom = 10.0
        old_zoom = self.zoom
        old_pos = self.mouse_to_image_coord([event.x(), event.y()])

        new_zoom = old_zoom * (1.0 + delta * scale)
        self.zoom = np.clip(new_zoom, min_zoom, max_zoom)
        
        if old_zoom != self.zoom:
            off_x = event.x() - self.zoom * old_pos[0] + 0.49
            off_y = event.y() - self.zoom * old_pos[1] + 0.49
            self.offset = np.array([off_x, off_y], np.int)
            self.update()

    # -------------------------------------------------------------- Paint Event

    def paintEvent(self, event):
        
        painter = QPainter()
        painter.begin(self)

        if self.qim_ is None:
            painter.eraseRect(event.rect());
        else:
          
            total_w = self.qim_.width()
            total_h = self.qim_.height()

            target_rect = QRect(self.offset[0], self.offset[1],
                                math.ceil(total_w * self.zoom),
                                math.ceil(total_h * self.zoom))

            painter.drawImage(target_rect, self.qim_, self.qim_.rect())

        if not self.post_paint_func is None:
            self.post_paint_func(event, painter)
        
        painter.end()
        
