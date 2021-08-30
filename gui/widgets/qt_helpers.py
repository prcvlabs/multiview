
import sys
import pdb
import math
import cv2

import numpy as np

from PyQt5.QtWidgets import QWidget, QApplication, QSizePolicy
from PyQt5.QtGui import QPainter, QImage
from PyQt5.QtCore import Qt, QRect

# --- This is kept for reference
def import_pdb_set_trace():
    '''Set a tracepoint in PDB that works with Qt'''
    from PyQt5.QtCore import pyqtRemoveInputHook
    pyqtRemoveInputHook()

    import pdb; pdb.set_trace()

# ------------------------------------------------------------- OpenCV to QImage

def opencv_to_qimage(image):

    bpp = 1
    h, w = 0, 0
    stride = 0
    
    if len(image.shape) == 2:
        h, w = image.shape
        stride = image.strides[0]
    else:
        h, w, bpp = image.shape
        stride = image.strides[0]

    # Color
    if bpp == 3:
        rgb = image.copy()
        cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB, rgb)
        stride = bpp * w
        return QImage(rgb.data, w, h, stride, QImage.Format_RGB888)

    # Greyscale
    grey = image.copy()
    return QImage(grey.data, w, h, stride, QImage.Format_Grayscale8)
    
    raise RuntimeError("Not implemented yet.")

