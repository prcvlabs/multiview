
import sys
import pdb
import math
import os
import types
import copy

import numpy as np

from multiview import *

def gl_vertex3(gl, x):
    gl.glVertex3d(x[0], x[1], x[2])

def gl_draw_axis(gl, line_len = 2.0, line_width = 4.0):
    axis_origin = Vector3(0, 0, 0)
    gl.glPushMatrix()
    gl.glScaled(line_len, line_len, line_len)
    gl.glLineWidth(line_width)
    gl.glBegin(gl.GL_LINES)
    gl.glColor3d(1, 0, 0)
    gl_vertex3(gl, axis_origin)
    gl_vertex3(gl, Vector3(1.0, 0.0, 0.0))
    gl.glColor3d(0, 1, 0)
    gl_vertex3(gl, axis_origin)
    gl_vertex3(gl, axis_origin + Vector3(0.0, 1.0, 0.0))
    gl.glColor3d(0, 0, 1)
    gl_vertex3(gl, axis_origin)
    gl_vertex3(gl, axis_origin + Vector3(0.0, 0.0, 1.0))
    gl.glEnd()
    gl.glLineWidth(1.0)
    gl.glPopMatrix()
    
