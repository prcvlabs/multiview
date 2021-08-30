
import sys
import pdb
import math
import os
import types
import copy

import numpy as np

import PyQt5.QtWidgets as QtWidgets

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from OpenGL.GLU import gluPerspective, gluLookAt

import OpenGL.GL as gl

# Our module level imports
sys.path.append('./../..' if os.path.dirname(__file__) == '' \
                else os.path.dirname(__file__) + '/../..' )
from multiview import *


from .gl_drawing import gl_draw_axis

class GLViewer(QOpenGLWidget):

    xRotChanged = pyqtSignal(int)
    yRotChanged = pyqtSignal(int)
    zRotChanged = pyqtSignal(int)
    zoomChanged = pyqtSignal()
    oneRevolutionCaptured = pyqtSignal()
    
    # -------------------------------------------------------------- Constructor

    def reset_pan_zoom(self):
        self._is_rotate = None
        
        self._xRot = 0
        self._yRot = 0
        self._zRot = 0
        self._lastPos = QPoint()

        # Translate and scale object
        self._translate = Vector3(0, 0, 0)
        self._scale = Vector3(1, 1, 1)
        self._fov = 90.0
        self._znear = 0.5
        self._zfar = 5000.0
        self._eye = Vector3(0, 0, 4)
        self._center = Vector3(0, 0, 0)
        self._up = Vector3(0, 1, 0)
        self._et = None

        self._rot_theta = None
        self._delta = 0.0
        self._camera_speed = 64.0 # 1/Hz, i.e., seconds per complete revolution
        self._time0 = tick()
        self._last_delta = 0
        
    def __init__(self, parent = None):
        super(GLViewer, self).__init__(parent)
        self.setMouseTracking(True)
        
        self._is_init_gl = False

        self.reset_pan_zoom()
        
        self._did_capture = False
        self._capture_one_rev = False
        self._capture_frames = False
        self._capture_counter = 1
        
        self._draw_gl_axis = False
        self._draw_shape = False
        self._rotating = False
        self._helicopter_rotation_inclination = 0.4

        self._animation_timer = QTimer()
        self._animation_timer.setSingleShot(False)
        self._animation_timer.timeout.connect(self.updateGL)
        self._animation_timer.start(1000.0 / 33.0) # i.e., 33 frames per second
        self.set_rotating(False)
        
        self._initUI()

        self.beforeDrawThunk = None
        self.afterDrawThunk = None
        
    def _initUI(self):
        pass
    
    # ------------------------------------------------------------------ Qt Size

    def minimumSizeHint(self):
        return QSize(50, 50)

    def sizeHint(self):
        return QSize(400, 400)

    # ----------------------------------------------------------------- Rotation

    def _normalizeAngle(self, angle):
        while angle < 0:
            angle += 360 * 16
        while angle > 360 * 16:
            angle -= 360 * 16
        return angle
    
    def setXRot(self, angle):
        angle = self._normalizeAngle(angle)
        if angle != self._xRot:
            self._xRot = angle
            self.xRotChanged.emit(angle)
            self.update()

    def setYRot(self, angle):
        angle = self._normalizeAngle(angle)
        if angle != self._yRot:
            self._yRot = angle
            self.yRotChanged.emit(angle)
            self.update()

    def setZRot(self, angle):
        angle = self._normalizeAngle(angle)
        if angle != self._zRot:
            self._zRot = angle
            self.zRotChanged.emit(angle)
            self.update()
    
    def set_rotating(self, rotate):
        if rotate and not self.is_rotating():
            self._time0 = tick()
        self._rotating = rotate

    def set_rotate_delta(self, delta):
        self._delta = delta

    def set_up(self, x, y, z):
        self._up = Vector3(x, y, z).normalized()

    def get_up(self):
        return self._up    
        
    def is_rotating(self):
        return self._rotating

    def set_rotation_speed(self, secs_per_revolution):
        self._camera_speed = secs_per_revolution

    # Rotation spirals above target.
    # If 0.0, then rotation orbits the target.
    def set_helicopter__inclination(self, value):
        self._helicopter_rotation_inclination = value
    
    def set_rot_theta(self, theta):
        self._rot_theta = theta

    # --------------------------------------------------------------------- Zoom
    
    def zoom_in(self):
        self.set_zoom(self._scale.norm() * 1.1)

    def zoom_out(self):
        self.set_zoom(self._scale.norm() * 0.9)

    def get_zoom(self):
        return self._scale.norm()
        
    def set_zoom(self, scalev):
        min_scale = 0.001
        max_scale = 1000.0
        if scalev < min_scale:
            print("Scale {} less than minimum scale {}".format(scalev,
                                                               min_scale))
            scalev = min_scale
        if scalev > max_scale:
            print("Scale {} greater than maximum scale {}".format(scalev,
                                                                  max_scale))
        self._scale = scalev * self._scale.normalized()
        self.zoomChanged.emit()
        self.updateGL()

    # -------------------------------------------------------------------- Scale
    
    def set_scale(self, dx, dy, dz):
        self._scale = Vector3(dx, dy, dz)

    def get_scale(self):
        return self._scale    
    
    # ------------------------------------------------------------------ Capture

    def is_capturing(self):
        return self._capture_frames
    
    def set_capture(self, capture):
        if capture and not self._capture_frames:
            self._capture_counter = 1
            self._capture_one_rev = True
        self._capture_frames = capture

    def capture_one_revolution(self):
        self.set_rotating(True)
        self.set_capture(True)
        self._capture_one_rev = True

    # --------------------------------------------------------- Drawing Examples

    def set_draw_axis(self, draw):
        self._draw_gl_axis = draw

    def set_draw_shape(self, draw):
        self._draw_shape = draw

    # ---------------------------------------------------------- Camera Position

    # Set to None to go back to other system
    def set_fixed_cam(self, euclidean_transform):
        self._et = euclidean_transform
    
    def set_eye(self, x, y, z):
        self._eye = Vector3(x, y, z)

    def get_eye(self):
        return self._eye
        
    def set_center(self, x, y, z):
        self._center = Vector3(x, y, z)

    def get_center(self):
        return self._center
        
    def get_radius(self):
        return (self._eye - self._center).norm()

    def set_radius(self, r):
        self._eye = self._center + r * (self._eye - self._center).normalized()

    def calc_delta(self):
        ellapsed_s = tock(self._time0)
        self._last_delta = self._delta
        self._delta = math.fmod(ellapsed_s, self._camera_speed) \
                      / self._camera_speed
        if self._capture_one_rev and ellapsed_s >= self._camera_speed:
            self.set_capture(False)
            self.oneRevolutionCaptured.emit()
        return self._delta

    # ---------------------------------------------------------------- To String

    def to_string(self):
        print("Rot    = [{}, {}, {}]"
              .format(self._xRot, self._yRot, self._zRot))
        print("Centre = [{}, {}, {}]"
              .format(self.centre[0], self.centre[1], self.centre[2]))
        print("Eye    = [{}, {}, {}]"
              .format(self.eye[0], self.eye[1], self.eye[2]))
        print("Up     = [{}, {}, {}]"
              .format(self.up[0], self.up[1], self.up[2]))
        print("Scale  = [{}, {}, {}]"
              .format(self.scale[0], self.scale[1], self.scale[2]))
        print("Transl = [{}, {}, {}]"
              .format(self.translate[0],self.translate[1],self.translate[2]))
        print("\n");
    
    # ------------------------------------------------------------ OpenGL Events

    def updateGL(self):
        self.update()
    
    def initializeGL(self):        

        #f = QSurfaceFormat()             # The default
        #p = QOpenGLVersionProfile(f)

        # self._gl = Qt.QOpenGLContext.currentContext()
        try:
            pass
            #self._gl = self.context().versionFunctions(p)
            #self._gl.initializeOpenGLFunctions()
        except ModuleNotFoundError as e:
            print("opengl_vendor: Importing version functions "
                  "failed: {}".format(e))

        if not self._is_init_gl or True:
                            
            # gl = self._gl
            gl.glEnable(gl.GL_BLEND)
            gl.glEnable(gl.GL_POLYGON_SMOOTH)        
            gl.glEnable(gl.GL_DEPTH_TEST)
            gl.glEnable(gl.GL_CULL_FACE)
            gl.glEnable(gl.GL_NORMALIZE)
            gl.glEnable(gl.GL_LIGHT0)

            gl.glLightModeli(gl.GL_LIGHT_MODEL_TWO_SIDE, gl.GL_TRUE)
        
            gl.glHint(gl.GL_PERSPECTIVE_CORRECTION_HINT, gl.GL_NICEST)
            gl.glHint(gl.GL_POINT_SMOOTH_HINT,           gl.GL_NICEST)
            gl.glHint(gl.GL_LINE_SMOOTH_HINT,            gl.GL_NICEST)
            gl.glHint(gl.GL_POLYGON_SMOOTH_HINT,         gl.GL_NICEST)

            gl.glPolygonMode(gl.GL_FRONT_AND_BACK,       gl.GL_FILL)
        
            gl.glShadeModel(gl.GL_SMOOTH)
            gl.glClearColor(0.0, 0.0, 0.0, 0.0)        
            gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
            gl.glDepthFunc(gl.GL_LEQUAL)
            gl.glClearDepth(1.0)

            self._is_init_gl = True

        # return self._gl

    def paintGL(self):

        self.initializeGL()

        # Clear canvas
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT);
        gl.glLoadIdentity();

        # Locate the current eye position
        radius = self.get_radius() * self._scale.norm()
        center = self.get_center()
        at = (center - self.get_eye()).normalized()
        up = self.get_up().normalized()
        yaxis = at.cross(up)
        xaxis = up.cross(yaxis)
        delta = 0
        
        if self.is_rotating():
            delta = self.calc_delta()
        theta = self._rot_theta \
                if not self._rot_theta is None \
                   else 2.0 * (delta - 0.5) * math.pi
        
        if self._et is None:
            
            # The eye is the helicopter position
            eye = center \
                + xaxis * math.cos(theta) * radius \
                + yaxis * math.sin(theta) * radius \
                + radius * math.tan(self._helicopter_rotation_inclination) * up
        
            gluLookAt(eye[0]-center[0], eye[1]-center[1], eye[2]-center[2],
                      0.0, 0.0, 0.0,
                      up[0], up[1], up[2])

            gl.glRotated(float(self._xRot) / 16.0, 1.0, 0.0, 0.0)
            gl.glRotated(float(self._yRot) / 16.0, 0.0, 1.0, 0.0)
            gl.glRotated(float(self._zRot) / 16.0, 0.0, 0.0, 1.0)

            gl.glTranslated(-center[0], -center[1], -center[2])
            
        else:
            
            eye     = self._et.translation
            rot0    = self._et.rotation

            rotx    = Quaternion()
            roty    = Quaternion()
            rotz    = Quaternion()
            rotx.axis_angle = Vector4(1,0,0, to_radians(float(self._xRot)/16.0))
            roty.axis_angle = Vector4(0,1,0, to_radians(float(self._yRot)/16.0))
            rotz.axis_angle = Vector4(0,0,1, to_radians(float(self._zRot)/16.0))
            
            rot = rot0 * rotx * roty * rotz
            
            direction = rot.rotate(Vector3(0, 0,-1))
            up        = rot.rotate(Vector3(0,-1, 0))
            center    = (eye - direction)
            
            gluLookAt(eye[0], eye[1], eye[2],
                      center[0], center[1], center[2],
                      up[0], up[1], up[2])
            
        if not self.beforeDrawThunk is None:
            self.beforeDrawThunk()
        
        if self._draw_gl_axis:
            gl_draw_axis(gl)

        if not self.afterDrawThunk is None:
            self.afterDrawThunk()

        gl.glFlush()

        if self._capture_frames or self._rotating:
            counter = self._capture_counter
            gl_capture_buffer('/tmp/capture_{:04d}.png'.format(counter))
            self._capture_counter += 1
            
    def resizeGL(self, width, height):
        side = min(width, height)
        if side < 0:
            return
        
        self.initializeGL()
        gl.glViewport((width - side) // 2, (height - side) // 2, side, side)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        gluPerspective(self._fov, float(width) / float(height),
                       self._znear, self._zfar)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        
    def _emit_oneRevolutionCaptured(self):
        self.set_capture(False)
        self.set_rotating(False)
        self._capture_one_rev = False
        self.oneRevolutionCaptured.emit()

    # ------------------------------------------------------------- Mouse Events

    def mousePressEvent(self, event):
        self._lastPos = event.pos()
        kmods = QGuiApplication.keyboardModifiers()

        shift_key = True if kmods & Qt.ShiftModifier else False
        ctrl_key = True if kmods & Qt.ControlModifier else False
        alt_key = True if kmods & Qt.AltModifier else False

        self._is_rotate = not shift_key and not ctrl_key and not alt_key
        
    def mouseMoveEvent(self, event):

        if self._is_rotate is None:
            return
        
        dx = event.x() - self._lastPos.x()
        dy = event.y() - self._lastPos.y()

        if self._is_rotate:
            if event.buttons() & Qt.LeftButton:
                self.setXRot(self._xRot + 8 * dy)
                self.setYRot(self._yRot + 8 * dx)
            elif event.buttons() & Qt.RightButton:
                self.setXRot(self._xRot + 8 * dy)
                self.setZRot(self._zRot + 8 * dx)
        else:
            # self._center = self._center + Vector3(0.1, 0, 0)
            pass
                
        self._lastPos = event.pos()

    def mouseReleaseEvent(self, event):
        self._is_rotate = None

    def wheelEvent(self, event):
        d_angle = event.angleDelta()
        if d_angle.y() < 0:
            self.zoom_in()
        if d_angle.y() > 0:
            self.zoom_out()

class GLViewerTestingWindow(QWidget):
    
    def __init__(self):
        super(GLViewerTestingWindow, self).__init__()

        self.glWidget = GLViewer()
        self.glWidget.set_draw_axis(True)
        self.glWidget.set_draw_shape(True)
        self.glWidget.set_rotating(True)
        self.glWidget.set_radius(7.0)
        
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(self.glWidget)
        self.setLayout(mainLayout)

        self.setWindowTitle("Hello GL")
    
if __name__ == '__main__':

    app = QApplication(sys.argv)
    window = GLViewerTestingWindow()
    window.show()
    sys.exit(app.exec_())
    
