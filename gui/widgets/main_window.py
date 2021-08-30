
import sys
import errno
import sys
import os
import cv2
import json
import pdb
import argparse
import getpass
import datetime
import types
import copy

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

# pyrcc5 res.rcc > ../resources.py
from . import resources
from .image_viewer import ImageViewer
from .qt_helpers import *
from .tweaker_cpanel import TweakerCPanel
from .gl_viewer import GLViewer

from multiview import *
from widgets.pblock import PBlock

APPLICATION_NAME = 'Camera Tweaker'
CPANEL_WIDTH = 400
SLIC_SCALE = 4

def setting_defaults():
    return { 'geometry': [20, 40, 1000, 800], # These are the default values
             'cpanel_width': CPANEL_WIDTH,
             'viewer_offset': [0, 0],
             'viewer_zoom': 1.0 }

class MainWindow(QMainWindow):

    # -------------------------------------------------------------- Constructor

    def __init__(self, argz):
        super().__init__()
        saved_state = self.read_save_state_(argz.gui_defaults)
        settings = types.SimpleNamespace(**saved_state)        
        self.pblock = PBlock(argz, settings)
        self.initUI()        
        self.pblock_reload()

        if hasattr(settings, 'misc_params'):
            self.load_misc_params(getattr(settings, 'misc_params'))
        
    # ------------------------------------------------------ Save/Load App State

    def settings_key_(self):
        return  APPLICATION_NAME.replace(' ', '_') + '_SETTINGS'
    
    def read_save_state_(self, use_gui_defaults):
        settings = QSettings()
        ret = setting_defaults()
        if settings.contains(self.settings_key_()) and not use_gui_defaults:
            dat = json.loads(settings.value(self.settings_key_()))
            for key, value in dat.items():
                ret[key] = value            
        return ret        
    
    def save_state_(self):
        settings = self.read_save_state_(self.pblock.argz.gui_defaults)
        if not self.isMaximized() and not self.isFullScreen() \
           and not self.isMinimized() and self.isVisible():
            print_info('Save GUI state')
            rect = self.geometry()
            settings['geometry'][0] = rect.x()
            settings['geometry'][1] = rect.x()
            settings['geometry'][2] = rect.width()
            settings['geometry'][3] = rect.height()
            settings['cpanel_width'] = CPANEL_WIDTH # self.cpanel.width()
            settings['viewer_offset'] = self.viewer.offset.tolist()
            if self.isShowSlic():
                settings['viewer_zoom'] = self.viewer.zoom * SLIC_SCALE
            else:
                settings['viewer_zoom'] = self.viewer.zoom
                
            # Write out OpenGL display parameters
            settings['opengl_render_opts'] \
                = self.pblock.gl_render_opts.write_to_string()
                
            # Write out tweaker parameters
            for i in range(0, self.pblock.models.size()):
                params = self.pblock.models[i]
                settings['tweaker_{}'.format(i)] = params.write_to_string()

            # Write out misc. parameters
            settings['misc_params'] = self.collate_misc_params()
                
            QSettings().setValue(self.settings_key_(), json.dumps(settings))

    def collate_misc_params(self):
        p = {}
        p['cb_left_image'] = self.cpanel.cb_left_image.isChecked()
        p['cam_index']     = self.cpanel.selected_camera()
        if self.cpanel.rb_raw_video.isChecked():
            p['rb_display'] = 'rb_raw_video'
        if self.cpanel.rb_disparity_map.isChecked():
            p['rb_display'] = 'rb_disp'
        if self.cpanel.rb_undistorted.isChecked():
            p['rb_display'] = 'rb_undist'
        if self.cpanel.rb_scharr.isChecked():
            p['rb_display'] = 'rb_scharr'
        if self.cpanel.rb_scharr_thresh.isChecked():
            p['rb_display'] = 'rb_scharrt'
        if self.cpanel.rb_harris.isChecked():
            p['rb_display'] = 'rb_harris'
        if self.cpanel.rb_slic.isChecked():
            p['rb_display'] = 'rb_slic'
        if self.cpanel.rb_planes.isChecked():
            p['rb_display'] = 'rb_planes'
        if self.cpanel.rb_hist.isChecked():
            p['rb_display'] = 'rb_hist'
        if self.cpanel.rb_point_cloud.isChecked():
            p['rb_display'] = 'rb_ptcloud'
        p['gl_simt_index'] = self.cpanel.gl_select_cam.currentIndex()
        return p

    def load_misc_params(self, p):
        if 'cb_left_image' in p:
            self.cpanel.cb_left_image.setChecked(p['cb_left_image'])
        if 'cam_index' in p:
            cam_num = p['cam_index']
            n_cams = self.pblock.argz.scene_desc.n_cameras()
            if cam_num >= 0 and cam_num < n_cams:
                val = self.cpanel.select_image.blockSignals(True)
                self.cpanel.select_image.setCurrentIndex(cam_num)
                self.cpanel.select_image.blockSignals(val)
        if 'rb_display' in p:
            val = p['rb_display']
            if val == 'rb_raw_video': self.cpanel.rb_raw_video.setChecked(True)
            if val == 'rb_disp':   self.cpanel.rb_disparity_map.setChecked(True)
            if val == 'rb_undist':  self.cpanel.rb_undistorted.setChecked(True)
            if val == 'rb_scharr':  self.cpanel.rb_scharr.setChecked(True)
            if val == 'rb_scharrt':self.cpanel.rb_scharr_thresh.setChecked(True)
            if val == 'rb_harris':  self.cpanel.rb_harris.setChecked(True)
            if val == 'rb_slic':    self.cpanel.rb_slic.setChecked(True)
            if val == 'rb_planes':  self.cpanel.rb_planes.setChecked(True)
            if val == 'rb_hist':    self.cpanel.rb_hist.setChecked(True)
            if val == 'rb_ptcloud': self.cpanel.rb_point_cloud.setChecked(True)

        if 'gl_simt_index' in p:
            ind = p['gl_simt_index']
            if ind >= 0 and ind < self.cpanel.gl_select_cam.count():
                val = self.cpanel.gl_select_cam.blockSignals(True)
                self.cpanel.gl_select_cam.setCurrentIndex(ind)
                self.cpanel.gl_select_cam.blockSignals(val)                
            
    # ------------------------------------------------------------------ Init UI
    
    def initUI(self):
        
        # ---------------------------------- #
        # --            Widgets           -- #
        # ---------------------------------- #
        # Statusbar
        self.statusbar = self.statusBar()
        self.statusbar.showMessage('Ready')

        # Control Panel
        self.cpanel = TweakerCPanel(self.pblock, self)
        self.last_slic = self.cpanel.rb_slic.isChecked()
        
        # Video
        self.viewer = ImageViewer(self)
        self.viewer.set_image(None)
        self.viewer.zoom = self.pblock.app_settings.viewer_zoom
        self.viewer.offset = np.array(self.pblock.app_settings.viewer_offset,
                                      np.int)
        self.viewer.on_mouse_press = self.on_viewer_click
        self.video_slider = QSlider(Qt.Horizontal)
        self.video_slider.setFocusPolicy(Qt.StrongFocus)
        self.video_slider.setTickPosition(QSlider.NoTicks)
        self.video_slider.setTickInterval(10)
        self.video_slider.setSingleStep(1)
        self.video_slider.setMinimum(0)
        self.video_slider.setMaximum(self.pblock.argz.scene_desc.n_frames() - 1)

        # OpenGL viewer
        self.gl_viewer = GLViewer()
        self.gl_viewer.beforeDrawThunk = self.render_gl
        
        # # Create a tooltip
        # QToolTip.setFont(QFont('SansSerif', 10))        
        # self.setToolTip('This is a <b>QWidget</b> widget')

        # # Create a pushbutton
        # btn = QPushButton('Button')
        # btn.setToolTip('This is a <b>QPushButton</b> widget')
        # #btn.resize(btn.sizeHint())
        # btn.clicked.connect(self.close)
        
        # ---------------------------------- #
        # --            Actions           -- #
        # ---------------------------------- #
        # Exit action
        self.exitAct_ = QAction(QIcon(':Icons/quit.png'), '&Exit', self)        
        self.exitAct_.setShortcut('Ctrl+Q')
        self.exitAct_.setStatusTip('Exit application')
        self.exitAct_.triggered.connect(qApp.quit)

        # View status-bar action
        self.viewStatAct_ = QAction('Hide statusbar', self)
        self.viewStatAct_.setStatusTip('Hide statusbar')
        self.viewStatAct_.triggered.connect(self.toggleStatusBarVisibility)

        # Reset offset-zoom action
        self.resetOffsetZoom_ = QAction('Reset pan/zoom', self)
        self.resetOffsetZoom_.setStatusTip('Reset video pan and zoom')
        self.resetOffsetZoom_.triggered.connect(self.reset_viewer_gl_viewer)

        # Rerun tweaker
        self.rerunTweakerAct_ = QAction('Rerun tweaker', self)
        self.rerunTweakerAct_.setStatusTip('Rerun tweaker execution')
        self.rerunTweakerAct_.setShortcut('F5')
        self.rerunTweakerAct_.triggered.connect(self.rerun_tweaker_)

        # Select camera action
        def make_camera_select_action(index, slot):
            action = QAction('Select Camera {}'.format(index), self)
            action.setStatusTip('Select Camera {}'.format(index))
            action.setShortcut('Ctrl+{}'.format(index+1 if index < 9 else 0))
            action.triggered.connect(slot)
            return action
            
        self.selectCam0Act_ = make_camera_select_action(0, self.select_cam_0_)
        self.selectCam1Act_ = make_camera_select_action(1, self.select_cam_1_)
        self.selectCam2Act_ = make_camera_select_action(2, self.select_cam_2_)
        self.selectCam3Act_ = make_camera_select_action(3, self.select_cam_3_)
        self.selectCam4Act_ = make_camera_select_action(4, self.select_cam_4_)
        self.selectCam5Act_ = make_camera_select_action(5, self.select_cam_5_)
        self.selectCam6Act_ = make_camera_select_action(6, self.select_cam_6_)
        self.selectCam7Act_ = make_camera_select_action(7, self.select_cam_7_)
        self.selectCam8Act_ = make_camera_select_action(8, self.select_cam_8_)
        self.selectCam9Act_ = make_camera_select_action(9, self.select_cam_9_)

        # Select camera action
        def make_sen_select_action(index, slot):            
            aruco_sensor_ids = self.pblock.argz.scene_desc.sensor_ids
            
            sensor_text = 'Clear selected sensor'
            if index < aruco_sensor_ids.size():
                sensor_text = 'Select {}'.format(aruco_sensor_ids[index])
                
            action = QAction(sensor_text, self)
            action.setStatusTip(sensor_text)
            action.setShortcut('Alt+{}'.format(index+1 if index < 9 else 0))
            action.triggered.connect(slot)
            return action
        
        self.selectSensor0Act_ = make_sen_select_action(0,self.select_sensor_0_)
        self.selectSensor1Act_ = make_sen_select_action(1,self.select_sensor_1_)
        self.selectSensor2Act_ = make_sen_select_action(2,self.select_sensor_2_)
        self.selectSensor3Act_ = make_sen_select_action(3,self.select_sensor_3_)
        self.selectSensor4Act_ = make_sen_select_action(4,self.select_sensor_4_)
        self.selectSensor5Act_ = make_sen_select_action(5,self.select_sensor_5_)
        self.selectSensor6Act_ = make_sen_select_action(6,self.select_sensor_6_)
        self.selectSensor7Act_ = make_sen_select_action(7,self.select_sensor_7_)
        self.selectSensor8Act_ = make_sen_select_action(8,self.select_sensor_8_)
        self.selectSensor9Act_ = make_sen_select_action(9,self.select_sensor_9_)
        
        # ---------------------------------- #
        # --           Menu Bar           -- #
        # ---------------------------------- #
        # Menu bar. Note you can also do sub-menus.
        menubar = self.menuBar()
        self.fileMenu_ = menubar.addMenu('&File')
        self.fileMenu_.addAction(self.viewStatAct_)
        self.fileMenu_.addSeparator()
        self.fileMenu_.addAction(self.exitAct_)

        self.cameraMenu_ = menubar.addMenu('&Camera')
        self.cameraMenu_.addAction(self.selectCam0Act_)
        self.cameraMenu_.addAction(self.selectCam1Act_)
        self.cameraMenu_.addAction(self.selectCam2Act_)
        self.cameraMenu_.addAction(self.selectCam3Act_)
        self.cameraMenu_.addAction(self.selectCam4Act_)
        self.cameraMenu_.addAction(self.selectCam5Act_)
        self.cameraMenu_.addAction(self.selectCam6Act_)
        self.cameraMenu_.addAction(self.selectCam7Act_)
        self.cameraMenu_.addAction(self.selectCam8Act_)
        self.cameraMenu_.addAction(self.selectCam9Act_)

        self.sensorMenu_ = menubar.addMenu('&Sensor')
        self.sensorMenu_.addAction(self.selectSensor0Act_)
        self.sensorMenu_.addAction(self.selectSensor1Act_)
        self.sensorMenu_.addAction(self.selectSensor2Act_)
        self.sensorMenu_.addAction(self.selectSensor3Act_)
        self.sensorMenu_.addAction(self.selectSensor4Act_)
        self.sensorMenu_.addAction(self.selectSensor5Act_)
        self.sensorMenu_.addAction(self.selectSensor6Act_)
        self.sensorMenu_.addAction(self.selectSensor7Act_)
        self.sensorMenu_.addAction(self.selectSensor8Act_)
        self.sensorMenu_.addAction(self.selectSensor9Act_)

        self.actionMenu_ = menubar.addMenu('&Action')
        self.actionMenu_.addAction(self.resetOffsetZoom_)
        self.actionMenu_.addSeparator()
        self.actionMenu_.addAction(self.rerunTweakerAct_)
        
        # ---------------------------------- #
        # --            Wiring            -- #
        # ---------------------------------- #
        qApp.aboutToQuit.connect(self.save_state_)
        self.video_slider.valueChanged.connect(self.set_frame)
        self.pblock.resultsChangedMustConnectQueued.connect(self.update_results,
                                                            Qt.QueuedConnection)
        self.cpanel.changed.connect(self.render)
        self.cpanel.changed.connect(self.update_planes_selected)
        self.video_slider.valueChanged.connect(self.pblock.execute_threaded)
        self.cpanel.onPlanesSelect.connect(self.planes_select)
        self.cpanel.onPlanesTypeSelect.connect(self.planes_type_select)
        self.cpanel.onPlanesUpdateD.connect(self.planes_d_update)
        self.cpanel.onPlanesShow.connect(self.planes_show)
        self.cpanel.onPlanesAdd.connect(self.planes_add)
        self.cpanel.onPlanesDel.connect(self.planes_del)
        self.cpanel.onPlanesSave.connect(self.planes_save_selected)
        self.cpanel.onPlanesLoad.connect(self.planes_load_selected)
        
        # ---------------------------------- #
        # --            Layout            -- #
        # ---------------------------------- #        
        vid_widget = QWidget()
        vbox = QVBoxLayout()
        vbox.addWidget(self.viewer)
        vbox.addWidget(self.gl_viewer)
        vbox.addWidget(self.video_slider)
        vid_widget.setLayout(vbox)        

        vline = QFrame()
        vline.setFrameShape(QFrame.VLine)
        vline.setFrameShadow(QFrame.Sunken)

        hbox = QHBoxLayout()
        hbox.addWidget(self.cpanel)
        hbox.addWidget(vline)
        hbox.addWidget(vid_widget)
        
        # ss = QSplitter()
        # ss.addWidget(self.cpanel)
        # ss.addWidget(vline)
        # ss.addWidget(self.viewer)
        # hbox = QHBoxLayout()
        # hbox.addWidget(ss)

        main_widget = QWidget()
        main_widget.setLayout(hbox)
        main_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setCentralWidget(main_widget)

        rect = self.pblock.app_settings.geometry
        self.setGeometry(rect[0], rect[1], rect[2], rect[3])
        self.setWindowIcon(QIcon(':/Icons/logo_web.png'))
        self.setWindowTitle(APPLICATION_NAME)

        self.set_widget_visibility()
        
        self.show()

    # --------------------------------------------------------------- isShowSlic

    def isShowSlic(self):
        return self.cpanel.rb_slic.isChecked() or \
               self.cpanel.rb_planes.isChecked()
        
    # ------------------------------------------------------------------- Events

    def closeEvent(self, event):
        use_message_box = False
        if use_message_box:
            reply = QMessageBox.question(self, 'Message',
                                         "Are you sure to quit?",
                                         QMessageBox.Yes | QMessageBox.No,
                                         QMessageBox.No)

            if reply == QMessageBox.Yes:
                event.accept()
            else:
                event.ignore()
        else:
            self.save_state_()
            event.accept()
            
    def toggleStatusBarVisibility(self, state):
        if not self.statusbar.isVisible():
            self.statusbar.show()
            self.viewStatAct_.setText('Hide statusbar')
        else:
            self.statusbar.hide()
            self.viewStatAct_.setText('Show statusbar')

    def contextMenuEvent(self, event):
        # Right-click to open the context-menu       
        cmenu = QMenu(self)
           
        newAct = cmenu.addAction("New")
        opnAct = cmenu.addAction("Open")
        quitAct = cmenu.addAction("Quit")
        action = cmenu.exec_(self.mapToGlobal(event.pos()))
        
        if action == quitAct:
            qApp.quit()

    def on_viewer_click(self, event):
        show_slic = self.isShowSlic()
        is_planes = self.cpanel.rb_planes.isChecked()
        if not show_slic and not is_planes:
            return

        cam_ind = self.pblock.current_camera_ind()
        if cam_ind is None:
            return
        
        is_left = self.cpanel.cb_left_image.isChecked()
        img_ind = 0 if is_left else 1
        cp = self.pblock.current_calib_plane()
        
        kmods = QGuiApplication.keyboardModifiers()
        shift_key = True if kmods & Qt.ShiftModifier else False
        ctrl_key = True if kmods & Qt.ControlModifier else False
        alt_key = True if kmods & Qt.AltModifier else False
        
        if not shift_key and not ctrl_key:
            return

        # Load the currently selected
        selected = np.array([], np.int32)
        if is_planes:
            selected = cp.get_selected(is_left)
        
        if ctrl_key:
            selected = np.array([], np.int32)

        else:
            pos = self.viewer.mouse_to_image_coord([event.x(), event.y()])
            
            pos[0] /= SLIC_SCALE
            pos[1] /= SLIC_SCALE

            pos_vec = Vector2(pos[0], pos[1])
            
            # print_info("pos-vec: {}, cam_ind = {}, img_ind = {}".format(pos_vec, cam_ind, img_ind))
            
            slic_label = self.pblock.results.get_slic_label(cam_ind,
                                                            img_ind,
                                                            pos_vec)

            if slic_label in selected:
                l = list(selected)
                l.remove(slic_label)
                selected = np.array(l, np.int32)
            else:
                selected = np.array(list(selected) + [slic_label], np.int32)

        print_info("slic selected set: {0}".format(selected))
        
        # Update the current selected
        if is_planes:
            cp.set_selected(is_left, selected)

        self.render()

    # ---------------------------------------------------------- Reset Gl Viewer

    def reset_viewer_gl_viewer(self):
        self.viewer.reset()
        self.gl_viewer.reset_pan_zoom()
        
    # ---------------------------------------------- Reset Distortion Parameters

    def optimize_cam_positions_(self):
        self.cpanel.widgets_to_model()
        self.cpanel.model_to_widgets()
        self.pblock.execute_opt_cam_threaded()
        
    def rerun_tweaker_(self):
        self.cpanel.widgets_to_model()
        self.cpanel.model_to_widgets()
        self.pblock.execute_threaded()
        
    # ------------------------------------------------ Setting the current frame

    def set_frame(self, index):
        self.pblock.set_current_frame_no(index)
        if self.pblock.has_current_frame_no():
            n_frames = self.pblock.n_frames()
            msg = "Loaded frame number {0}/{1}".format(index + 1, n_frames)
            self.statusbar.showMessage(msg)
        self.render()

    def next_frame(self):
        if not self.pblock.has_current_video() is None:
            msg = "Cannot read next frame: no video file open"
            self.statusbar.showMessage(msg)
            return
        self.set_frame((self.pblock.current_frame_no()+1) \
                       % self.pblock.current_video().n_frames())

    def prev_frame(self):
        if not self.pblock.has_current_video() is None:
            msg = "Cannot read previous frame: no video file open"
            self.statusbar.showMessage(msg)
            return
        frame_no = self.pblock.current_frame_no() - 1
        if frame_no < 0:
            frame_no = self.pblock.current_video().n_frames() - 1
        self.set_frame(frame_no)

    # ------------------------------------------------------------ Select Camera

    def select_cam_0_(self):
        self.cpanel.update_selected_camera(0)
    def select_cam_1_(self):
        self.cpanel.update_selected_camera(1)
    def select_cam_2_(self):
        self.cpanel.update_selected_camera(2)
    def select_cam_3_(self):
        self.cpanel.update_selected_camera(3)
    def select_cam_4_(self):
        self.cpanel.update_selected_camera(4)
    def select_cam_5_(self):
        self.cpanel.update_selected_camera(5)
    def select_cam_6_(self):
        self.cpanel.update_selected_camera(6)
    def select_cam_7_(self):
        self.cpanel.update_selected_camera(7)
    def select_cam_8_(self):
        self.cpanel.update_selected_camera(8)
    def select_cam_9_(self):
        self.cpanel.update_selected_camera(9)

    # ------------------------------------------------------------ Select Sensor
    
    def select_sensor_0_(self):
        self.gl_viewer_set_sensor(0)
    def select_sensor_1_(self):
        self.gl_viewer_set_sensor(1)
    def select_sensor_2_(self):
        self.gl_viewer_set_sensor(2)
    def select_sensor_3_(self):
        self.gl_viewer_set_sensor(3)
    def select_sensor_4_(self):
        self.gl_viewer_set_sensor(4)
    def select_sensor_5_(self):
        self.gl_viewer_set_sensor(5)
    def select_sensor_6_(self):
        self.gl_viewer_set_sensor(6)
    def select_sensor_7_(self):
        self.gl_viewer_set_sensor(7)
    def select_sensor_8_(self):
        self.gl_viewer_set_sensor(8)
    def select_sensor_9_(self):
        self.gl_viewer_set_sensor(9)

    # ---------------------------------------------------- Set the current Video

    def pblock_reload(self):
        self.pblock.set_cpanel(self.cpanel)
        self.pblock.set_current_frame_no(0)        
        # self.set_current_camera(0)
        if(self.pblock.argz.start > 1):
            fnum = self.pblock.argz.start - 1
            self.set_frame(fnum)
            blocked = self.video_slider.blockSignals(True)
            self.video_slider.setValue(fnum)
            self.video_slider.blockSignals(blocked)
        self.pblock.execute_threaded()

    def set_current_camera(self, camera_id):
        self.pblock.set_current_camera(0)
        if not self.pblock.has_current_video():
            cam_name = self.pblock.current_camera_name()
            self.statusbar.showMessage("No video file loaded for camera {0}."
                                       "".format(cam_name))
        else:
            vid = self.pblock.current_video()
            vid_name = os.path.basename(vid.filename()) 
            cam_name = self.pblock.current_camera_name()
            self.statusbar.showMessage("Loaded video file '{0}' for camera {1}."
                                       ''.format(vid_name, cam_name))
            blocked = self.video_slider.blockSignals(True)
            self.video_slider.setMinimum(0)
            self.video_slider.setMaximum(vid.n_frames() - 1)
            self.video_slider.blockSignals(blocked)
            self.set_frame(self.pblock.current_frame_no())

    # ----------------------------------------------------------- Update Results

    def update_results(self):
        self.render()
            
    # ---------------------------------------------------- Set Widget Visibility

    def show_gl(self):
        return self.cpanel.rb_point_cloud.isChecked() \
            or self.cpanel.rb_slic_3d.isChecked()
    
    def set_widget_visibility(self):
        show_gl = self.show_gl()
        self.gl_viewer.setVisible(show_gl)
        self.viewer.setVisible(not show_gl)
        
    # ------------------------------------------------------------------- Render

    def render_to_qimage_viewer(self):
        """Responsible for generating a QImage, and setting it to the image 
        viewer"""

        # What are the show parameters?
        cam_num            = self.cpanel.selected_camera()
        sensor_ind         = 0 if self.cpanel.cb_left_image.isChecked() else 1
        show_gray          = False
        show_distorted     = self.cpanel.rb_raw_video.isChecked()
        show_disparity     = self.cpanel.rb_disparity_map.isChecked()
        show_undistorted   = self.cpanel.rb_undistorted.isChecked()
        show_scharr        = self.cpanel.rb_scharr.isChecked()
        show_scharr_thresh = self.cpanel.rb_scharr_thresh.isChecked()
        show_harris        = self.cpanel.rb_harris.isChecked()
        show_slic          = self.cpanel.rb_slic.isChecked()  
        show_hist          = self.cpanel.rb_hist.isChecked()
        show_planes        = self.cpanel.rb_planes.isChecked()
        
        # We can only show cam-no '1' when we have a nondual camera
        if not self.pblock.results.cam_sensor_good(cam_num, sensor_ind):
            self.viewer.set_image(None) # Not a dual image!
            return

        # Load image from results
        res = self.pblock.results
        
        if False:
            pass
        elif show_hist:
            self.viewer.set_image(res.floor_histogram_image())
        elif show_distorted:
            self.viewer.set_image(res.equalized(cam_num, sensor_ind))
        elif show_gray:
            self.viewer.set_image(res.gray(cam_num, sensor_ind))
        elif show_undistorted:
            self.viewer.set_image(res.undistorted(cam_num, sensor_ind))
        elif show_disparity:            
            ind = int(self.cpanel.disp_view_select_slider.value())
            if ind < TweakerResults.n_disparities():
                im = res.disparity_image(cam_num, ind)
                self.viewer.set_image(im)
        elif show_scharr:
            self.viewer.set_image(res.scharr_image(cam_num, sensor_ind))
        elif show_scharr_thresh:
            self.viewer.set_image(res.scharr_filter_image(cam_num, sensor_ind))
        elif show_harris:
            self.viewer.set_image(res.harris_image(cam_num, sensor_ind))
        elif show_slic:
            p = self.pblock.models[cam_num]
            self.viewer.set_image(res.slic_labeled_image(cam_num, sensor_ind,p))
        elif show_planes:
            cps = self.pblock.calib_plane_sets[cam_num]
            show_d = self.cpanel.plane_rb_show_d.isChecked()
            show_p3_res = self.cpanel.plane_rb_p3_result.isChecked()
            self.viewer.set_image(res.slic_label_with_calib(cam_num,
                                                            sensor_ind,
                                                            cps,
                                                            show_d,
                                                            show_p3_res))

    def gl_viewer_set_sensor(self, ind):
        
        aruco_sensor_ids = self.pblock.argz.scene_desc.sensor_ids
        aruco_transforms = self.pblock.argz.scene_desc.aruco_transforms()

        if ind < 0 or ind >= aruco_transforms.size():
            self.gl_viewer.set_fixed_cam(None)
            self.statusbar.showMessage('Cleared fixed sensor')
        else:
            self.gl_viewer.set_fixed_cam(aruco_transforms[ind])
            self.statusbar.showMessage(aruco_sensor_ids[ind])
            
    def render_gl(self):        
        
        if self.pblock.results:            

            is_slic = self.cpanel.rb_slic_3d.isChecked()
            
            self.cpanel.gl_widgets_to_opts()
            opts = self.pblock.gl_render_opts
            self.gl_viewer.set_rotating(self.pblock.gl_render_opts.do_rotate)

            opts.do_capture = self.pblock.gl_render_opts.do_rotate

            if is_slic:
                gl_render_slic_3d(self.pblock.results, opts)
            else:
                gl_render_point_cloud(self.pblock.results, opts)
            
        else:

            # There's no point cloud to display
            pass
                
    def render(self):
        # Set the visibility of the widgets
        self.set_widget_visibility()

        # Manage rescaling the image depending on whether slic is there or not
        # so that SLIC stays the same size
        is_slic = self.isShowSlic()
        if is_slic and not self.last_slic:
            self.viewer.zoom /= SLIC_SCALE
        elif not is_slic and self.last_slic:
            self.viewer.zoom *= SLIC_SCALE
        self.last_slic = is_slic
        
        if self.show_gl():
            pass # This should already be set
        else:
            self.render_to_qimage_viewer()

    # ------------------------------------------------ Load/Save Planes Selected
    def update_planes_selected(self):
        combo = self.cpanel.plane_combo
        # block signals
        blocked = combo.blockSignals(True)

        cps = self.pblock.current_plane_set()

        combo.clear()

        if not cps is None:
            sel_idx = cps.selected_index
            n = cps.p3s.size()
            index = combo.currentIndex()
            
            for i in range(1, n+1):
                combo.addItem('{}'.format(i))
            if sel_idx < 0 and sel_idx >= n:
                sel_idx = n - 1
            self.planes_select(sel_idx)

            if sel_idx >= 0:
                combo.setCurrentIndex(sel_idx)
        
        # unblock signals
        combo.blockSignals(blocked)
    
    def planes_select(self, value):
        cps = self.pblock.current_plane_set()
        if cps is None:
            return        
        cps.selected_index = value
        if cps.selected_index >= 0 and cps.selected_index < cps.p3s.size():
            cp = cps.p3s[cps.selected_index]
            blocked = self.cpanel.plane_type_combo.blockSignals(True)
            self.cpanel.plane_d_slider.blockSignals(True)
            self.cpanel.plane_type_combo.setCurrentIndex(cp.plane_type)
            self.cpanel.plane_d_slider.set_value(cp.p3.w)
            self.cpanel.plane_type_combo.blockSignals(blocked)
            self.cpanel.plane_d_slider.blockSignals(blocked)

        # print_info('planes-select: {}'.format(value))
        self.render()

    def planes_type_select(self, value):
        cps = self.pblock.current_plane_set()
        if cps is None:
            return   
        if cps.selected_index >= 0 and cps.selected_index < cps.p3s.size():
            cp = cps.p3s[cps.selected_index]
            blocked = self.cpanel.plane_type_combo.blockSignals(True)
            cp.plane_type = self.cpanel.plane_type_combo.currentIndex()
            self.cpanel.plane_type_combo.blockSignals(blocked)

    def planes_d_update(self, value):
        cps = self.pblock.current_plane_set()
        if cps is None:
            return   
        if cps.selected_index < 0 or cps.selected_index >= cps.p3s.size():
            return
        cp = cps.p3s[cps.selected_index]
        if cps.selected_index == 0 and False:
            value = 0.0
            blocked = self.cpanel.plane_d_slider.blockSignals(True)
            self.cpanel.plane_d_slider.set_value(value)
            self.cpanel.plane_d_slider.blockSignals(blocked)
            
        cp.p3.w = value
        self.render()

    def planes_show(self):
        self.render()
        
    def planes_add(self):
        # Here we add a plane        
        cps = self.pblock.current_plane_set()
        if cps is None:
            return
        cps.p3s.push_back(CalibPlane())
        cps.selected_index = cps.p3s.size() - 1        
        self.update_planes_selected()

    def planes_del(self):
        cps = self.pblock.current_plane_set()
        if cps is None:
            return
        if cps.p3s.size() < 1:
            print_info('not deleting last CalibPlane')
            return
        cps.erase_p3_at_idx(cps.selected_index)
            
        # Here we delete a plane
        self.update_planes_selected()

    def planes_load_selected(self):
        # Must save/load a vector of these
        fname = '{}/TMP/plane-set-selected.json'.format(perceive_data_dir())
        print('loading plane-parameters from {}'.format(fname))
        cps = self.pblock.current_plane_set()
        self.pblock.calib_plane_sets.load(fname)
        self.update_planes_selected()
        # if not cps is None:
        #     cps.load(fname)
        #     print_info("load cps from '{}'".format(fname))
        #     self.update_planes_selected()
    
    def planes_save_selected(self):
        # Must save/load a vector of these
        fname = '{}/TMP/plane-set-selected.json'.format(perceive_data_dir())
        print('saving planes-parameters to {}'.format(fname))
        self.pblock.calib_plane_sets.save(fname)
        # cps = self.pblock.current_plane_set()
        #         if not cps is None:
        #             cps.save(fname)
        #             print_info("save cps from '{}'".format(fname))

    
