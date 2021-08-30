
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

from .labeled_slider import LabeledSlider
from .three_d_controls import ThreeDControls

# Must be on path
from multiview import *

class QHLine(QFrame):
    def __init__(self):
        super(QHLine, self).__init__()
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)

class QVLine(QFrame):
    def __init__(self):
        super(QVLine, self).__init__()
        self.setFrameShape(QFrame.VLine)
        self.setFrameShadow(QFrame.Sunken)

def make_vertical_spacer(height):
    return QtWidgets.QSpacerItem(20, height,
                                 QtWidgets.QSizePolicy.Minimum,
                                 QtWidgets.QSizePolicy.Expanding)
        
def make_double_line_edit(min_val=float('-inf'),
                          max_val=float('inf'),
                          units=1000):
    le = QLineEdit()
    validator = QDoubleValidator(min_val, max_val, units)
    validator.setNotation(QDoubleValidator.StandardNotation)
    le.setValidator(validator)
    return le

def make_panel_2(wgt1, wgt2):
    layout = QHBoxLayout()
    layout.addWidget(wgt1)
    layout.addWidget(wgt2)
    layout.setContentsMargins(0, 0, 0, 0)
    panel = QWidget()
    panel.setLayout(layout)
    return panel

def make_panel_3(wgt1, wgt2, wgt3):
    layout = QHBoxLayout()
    layout.addWidget(wgt1)
    layout.addWidget(wgt2)
    layout.addWidget(wgt3)
    layout.setContentsMargins(0, 0, 0, 0)
    panel = QWidget()
    panel.setLayout(layout)
    return panel

def make_panel_4(wgt1, wgt2, wgt3, wgt4):
    layout = QHBoxLayout()
    layout.addWidget(wgt1)
    layout.addWidget(wgt2)
    layout.addWidget(wgt3)
    layout.addWidget(wgt4)
    layout.setContentsMargins(0, 0, 0, 0)
    panel = QWidget()
    panel.setLayout(layout)
    return panel

def make_panel_5(wgt1, wgt2, wgt3, wgt4, wgt5):
    layout = QHBoxLayout()
    layout.addWidget(wgt1)
    layout.addWidget(wgt2)
    layout.addWidget(wgt3)
    layout.addWidget(wgt4)
    layout.addWidget(wgt5)
    layout.setContentsMargins(0, 0, 0, 0)
    panel = QWidget()
    panel.setLayout(layout)
    return panel

def make_panel_6(wgt1, wgt2, wgt3, wgt4, wgt5, wgt6):
    layout = QHBoxLayout()
    layout.addWidget(wgt1)
    layout.addWidget(wgt2)
    layout.addWidget(wgt3)
    layout.addWidget(wgt4)
    layout.addWidget(wgt5)
    layout.addWidget(wgt6)
    layout.setContentsMargins(0, 0, 0, 0)
    panel = QWidget()
    panel.setLayout(layout)
    return panel

def make_select_image_combo_box(scene_info):
    combo = QComboBox()
    bcam_keys = scene_info.bcam_keys
    for key in scene_info.bcam_keys:
        combo.addItem(key)
    return combo

class TweakerCPanel(QWidget):

    modelChanged = pyqtSignal()
    changed = pyqtSignal()
    onPlanesSelect = pyqtSignal(int)
    onPlanesTypeSelect = pyqtSignal(int)
    onPlanesUpdateD = pyqtSignal(float)
    onPlanesShow = pyqtSignal()
    onPlanesAdd = pyqtSignal()
    onPlanesDel = pyqtSignal()
    onPlanesSave = pyqtSignal()
    onPlanesLoad = pyqtSignal()

    # -------------------------------------------------------------- Constructor

    def __init__(self, pblock, parent = None):
        super().__init__(parent)
        self.pblock = pblock
        self.initUI()
        
    # ------------------------------------------------------------------ Init UI

    def make_display_widget_(self):
        
        # ---- Display Widgets ----
        scene_info = self.pblock.argz.scene_desc.scene_info
        self.select_image     = make_select_image_combo_box(scene_info)
        self.cb_left_image    = QCheckBox()
        self.rb_raw_video     = QRadioButton("Raw Video")
        self.rb_undistorted   = QRadioButton("Undistorted")
        self.rb_disparity_map = QRadioButton("Disparity Map")
        self.rb_point_cloud   = QRadioButton("Point Cloud")
        self.rb_scharr        = QRadioButton("Scharr")
        self.rb_scharr_thresh = QRadioButton("Scharr Threshold")
        self.rb_harris        = QRadioButton("Harris")
        self.rb_slic          = QRadioButton("Slic")
        self.rb_hist          = QRadioButton("Hist")
        self.rb_slic_3d       = QRadioButton("Slic 3d")
        self.rb_planes        = QRadioButton("Planes")

        # ---- Display Layout ----
        layout = QGridLayout()

        layout.addWidget(QLabel('Display'), 0, 0)     # Row 0
        layout.addWidget(self.select_image, 0, 1)
        layout.addWidget(make_panel_2(QLabel('Left Image'),
                                      self.cb_left_image), 0, 2)
        layout.addWidget(self.rb_raw_video, 1, 0)     # Row 1
        layout.addWidget(self.rb_undistorted, 1, 1)        
        layout.addWidget(self.rb_harris, 2, 0)        # Row 2
        layout.addWidget(self.rb_scharr, 2, 1)
        layout.addWidget(self.rb_scharr_thresh, 2, 2)        
        layout.addWidget(self.rb_slic,  3, 0)         # Row 3
        layout.addWidget(self.rb_slic_3d,  3, 1)
        layout.addWidget(self.rb_planes,  3, 2)
        layout.addWidget(self.rb_disparity_map, 4, 0) # Row 4
        layout.addWidget(self.rb_point_cloud, 4, 1)
        layout.addWidget(self.rb_hist, 4, 2)

        # ---- Display Initialization ----
        self.rb_undistorted.setChecked(True)
        self.cb_left_image.setChecked(True)
        
        # ---- Display Wiring ----
        self.select_image.currentIndexChanged.connect(self.cpanel_changed_1arg_)
        self.cb_left_image     .stateChanged .connect(self.cpanel_changed_1arg_)
        self.rb_undistorted    .toggled.connect(self.cpanel_changed_1arg_)
        self.rb_disparity_map  .toggled.connect(self.cpanel_changed_1arg_)
        self.rb_raw_video      .toggled.connect(self.cpanel_changed_1arg_)
        self.rb_point_cloud    .toggled.connect(self.cpanel_changed_1arg_)
        self.rb_scharr         .toggled.connect(self.cpanel_changed_1arg_)
        self.rb_scharr_thresh  .toggled.connect(self.cpanel_changed_1arg_)
        self.rb_harris         .toggled.connect(self.cpanel_changed_1arg_)
        self.rb_slic           .toggled.connect(self.cpanel_changed_1arg_)
        self.rb_hist           .toggled.connect(self.cpanel_changed_1arg_)
        self.rb_slic_3d        .toggled.connect(self.cpanel_changed_1arg_)
        self.rb_planes         .toggled.connect(self.cpanel_changed_1arg_)
        
        # -- #
        display_cpanel = QWidget()
        display_cpanel.setLayout(layout)
        return display_cpanel        

    def make_planes_panel_(self):

        def make_plane_view_panel():
            self.plane_rb_spixel_select     = QRadioButton("spixel select")
            self.plane_rb_show_d            = QRadioButton("show d")
            self.plane_rb_p3_result         = QRadioButton("p3 result")
            layout = QGridLayout()
            layout.addWidget(self.plane_rb_spixel_select, 0, 0)
            layout.addWidget(self.plane_rb_show_d       , 0, 1)
            layout.addWidget(self.plane_rb_p3_result    , 0, 2)
            self.plane_rb_spixel_select.setChecked(True)
            panel = QWidget()
            panel.setLayout(layout)
            return panel
        
        self.plane_combo                = QComboBox()
        self.plane_add_cp               = QPushButton("add cp")
        self.plane_del_cp               = QPushButton("remove")
        self.plane_type_combo           = QComboBox()
        self.plane_d_slider             = LabeledSlider(-4.00, 4.0,
                                                        (2.0 - -2.000)*1000 + 1)
        self.plane_view_panel = make_plane_view_panel()
        
        self.plane_save_select_btn      = QPushButton("save")
        self.plane_load_select_btn      = QPushButton("load")                

        self.plane_type_combo.addItem("X")
        self.plane_type_combo.addItem("Y")
        self.plane_type_combo.addItem("Z")
        
        layout = QFormLayout()
        layout.setVerticalSpacing(0)

        # Combo/Add/Remove
        layout.addWidget(self.plane_combo)
        layout.addItem(make_vertical_spacer(4))
        layout.addWidget(self.plane_type_combo)
        layout.addItem(make_vertical_spacer(4))
        layout.addWidget(self.plane_view_panel)
        layout.addItem(make_vertical_spacer(4))
        layout.addRow("d", self.plane_d_slider)
        layout.addItem(make_vertical_spacer(8))
        layout.addWidget(self.plane_add_cp)
        layout.addItem(make_vertical_spacer(4))
        layout.addWidget(self.plane_del_cp)

        # Load/Save
        layout.addItem(make_vertical_spacer(52))
        layout.addWidget(self.plane_save_select_btn)
        layout.addItem(make_vertical_spacer(4))
        layout.addWidget(self.plane_load_select_btn)
        
        wgts = QWidget()
        wgts.setLayout(layout)
        return wgts
    
    def make_cpanel_(self):

        # ---- cpanel widgets ----
        # Undistorted image size parameters
        self.uimagesz_w_slider           = LabeledSlider(256, 1024, 1024-256+1)
        self.uimagesz_h_slider           = LabeledSlider(256, 1024, 1024-256+1)
        self.uimagesz_ppt_dx_slider      = LabeledSlider(-400, 400, 801)
        self.uimagesz_ppt_dy_slider      = LabeledSlider(-400, 400, 801)
        self.uimagesz_f_slider           = LabeledSlider(100, 800, 800-100+1)
        self.uimagesz_use_calib_slider   = LabeledSlider(0, 1, 2)
        self.uimagesz_software_equalize  = LabeledSlider(0, 1, 2)
        
        # Disparity
        self.disp_method_combo = QComboBox()
        self.disp_method_combo.addItem("StereoBM")
        self.disp_method_combo.addItem("StereoSGBM")
        self.disp_method_combo.addItem("Triclops")
        self.disp_view_select_slider         = LabeledSlider(0, 4, 5)
        self.disp_presmooth_sigma_slider     = LabeledSlider(0.0, 10.0, 1001)
        
        min_SAD, max_SAD = 7, 51
        self.disp_SAD_window_size_slider     = LabeledSlider(min_SAD,max_SAD, (max_SAD-min_SAD)/2+1)
        self.disp_12_max_diff_slider         = LabeledSlider(0, 1000, 1001)
        self.disp_min_disparity_slider       = LabeledSlider(0, 100, 101)
        min_nd, max_nd = 16, 1024
        self.disp_num_disparities_slider     = LabeledSlider(min_nd, max_nd, (max_nd - min_nd)/16+1)
        self.disp_spek_window_size_slider    = LabeledSlider(0, 1000, 1001)
        self.disp_spek_range_slider          = LabeledSlider(0, 1000, 1001)
        self.disp_prefilter_cap_slider       = LabeledSlider(0, 63, 64)
        min_pf, max_pf = 5, 51
        self.disp_prefilter_size_slider      = LabeledSlider(min_pf, max_pf, (max_pf - min_pf)/2+1)
        self.disp_prefilter_type_slider      = LabeledSlider(0, 1, 2)
        self.disp_block_size_slider          = LabeledSlider(0, 100, 101)
        self.disp_texture_threshold_slider   = LabeledSlider(0, 5000, 5001)
        self.disp_unique_ratio_slider        = LabeledSlider(0, 100, 101)

        self.disp_sg_SAD_window_size_slider  = LabeledSlider(min_SAD,max_SAD, (max_SAD-min_SAD)/2+1)
        self.disp_sg_12_max_diff_slider      = LabeledSlider(0, 1000, 1001)
        self.disp_sg_min_disparity_slider    = LabeledSlider(0, 100, 101)
        min_nd, max_nd = 16, 1024
        self.disp_sg_num_disparities_slider  = LabeledSlider(min_nd, max_nd, (max_nd - min_nd)/16+1)
        self.disp_sg_spek_window_size_slider = LabeledSlider(0, 1000, 1001)
        self.disp_sg_spek_range_slider        = LabeledSlider(0, 1000, 1001)
        self.disp_sg_prefilter_cap_slider    = LabeledSlider(0, 63, 64)
        self.disp_sg_unique_ratio_slider     = LabeledSlider(0, 100, 101)
        self.disp_sg_mode_slider             = LabeledSlider(1, 4, 4)
        self.disp_sg_p1_slider               = LabeledSlider(0, 1000, 1001)
        self.disp_sg_p2_slider               = LabeledSlider(0, 1000, 1001)
        
        self.disp_apply_wls_slider           = LabeledSlider(0, 1, 2)
        self.disp_wls_lambda_slider          = LabeledSlider(0, 20000, 2001)
        self.disp_wls_sigma_slider           = LabeledSlider(0.1, 10.0, 1001)
        self.disp_wls_lrc_thres_slider       = LabeledSlider(3, 100, 100-3+1)
        self.disp_wls_discont_radius_slider  = LabeledSlider(0, 50, 50 - 0 + 1)
        
        self.disp_tri_lowpass_slider         = LabeledSlider(0, 1, 2)
        self.disp_tri_subpixel_slider        = LabeledSlider(0, 1, 2)
        self.disp_tri_subpixel_val_slider    = LabeledSlider(0, 1, 2)
        self.disp_tri_edge_correl_slider     = LabeledSlider(0, 1, 2)
        self.disp_tri_min_disp_slider        = LabeledSlider(0, 50, 51)
        self.disp_tri_max_disp_slider        = LabeledSlider(51, 200, 200-51+1)
        self.disp_tri_edge_mask_slider       = LabeledSlider(7, 51, (51 - 7)/2 + 1)
        self.disp_tri_stereo_mask_slider     = LabeledSlider(7, 51, (51 - 7)/2 + 1)
        self.disp_tri_texture_val_slider     = LabeledSlider(0, 1, 2)
        self.disp_tri_surface_val_slider     = LabeledSlider(0, 1, 2)
        self.disp_tri_surface_val_sz_slider  = LabeledSlider(100, 1000, 1000-100+1)

        # Features2d
        self.scharr_blur_size_slider    = LabeledSlider(3, 11, (11 - 3)/2 + 1)
        self.scharr_blur_sigma_slider   = LabeledSlider(0.1, 20.0,
                                                        (20.0 - 0.1) * 100 + 1)
        self.harris_block_size_slider   = LabeledSlider(3, 11, (11 - 3)/2 + 1)
        self.harris_k_slider            = LabeledSlider(0.001, 0.1,
                                                        (0.1 - 0.001)*1000 + 1)
        self.harris_use_slider          = LabeledSlider(0, 1, 2)
        self.harris_max_corners_slider  = LabeledSlider(10, 1000, 1000 - 10 + 1)
        self.harris_quality_slider      = LabeledSlider(0.001, 0.1,
                                                        (0.1 - 0.001)*1000 + 1)
        self.harris_min_dist_slider     = LabeledSlider(1, 20, 20 - 1 + 1)
        self.slic_superpixel_size_slider= LabeledSlider(50, 2000, 2000 - 50 + 1)
        self.slic_compactness_slider    = LabeledSlider(0.1, 20.0,
                                                        (20.0 - 0.1)*10 + 1)
        self.slic_test_opt_slider       = LabeledSlider(0, 1, 2)
        self.slic_test_set_lbl_slider   = LabeledSlider(0, 1, 2)
        self.slic_test_d_slider         = LabeledSlider(0, 4, 1000)
        self.slic_test_inc_slider       = LabeledSlider(-math.pi, math.pi, 1000)
        self.slic_test_azi_slider       = LabeledSlider(-math.pi, math.pi, 1000)

        # Planes
        self.plane_widgets              = self.make_planes_panel_()
                
        # ---- cpanel Layout ----
        layout = QFormLayout()
        layout.setVerticalSpacing(0)

        # Video        
        layout.addItem(make_vertical_spacer(11))

        # Undistorted image size parameters
        layout.addRow('Width',              self.uimagesz_w_slider)
        layout.addRow('Height',             self.uimagesz_h_slider)
        layout.addRow('ppt dx',             self.uimagesz_ppt_dx_slider)
        layout.addRow('ppt dy',             self.uimagesz_ppt_dy_slider)
        layout.addRow('focal length',       self.uimagesz_f_slider)
        layout.addRow('use calib region',   self.uimagesz_use_calib_slider)
        layout.addRow('software equalize',  self.uimagesz_software_equalize)
        
        # Disparity
        layout.addRow('Disparity method',   self.disp_method_combo)
        layout.addRow('Presmooth sigma',    self.disp_presmooth_sigma_slider)
        layout.addRow('View',               self.disp_view_select_slider)
        
        layout.addRow('SAD window size',    self.disp_SAD_window_size_slider)
        layout.addRow('12 max diff',        self.disp_12_max_diff_slider)
        layout.addRow('Min disparity',      self.disp_min_disparity_slider)
        layout.addRow('# of disparities',   self.disp_num_disparities_slider)
        layout.addRow('Speckle window size',self.disp_spek_window_size_slider)
        layout.addRow('Speckle range',      self.disp_spek_range_slider)
        layout.addRow('Prefilter cap',      self.disp_prefilter_cap_slider)
        layout.addRow('Prefilter size',     self.disp_prefilter_size_slider)
        layout.addRow('Prefilter type',     self.disp_prefilter_type_slider)
        layout.addRow('Block size',         self.disp_block_size_slider)
        layout.addRow('Texture threshold',  self.disp_texture_threshold_slider)
        layout.addRow('Uniquness ratio',    self.disp_unique_ratio_slider)

        layout.addRow('SAD window size',    self.disp_sg_SAD_window_size_slider)
        layout.addRow('12 max diff',        self.disp_sg_12_max_diff_slider)
        layout.addRow('Min disparity',      self.disp_sg_min_disparity_slider)
        layout.addRow('# of disparities',   self.disp_sg_num_disparities_slider)
        layout.addRow('Speckle window size',self.disp_sg_spek_window_size_slider)
        layout.addRow('Speckle range',      self.disp_sg_spek_range_slider)
        layout.addRow('Prefilter cap',      self.disp_sg_prefilter_cap_slider)
        layout.addRow('Uniquness ratio',    self.disp_sg_unique_ratio_slider)
        layout.addRow('SGBM Mode',          self.disp_sg_mode_slider)
        layout.addRow('P1',                 self.disp_sg_p1_slider)
        layout.addRow('P2',                 self.disp_sg_p2_slider)
        
        layout.addRow('Apply WLS filter',   self.disp_apply_wls_slider)
        layout.addRow('WLS lambda',         self.disp_wls_lambda_slider)
        layout.addRow('WLS sigma',          self.disp_wls_sigma_slider)
        layout.addRow('WLS LRC thresh',     self.disp_wls_lrc_thres_slider)
        layout.addRow('WLS discont radius', self.disp_wls_discont_radius_slider)
        
        layout.addRow('Lowpass',            self.disp_tri_lowpass_slider)
        layout.addRow('Subpixel',           self.disp_tri_subpixel_slider)
        layout.addRow('Subpixel validation',self.disp_tri_subpixel_val_slider)
        layout.addRow('Edge correlation',   self.disp_tri_edge_correl_slider)
        layout.addRow('Min disparity',      self.disp_tri_min_disp_slider)
        layout.addRow('Max disparity',      self.disp_tri_max_disp_slider)
        layout.addRow('Edge mask',          self.disp_tri_edge_mask_slider)
        layout.addRow('Stereo mask',        self.disp_tri_stereo_mask_slider)
        layout.addRow('Texture validation', self.disp_tri_texture_val_slider)
        layout.addRow('Surface validation', self.disp_tri_surface_val_slider)
        layout.addRow('Surface val size',   self.disp_tri_surface_val_sz_slider)
                
        # Features2d        
        layout.addRow('Blur kernel size',   self.scharr_blur_size_slider)
        layout.addRow('Blur sigma',         self.scharr_blur_sigma_slider)
        layout.addRow('Block size',         self.harris_block_size_slider)
        layout.addRow('Harris k',           self.harris_k_slider)
        layout.addRow('Use harris',         self.harris_use_slider)
        layout.addRow('Max corners',        self.harris_max_corners_slider)
        layout.addRow('Quality',            self.harris_quality_slider)
        layout.addRow('Minimum distance',   self.harris_min_dist_slider)
        layout.addRow('Superpixel size',    self.slic_superpixel_size_slider)
        layout.addRow('Compactness',        self.slic_compactness_slider)
        layout.addRow('Test apply-opt',     self.slic_test_opt_slider)
        layout.addRow('Test set label',     self.slic_test_set_lbl_slider)
        layout.addRow('Test d',             self.slic_test_d_slider)
        layout.addRow('Test n inclination', self.slic_test_inc_slider)
        layout.addRow('Test n azimuth',     self.slic_test_azi_slider)

        # Planes
        layout.addWidget(self.plane_widgets)
        
        # ---- cpanel Wiring ----
        # Undistorted image size parameters
        self.uimagesz_w_slider              .valueChanged.connect(self.wgt1arg_)
        self.uimagesz_h_slider              .valueChanged.connect(self.wgt1arg_)
        self.uimagesz_ppt_dx_slider         .valueChanged.connect(self.wgt1arg_)
        self.uimagesz_ppt_dy_slider         .valueChanged.connect(self.wgt1arg_)
        self.uimagesz_f_slider              .valueChanged.connect(self.wgt1arg_)
        self.uimagesz_use_calib_slider      .valueChanged.connect(self.wgt1arg_)
        self.uimagesz_software_equalize     .valueChanged.connect(self.wgt1arg_)
        
        # Disparity
        self.disp_method_combo              .activated   .connect(self.wgt1arg_)
        self.disp_view_select_slider        .valueChanged.connect(self.cpanel_changed_1arg_)
        self.disp_presmooth_sigma_slider    .valueChanged.connect(self.wgt1arg_)
        
        self.disp_SAD_window_size_slider    .valueChanged.connect(self.wgt1arg_)
        self.disp_12_max_diff_slider        .valueChanged.connect(self.wgt1arg_)
        self.disp_min_disparity_slider      .valueChanged.connect(self.wgt1arg_)
        self.disp_num_disparities_slider    .valueChanged.connect(self.wgt1arg_)
        self.disp_spek_window_size_slider   .valueChanged.connect(self.wgt1arg_)
        self.disp_spek_range_slider         .valueChanged.connect(self.wgt1arg_)
        self.disp_prefilter_cap_slider      .valueChanged.connect(self.wgt1arg_)
        self.disp_prefilter_size_slider     .valueChanged.connect(self.wgt1arg_)
        self.disp_prefilter_type_slider     .valueChanged.connect(self.wgt1arg_)
        self.disp_block_size_slider         .valueChanged.connect(self.wgt1arg_)
        self.disp_texture_threshold_slider  .valueChanged.connect(self.wgt1arg_)
        self.disp_unique_ratio_slider       .valueChanged.connect(self.wgt1arg_)

        self.disp_sg_SAD_window_size_slider .valueChanged.connect(self.wgt1arg_)
        self.disp_sg_12_max_diff_slider     .valueChanged.connect(self.wgt1arg_)
        self.disp_sg_min_disparity_slider   .valueChanged.connect(self.wgt1arg_)
        self.disp_sg_num_disparities_slider .valueChanged.connect(self.wgt1arg_)
        self.disp_sg_spek_window_size_slider.valueChanged.connect(self.wgt1arg_)
        self.disp_sg_spek_range_slider      .valueChanged.connect(self.wgt1arg_)
        self.disp_sg_prefilter_cap_slider   .valueChanged.connect(self.wgt1arg_)
        self.disp_sg_unique_ratio_slider    .valueChanged.connect(self.wgt1arg_)
        self.disp_sg_mode_slider            .valueChanged.connect(self.wgt1arg_)
        self.disp_sg_p1_slider              .valueChanged.connect(self.wgt1arg_)
        self.disp_sg_p2_slider              .valueChanged.connect(self.wgt1arg_)
        
        self.disp_apply_wls_slider          .valueChanged.connect(self.wgt1arg_)
        self.disp_wls_lambda_slider         .valueChanged.connect(self.wgt1arg_)
        self.disp_wls_sigma_slider          .valueChanged.connect(self.wgt1arg_)
        self.disp_wls_lrc_thres_slider      .valueChanged.connect(self.wgt1arg_)
        self.disp_wls_discont_radius_slider .valueChanged.connect(self.wgt1arg_)
        
        self.disp_tri_lowpass_slider        .valueChanged.connect(self.wgt1arg_)
        self.disp_tri_subpixel_slider       .valueChanged.connect(self.wgt1arg_)
        self.disp_tri_subpixel_val_slider   .valueChanged.connect(self.wgt1arg_)
        self.disp_tri_edge_correl_slider    .valueChanged.connect(self.wgt1arg_)
        self.disp_tri_min_disp_slider       .valueChanged.connect(self.wgt1arg_)
        self.disp_tri_max_disp_slider       .valueChanged.connect(self.wgt1arg_)
        self.disp_tri_edge_mask_slider      .valueChanged.connect(self.wgt1arg_)
        self.disp_tri_stereo_mask_slider    .valueChanged.connect(self.wgt1arg_)
        self.disp_tri_texture_val_slider    .valueChanged.connect(self.wgt1arg_)
        self.disp_tri_surface_val_slider    .valueChanged.connect(self.wgt1arg_)
        self.disp_tri_surface_val_sz_slider .valueChanged.connect(self.wgt1arg_)

        # Scharr        
        self.scharr_blur_size_slider        .valueChanged.connect(self.wgt1arg_)
        self.scharr_blur_sigma_slider       .valueChanged.connect(self.wgt1arg_)

        # Harris
        self.harris_block_size_slider       .valueChanged.connect(self.wgt1arg_)
        self.harris_k_slider                .valueChanged.connect(self.wgt1arg_)
        self.harris_use_slider              .valueChanged.connect(self.wgt1arg_)
        self.harris_max_corners_slider      .valueChanged.connect(self.wgt1arg_)
        self.harris_quality_slider          .valueChanged.connect(self.wgt1arg_)
        self.harris_min_dist_slider         .valueChanged.connect(self.wgt1arg_)
        
        # SLIC
        self.slic_superpixel_size_slider    .valueChanged.connect(self.wgt1arg_)
        self.slic_compactness_slider        .valueChanged.connect(self.wgt1arg_)
        self.slic_test_d_slider             .valueChanged.connect(self.wgt1arg_)

        # Planes
        self.plane_combo.currentIndexChanged.connect(self.planes_combo_sel_)
        self.plane_type_combo.currentIndexChanged.connect(self.planes_type_combo_sel_)
        self.plane_d_slider          .valueChanged.connect(self.planes_d_update_)
        self.plane_add_cp            .clicked.connect(self.planes_add_clicked_)
        self.plane_del_cp            .clicked.connect(self.planes_del_clicked_)
        self.plane_save_select_btn   .clicked.connect(self.planes_save_clicked_)
        self.plane_load_select_btn   .clicked.connect(self.planes_load_clicked_)
        self.plane_rb_spixel_select  .clicked.connect(self.planes_show_clicked_)
        self.plane_rb_show_d         .clicked.connect(self.planes_show_clicked_)
        self.plane_rb_p3_result      .clicked.connect(self.planes_show_clicked_)
        
        # -- #
        self.video_cpanel_layout = layout
        video_cpanel = QWidget()        
        video_cpanel.setLayout(layout)
        #video_cpanel.setFixedHeight(720)
        video_cpanel.setSizePolicy(QSizePolicy.Expanding,
                                   QSizePolicy.Expanding)
        return video_cpanel

    def make_gl_cpanel_(self):

        n_cameras = self.pblock.argz.scene_desc.n_cameras()
        
        # ---- OpenGl widgets ----
        self.gl_cb_rotate            = QCheckBox()
        self.gl_cb_image_colors      = QCheckBox()
        self.gl_cb_draw_axis         = QCheckBox()
        self.gl_cb_draw_grid_xy      = QCheckBox()
        self.gl_cb_draw_grid_xz      = QCheckBox()
        self.gl_cb_draw_grid_yz      = QCheckBox()
        self.gl_cb_draw_cad_model    = QCheckBox()
        self.gl_cb_draw_aruco_sensor = QCheckBox()
        self.gl_cb_draw_aruco_cam    = QCheckBox()
        self.gl_cb_draw_final_cam    = QCheckBox()
        self.gl_cb_pt_cloud_final    = QCheckBox()
        
        # Selecting point clouds to view
        self.gl_cbs_view = []

        def make_disparites_wgt(cam_num):
            cb_list = []
            hbox = QHBoxLayout()
            n_disparities = TweakerResults.n_disparities()+2
            for i in range(n_disparities):
                cb = QCheckBox()
                cb.setObjectName('cb_{}_{}'.format(cam_num, i))
                hbox.addWidget(cb)
                cb_list.append(cb)
            wgt = QWidget()
            wgt.setLayout(hbox)
            self.gl_cbs_view.append(cb_list)
            return wgt

        # Camera select (for rotation controls)
        scene_info = self.pblock.argz.scene_desc.scene_info
        self.gl_select_cam = make_select_image_combo_box(scene_info)
        self.gl_select_cam.insertItem(0, 'OpenGL Camera')
        self.gl_select_cam.setCurrentIndex(0)
        
        # Rotation controls        
        self.gl_simt = ThreeDControls('<b>Similarity Transform Controls</b>')
        
        # ---- Layout ----
        layout = QFormLayout()
        layout.setVerticalSpacing(0)
        
        layout.addRow('Rotate',              self.gl_cb_rotate)
        layout.addRow('Image colors',        self.gl_cb_image_colors)
        layout.addRow('Draw axis',           self.gl_cb_draw_axis)
        layout.addRow('Draw grid', make_panel_3(self.gl_cb_draw_grid_xy,
                                                self.gl_cb_draw_grid_xz,
                                                self.gl_cb_draw_grid_yz))
        layout.addRow('Draw CAD',            self.gl_cb_draw_cad_model)
        layout.addRow('Draw Aruco', make_panel_5(self.gl_cb_draw_aruco_sensor,
                                                QLabel('A-Cam'),
                                                self.gl_cb_draw_aruco_cam,
                                                QLabel('Final'),
                                                self.gl_cb_draw_final_cam))

        for cam_num in range(n_cameras):
            cam_name = scene_info.bcam_keys[cam_num]
            layout.addRow(cam_name, make_disparites_wgt(cam_num))

        layout.addRow('Pt-cloud pos', self.gl_cb_pt_cloud_final)

        # Rotation controls
        layout.addItem(make_vertical_spacer(11))
        layout.addRow(QHLine())
        layout.addItem(make_vertical_spacer(11))
        layout.addRow(self.gl_select_cam)
        layout.addRow(self.gl_simt)
        layout.addRow(QHLine())
        
        # ---- Wiring ----
        self.gl_select_cam.currentIndexChanged.connect(self.gl_opts_camera_sel_)
        
        # ---- Initialization ----
        self.gl_opts_to_widgets()
        
        gl_cpanel = QWidget()
        gl_cpanel.setLayout(layout)
        self.gl_cpanel_layout = layout
        return gl_cpanel
        
    def initUI(self):                
      
        # ---- Make widgets ----
        self.display_cpanel = self.make_display_widget_()
        self.video_cpanel   = self.make_cpanel_()
        self.gl_cpanel      = self.make_gl_cpanel_()
    
        # ---- Layout ----
        layout = QVBoxLayout()
        layout.addWidget(self.display_cpanel)
        layout.addWidget(QHLine())
        layout.addWidget(self.video_cpanel)
        layout.addWidget(self.gl_cpanel)
        layout.addItem(make_vertical_spacer(1))
        layout.addStretch(1)
        self.layout = layout
        self.setLayout(layout)
        self.setFixedWidth(self.pblock.app_settings.cpanel_width)
        self.setFixedHeight(850)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
                
        # ---- Initialization ----
        self.model_to_widgets()

        # ---- Wiring ----
        # -- Model changed is connected to pblock -- #
        self.modelChanged.connect(self.pblock.execute_threaded)

        self.signal_ms = 125 # We collate modelChanged signals
        self.model_changed_timer_ = QTimer()
        self.model_changed_timer_.setSingleShot(True)
        self.model_changed_timer_.timeout.connect(self.emit_model_changed_)

    # ------------------------------------------------------------------ Helpers

    def selected_camera(self):
        return self.select_image.currentIndex()

    def update_selected_camera(self, value):
        if value < 0 or value >= self.select_image.count():
            print_err('Camera index out of range: {} (n-cameras = {})'
                      .format(value, self.select_image.count()))
            return
        if value != self.selected_camera():
            self.model_to_widgets()
            self.select_image.setCurrentIndex(value)            
    
    def wgt1arg_(self, value):
        """Signals that take 1 argument (like valueChanged(value)) are 
        routed through this function. We really just want to call 
        'self.widget_to_model()'"""
        self.widgets_to_model()

    def planes_combo_sel_(self, value):
        self.onPlanesSelect.emit(value)

    def planes_type_combo_sel_(self, value):
        self.onPlanesTypeSelect.emit(value)
        
    def planes_d_update_(self, value):
        self.onPlanesUpdateD.emit(value)

    def planes_show_clicked_(self):
        self.onPlanesShow.emit()    
        
    def planes_add_clicked_(self, value):
        self.onPlanesAdd.emit()

    def planes_del_clicked_(self, value):
        self.onPlanesDel.emit()
        
    def planes_save_clicked_(self, value):
        self.onPlanesSave.emit()

    def planes_load_clicked_(self, value):
        self.onPlanesLoad.emit()
        
    def emit_model_changed_(self):
        self.show_hide_widgets()
        self.emit_changed_()
        
    def cpanel_changed_1arg_(self, value):
        """Checkbox and other non-model related widgets connect to this slot."""
        self.show_hide_widgets()
        self.changed.emit()
        
    def emit_changed_(self):
        """(Private) target method for QTimer that emits the modelChanged 
        signal"""
        self.modelChanged.emit()
        
    def show_hide_widgets(self):
        """We show/hide input widgets depending on the disparity method"""

        is_raw = self.rb_raw_video.isChecked()
        is_undistorted = self.rb_undistorted.isChecked()
        is_disparity = self.rb_disparity_map.isChecked()
        is_scharr = self.rb_scharr.isChecked()
        is_scharr_thresh = self.rb_scharr_thresh.isChecked();
        is_harris = self.rb_harris.isChecked()
        is_slic = self.rb_slic.isChecked()
        is_hist = self.rb_hist.isChecked()
        is_slic_3d = self.rb_slic_3d.isChecked()
        is_planes = self.rb_planes.isChecked()
        
        is_stereo_bm = is_disparity \
                       and self.disp_method_combo.currentIndex() == 0
        is_stereo_sgbm = is_disparity \
                         and self.disp_method_combo.currentIndex() == 1
        is_stereo_triclops = is_disparity \
                             and not is_stereo_bm and not is_stereo_sgbm

        # Which subpanel are we showing
        show_video_cpanel = self.rb_raw_video.isChecked() or \
                            self.rb_undistorted.isChecked() or \
                            self.rb_disparity_map.isChecked() or \
                            is_slic or \
                            is_scharr or is_scharr_thresh or \
                            is_harris or is_hist or is_planes

        show_gl_cpanel = not show_video_cpanel

        vid_cpanel_height = self.video_cpanel.height()
        
        self.video_cpanel.setVisible(show_video_cpanel)
        self.gl_cpanel.setVisible(show_gl_cpanel)
        
        def process_widget(widget, visible):
            self.video_cpanel_layout.labelForField(widget).setVisible(visible)
            widget.setVisible(visible)

        # Raw input    
        process_widget(self.uimagesz_software_equalize, is_raw)
        
        # Undistorted image size parameters
        process_widget(self.uimagesz_w_slider, is_undistorted)
        process_widget(self.uimagesz_h_slider, is_undistorted)
        process_widget(self.uimagesz_ppt_dx_slider, is_undistorted)
        process_widget(self.uimagesz_ppt_dy_slider, is_undistorted)
        process_widget(self.uimagesz_f_slider, is_undistorted)
        process_widget(self.uimagesz_use_calib_slider, is_undistorted)
        
        # Any disparity
        process_widget(self.disp_method_combo, is_disparity)
        process_widget(self.disp_presmooth_sigma_slider, is_disparity)
        process_widget(self.disp_view_select_slider, is_disparity)
                    
        # Stereo BM
        process_widget(self.disp_SAD_window_size_slider, is_stereo_bm)
        process_widget(self.disp_12_max_diff_slider, is_stereo_bm)
        process_widget(self.disp_min_disparity_slider, is_stereo_bm)
        process_widget(self.disp_num_disparities_slider, is_stereo_bm)
        process_widget(self.disp_spek_window_size_slider, is_stereo_bm)
        process_widget(self.disp_spek_range_slider, is_stereo_bm)
        process_widget(self.disp_prefilter_cap_slider, is_stereo_bm)
        process_widget(self.disp_prefilter_size_slider, is_stereo_bm)
        process_widget(self.disp_prefilter_type_slider, is_stereo_bm)
        process_widget(self.disp_block_size_slider, is_stereo_bm)
        process_widget(self.disp_texture_threshold_slider, is_stereo_bm)
        process_widget(self.disp_unique_ratio_slider, is_stereo_bm)
        
        # Stereo SGBM
        process_widget(self.disp_sg_SAD_window_size_slider, is_stereo_sgbm)
        process_widget(self.disp_sg_12_max_diff_slider, is_stereo_sgbm)
        process_widget(self.disp_sg_min_disparity_slider, is_stereo_sgbm)
        process_widget(self.disp_sg_num_disparities_slider, is_stereo_sgbm)
        process_widget(self.disp_sg_spek_window_size_slider, is_stereo_sgbm)
        process_widget(self.disp_sg_spek_range_slider, is_stereo_sgbm)
        process_widget(self.disp_sg_prefilter_cap_slider, is_stereo_sgbm)
        process_widget(self.disp_sg_unique_ratio_slider, is_stereo_sgbm)
        process_widget(self.disp_sg_mode_slider, is_stereo_sgbm)
        process_widget(self.disp_sg_p1_slider, is_stereo_sgbm)
        process_widget(self.disp_sg_p2_slider, is_stereo_sgbm)

        # WLS
        process_widget(self.disp_apply_wls_slider,
                       is_stereo_sgbm or is_stereo_bm)
        process_widget(self.disp_wls_lambda_slider,
                       is_stereo_sgbm or is_stereo_bm)
        process_widget(self.disp_wls_sigma_slider,
                       is_stereo_sgbm or is_stereo_bm)
        process_widget(self.disp_wls_lrc_thres_slider,
                       is_stereo_sgbm or is_stereo_bm)
        process_widget(self.disp_wls_discont_radius_slider,
                       is_stereo_sgbm or is_stereo_bm)
        
        # Triclops
        process_widget(self.disp_tri_lowpass_slider, is_stereo_triclops)
        process_widget(self.disp_tri_subpixel_slider, is_stereo_triclops)
        process_widget(self.disp_tri_subpixel_val_slider, is_stereo_triclops) 
        process_widget(self.disp_tri_edge_correl_slider, is_stereo_triclops)   
        process_widget(self.disp_tri_min_disp_slider, is_stereo_triclops)      
        process_widget(self.disp_tri_max_disp_slider, is_stereo_triclops)       
        process_widget(self.disp_tri_edge_mask_slider, is_stereo_triclops)      
        process_widget(self.disp_tri_stereo_mask_slider, is_stereo_triclops)    
        process_widget(self.disp_tri_texture_val_slider, is_stereo_triclops)    
        process_widget(self.disp_tri_surface_val_slider, is_stereo_triclops)    
        process_widget(self.disp_tri_surface_val_sz_slider, is_stereo_triclops)

        # Scharr
        process_widget(self.scharr_blur_size_slider, is_scharr)
        process_widget(self.scharr_blur_sigma_slider, is_scharr)

        # Harris
        process_widget(self.harris_block_size_slider, is_harris)
        process_widget(self.harris_k_slider, is_harris)
        process_widget(self.harris_use_slider, is_harris)
        process_widget(self.harris_max_corners_slider, is_harris)
        process_widget(self.harris_quality_slider, is_harris)
        process_widget(self.harris_min_dist_slider, is_harris)
        
        # SLIC
        process_widget(self.slic_superpixel_size_slider, is_slic)
        process_widget(self.slic_compactness_slider, is_slic)
        process_widget(self.slic_test_opt_slider, is_slic)
        process_widget(self.slic_test_set_lbl_slider, is_slic)
        process_widget(self.slic_test_d_slider, is_slic)
        process_widget(self.slic_test_inc_slider, is_slic)
        process_widget(self.slic_test_azi_slider, is_slic)

        # Planes
        self.plane_widgets.setVisible(is_planes)
                
    # ---------------------------------------------------------- Widget to Model
        
    def widgets_to_model(self):
        """Copys widget values into the data model, and then initites 
        the emission of a signal of the model has indeed changed."""

        cam_num = self.selected_camera()
        if cam_num < 0 or cam_num >= self.pblock.models.size():
            return
        
        m = copy.copy(self.pblock.models[cam_num])

        ppt = Vector2(float(self.uimagesz_ppt_dx_slider.value()) + 0.5*m.width,
                      float(self.uimagesz_ppt_dy_slider.value()) + 0.5*m.height)
        m.width                = int(self.uimagesz_w_slider.value())
        m.height               = int(self.uimagesz_h_slider.value())
        m.focal_length         = float(self.uimagesz_f_slider.value())
        m.ppt                  = ppt
        m.use_calib_roi        = bool(self.uimagesz_use_calib_slider.value())
        m.software_equalize    = bool(self.uimagesz_software_equalize.value())
        
        p = m.disp_params
        p.method               = self.disp_method_combo.currentIndex()
        p.presmooth_sigma      = float(self.disp_presmooth_sigma_slider.value())
        
        p.bm_SAD_window_size   = int(self.disp_SAD_window_size_slider.value())
        p.bm_max_diff_12       = int(self.disp_12_max_diff_slider.value())
        p.bm_min_disparity     = int(self.disp_min_disparity_slider.value())
        p.bm_num_disparities   = int(self.disp_num_disparities_slider.value())
        p.bm_speckle_window_size= int(self.disp_spek_window_size_slider.value())
        p.bm_speckle_range     = int(self.disp_spek_range_slider.value())
        p.bm_prefilter_cap     = int(self.disp_prefilter_cap_slider.value())
        p.bm_prefilter_size    = int(self.disp_prefilter_size_slider.value())
        p.bm_prefilter_type    = int(self.disp_prefilter_type_slider.value())
        p.bm_block_size        = int(self.disp_block_size_slider.value())
        p.bm_texture_threshold = int(self.disp_texture_threshold_slider.value())
        p.bm_uniqueness_ratio  = int(self.disp_unique_ratio_slider.value())

        p.sg_SAD_window_size   =int(self.disp_sg_SAD_window_size_slider.value())
        p.sg_max_diff_12       = int(self.disp_sg_12_max_diff_slider.value())
        p.sg_min_disparity     = int(self.disp_sg_min_disparity_slider.value())
        p.sg_num_disparities   =int(self.disp_sg_num_disparities_slider.value())
        p.sg_speckle_window_size=int(self.disp_sg_spek_window_size_slider.value())
        p.sg_speckle_range     = int(self.disp_sg_spek_range_slider.value())
        p.sg_prefilter_cap     = int(self.disp_sg_prefilter_cap_slider.value())
        p.sg_uniqueness_ratio =int(self.disp_sg_unique_ratio_slider.value())
        p.sg_mode              = int(self.disp_sg_mode_slider.value())
        p.sg_p1                = int(self.disp_sg_p1_slider.value())
        p.sg_p2                = int(self.disp_sg_p2_slider.value())
        
        p.apply_wls            = bool(self.disp_apply_wls_slider.value())
        p.wls_lambda           = float(self.disp_wls_lambda_slider.value())
        p.wls_sigma            = float(self.disp_wls_sigma_slider.value())
        p.wls_lrc_thres        = int(self.disp_wls_lrc_thres_slider.value())
        p.wls_dicont_radius    =int(self.disp_wls_discont_radius_slider.value())
        p.tri_lowpass          = bool(self.disp_tri_lowpass_slider.value())    
        p.tri_subpixel         = bool(self.disp_tri_subpixel_slider.value())    
        p.tri_subpixel_valid   = bool(self.disp_tri_subpixel_val_slider.value())
        p.tri_edge_correlation = bool(self.disp_tri_edge_correl_slider.value()) 
        p.tri_min_disp         = int(self.disp_tri_min_disp_slider.value())     
        p.tri_max_disp         = int(self.disp_tri_max_disp_slider.value())     
        p.tri_edge_mask        = int(self.disp_tri_edge_mask_slider.value())    
        p.tri_stereo_mask      = int(self.disp_tri_stereo_mask_slider.value())  
        p.tri_text_valid       = bool(self.disp_tri_texture_val_slider.value()) 
        p.tri_surf_valid       = bool(self.disp_tri_surface_val_slider.value()) 
        p.tri_surf_valid_sz    =int(self.disp_tri_surface_val_sz_slider.value())

        f = m.f2d_params
        f.scharr_blur_size     = int(self.scharr_blur_size_slider.value())
        f.scharr_blur_sigma    = float(self.scharr_blur_sigma_slider.value())
        f.harris_block_size    = int(self.harris_block_size_slider.value())
        f.harris_k             = float(self.harris_k_slider.value())
        f.use_harris           = bool(self.harris_use_slider.value())
        f.harris_max_corners   = int(self.harris_max_corners_slider.value())
        f.harris_quality_level = float(self.harris_quality_slider.value())
        f.harris_min_dist      = int(self.harris_min_dist_slider.value())
        f.superpixel_size      = int(self.slic_superpixel_size_slider.value())
        f.compactness          = float(self.slic_compactness_slider.value())
        f.test_do_opt          = bool(self.slic_test_opt_slider.value())
        f.test_set_label       = bool(self.slic_test_set_lbl_slider.value())
        f.test_d               = float(self.slic_test_d_slider.value())
        f.test_inc             = float(self.slic_test_inc_slider.value())
        f.test_azi             = float(self.slic_test_azi_slider.value())
        
        m.disp_params = p
        m.f2d_params= f
        
        if m != self.pblock.models[cam_num]:
            self.pblock.models[cam_num] = m
            self.model_changed_timer_.start(self.signal_ms)
        
    # --------------------------------------------------------- Model to Widgets

    def model_to_widgets(self):

        cam_num = self.selected_camera()
        if cam_num < 0 or cam_num >= self.pblock.models.size():
            return

        # Video parameters
        model = self.pblock.models[cam_num]
        m = model

        self.uimagesz_w_slider.set_value(m.width)
        self.uimagesz_h_slider.set_value(m.height)
        self.uimagesz_f_slider.set_value(m.focal_length)
        self.uimagesz_ppt_dx_slider.set_value(m.ppt.x - m.width * 0.5)
        self.uimagesz_ppt_dy_slider.set_value(m.ppt.y - m.height * 0.5)
        self.uimagesz_use_calib_slider.set_value(m.use_calib_roi)
        self.uimagesz_software_equalize.set_value(m.software_equalize)
        
        # Disparity method
        p = model.disp_params
        self.disp_presmooth_sigma_slider.set_value(p.presmooth_sigma)
        self.disp_method_combo.setCurrentIndex(p.method)
        
        self.disp_SAD_window_size_slider.set_value(p.bm_SAD_window_size)
        self.disp_12_max_diff_slider.set_value(p.bm_max_diff_12)
        self.disp_min_disparity_slider.set_value(p.bm_min_disparity)
        self.disp_num_disparities_slider.set_value(p.bm_num_disparities)
        self.disp_spek_window_size_slider.set_value(p.bm_speckle_window_size)
        self.disp_spek_range_slider.set_value(p.bm_speckle_range)
        self.disp_prefilter_cap_slider.set_value(p.bm_prefilter_cap)
        self.disp_prefilter_size_slider.set_value(p.bm_prefilter_size)
        self.disp_prefilter_type_slider.set_value(p.bm_prefilter_type)
        self.disp_block_size_slider.set_value(p.bm_block_size)
        self.disp_texture_threshold_slider.set_value(p.bm_texture_threshold)
        self.disp_unique_ratio_slider.set_value(p.bm_uniqueness_ratio)
        
        self.disp_sg_SAD_window_size_slider.set_value(p.sg_SAD_window_size)
        self.disp_sg_12_max_diff_slider.set_value(p.sg_max_diff_12)
        self.disp_sg_min_disparity_slider.set_value(p.sg_min_disparity)
        self.disp_sg_num_disparities_slider.set_value(p.sg_num_disparities)
        self.disp_sg_spek_window_size_slider.set_value(p.sg_speckle_window_size)
        self.disp_sg_spek_range_slider.set_value(p.sg_speckle_range)
        self.disp_sg_prefilter_cap_slider.set_value(p.sg_prefilter_cap)
        self.disp_sg_unique_ratio_slider.set_value(p.sg_uniqueness_ratio)
        self.disp_sg_mode_slider.set_value(p.sg_mode)
        self.disp_sg_p1_slider.set_value(p.sg_p1)
        self.disp_sg_p2_slider.set_value(p.sg_p2)

        self.disp_apply_wls_slider.set_value(p.apply_wls)
        self.disp_wls_lambda_slider.set_value(p.wls_lambda)
        self.disp_wls_sigma_slider.set_value(p.wls_sigma)
        self.disp_wls_lrc_thres_slider.set_value(p.wls_lrc_thres)
        self.disp_wls_discont_radius_slider.set_value(p.wls_discont_radius)
        
        self.disp_tri_lowpass_slider.set_value(p.tri_lowpass)         
        self.disp_tri_subpixel_slider.set_value(p.tri_subpixel)        
        self.disp_tri_subpixel_val_slider.set_value(p.tri_subpixel_valid)    
        self.disp_tri_edge_correl_slider.set_value(p.tri_edge_correlation)     
        self.disp_tri_min_disp_slider.set_value(p.tri_min_disp)        
        self.disp_tri_max_disp_slider.set_value(p.tri_max_disp)        
        self.disp_tri_edge_mask_slider.set_value(p.tri_edge_mask)       
        self.disp_tri_stereo_mask_slider.set_value(p.tri_stereo_mask)     
        self.disp_tri_texture_val_slider.set_value(p.tri_text_valid)     
        self.disp_tri_surface_val_slider.set_value(p.tri_surf_valid)     
        self.disp_tri_surface_val_sz_slider.set_value(p.tri_surf_valid_sz)

        # Features2d
        f = model.f2d_params
        self.scharr_blur_size_slider.set_value(f.scharr_blur_size)
        self.scharr_blur_sigma_slider.set_value(f.scharr_blur_sigma)
        self.harris_block_size_slider.set_value(f.harris_block_size)
        self.harris_k_slider.set_value(f.harris_k)
        self.harris_use_slider.set_value(f.use_harris)
        self.harris_max_corners_slider.set_value(f.harris_max_corners)
        self.harris_quality_slider.set_value(f.harris_quality_level)
        self.harris_min_dist_slider.set_value(f.harris_min_dist)
        self.slic_superpixel_size_slider.set_value(f.superpixel_size)
        self.slic_compactness_slider.set_value(f.compactness)
        self.slic_test_opt_slider.set_value(f.test_do_opt)
        self.slic_test_set_lbl_slider.set_value(f.test_set_label)
        self.slic_test_d_slider.set_value(f.test_d)
        self.slic_test_inc_slider.set_value(f.test_inc)
        self.slic_test_azi_slider.set_value(f.test_azi)
        
        self.show_hide_widgets()

    # -------------------------------------------------------

    def gl_opts_camera_sel_(self, index):
        self.gl_opts_to_widgets()
    
    # ------------------------------------------------------- GL opts to Widgets
    
    def gl_opts_to_widgets(self):

        o = self.pblock.gl_render_opts
        
        self.gl_cb_rotate           .setChecked(o.do_rotate)
        self.gl_cb_image_colors     .setChecked(o.use_image_colors)
        self.gl_cb_draw_axis        .setChecked(o.draw_axis)
        self.gl_cb_draw_grid_xy     .setChecked(o.draw_grid_xy)
        self.gl_cb_draw_grid_xz     .setChecked(o.draw_grid_xz)
        self.gl_cb_draw_grid_yz     .setChecked(o.draw_grid_yz)
        self.gl_cb_draw_cad_model   .setChecked(o.draw_cad_model)
        self.gl_cb_draw_aruco_sensor.setChecked(o.draw_arucos)
        self.gl_cb_draw_aruco_cam   .setChecked(o.draw_aruco_cams)
        self.gl_cb_draw_final_cam   .setChecked(o.draw_final_cams)
        self.gl_cb_pt_cloud_final   .setChecked(o.pt_cloud_final)

        
        # Set all checkboxes to False
        for cam_num in range(self.pblock.argz.scene_desc.n_cameras()):
            for cb in self.gl_cbs_view[cam_num]:
                cb.setChecked(False)

        # Conditionally set some to True
        for X in o.cam_pts_cloud_ind:
            cam_num = X.x
            pt_cloud = X.y
            if cam_num >= 0 and cam_num < len(self.gl_cbs_view):
                if pt_cloud >= 0 and pt_cloud < len(self.gl_cbs_view[cam_num]):
                    self.gl_cbs_view[cam_num][pt_cloud].setChecked(True)

        simt_index = self.gl_select_cam.currentIndex()

        if simt_index == 0:
            self.gl_simt.set_euclidean_transform(o.gl_transform)
        else:
            while o.cam_corrections.size() < simt_index:
                o.cam_corrections.push_back(EuclideanTransform())
            disp_ind = simt_index - 1
            self.gl_simt.set_euclidean_transform(o.cam_corrections[disp_ind])
                    
        self.gl_simt.model_to_widgets()
        
    # ------------------------------------------------------- GL Widgets to Opts

    def gl_widgets_to_opts(self):

        o = copy.copy(self.pblock.gl_render_opts)

        o.do_rotate         = self.gl_cb_rotate.isChecked()
        o.use_image_colors  = self.gl_cb_image_colors.isChecked()
        o.draw_axis         = self.gl_cb_draw_axis.isChecked()
        o.draw_grid_xy      = self.gl_cb_draw_grid_xy.isChecked()
        o.draw_grid_xz      = self.gl_cb_draw_grid_xz.isChecked()
        o.draw_grid_yz      = self.gl_cb_draw_grid_yz.isChecked()
        o.draw_cad_model    = self.gl_cb_draw_cad_model.isChecked()
        o.draw_arucos       = self.gl_cb_draw_aruco_sensor.isChecked()
        o.draw_aruco_cams   = self.gl_cb_draw_aruco_cam.isChecked()        
        o.draw_final_cams   = self.gl_cb_draw_final_cam.isChecked()
        o.pt_cloud_final    = self.gl_cb_pt_cloud_final.isChecked()
        
        o.cam_pts_cloud_ind.clear()
        n_cameras = self.pblock.argz.scene_desc.n_cameras()
        for cam_num in range(n_cameras):
            for i in range(TweakerResults.n_disparities()+2):
                if self.gl_cbs_view[cam_num][i].isChecked():
                    o.cam_pts_cloud_ind.push_back(Point2(cam_num, i))
                    
        self.gl_simt.widgets_to_model()
        
        simt_index = self.gl_select_cam.currentIndex()
        if simt_index == 0:
            o.gl_transform        = self.gl_simt.euclidean_transform
        else:
            while o.cam_corrections.size() < simt_index:
                o.cam_corrections.push_back(EuclideanTransform())
            disp_ind = simt_index - 1
            o.cam_corrections[disp_ind] = self.gl_simt.euclidean_transform
        
        self.pblock.gl_render_opts = o

