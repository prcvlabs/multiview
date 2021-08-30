
import json
import pdb
import gc
import os
import copy

import PyQt5

from PyQt5.QtCore import QObject
from multiview import *

# -- Load Video File --
def load_video_file(in_filename, start_frame = 0, end_frame = -1):
    video = None

    # Try figure out where that video file is located
    print_info("Loading video file '{0}'".format(in_filename))
    filename = in_filename
    if not os.path.isfile(filename):
        filename = perceive_data_filename(filename)
    if not os.path.isfile(filename):
        print_err("Could not locate video file '{0}'".format(in_filename))
        return None
        
    try:
        video = VideoFileBuffer(filename, start_frame, end_frame)
    except RuntimeError:
        video = None
        print_err("Failed to read video file '{0}'", filename)

    if not video is None:
        if len(video) == 0:
            print_warn("Video file '{0}' does not have frame number information"
                       " in its header... the format may not support it. As "
                       "such the GUI will load in the first 100 frames, and "
                       "that's all you get to work with. The full video can be "
                       "loaded by converting it to an mp4 file as follows: \n\n"
                       "   > ffmpeg -i raw.h264 -v:c copy nice.mp4\n"
                       "".format(filename))
            video.get(99)
            print_info('Video length set to {0}'.format(len(video)))

    return video
    
# ------------------------------------------------------------------------------
# --                               PBlock                                     --
# ------------------------------------------------------------------------------

class PBlock(QObject):
    """
    Application parameter block for "Camera-Tweaker"
    """
    
    # WARNING, this signal must be connected as a QueuedConnection
    resultsChangedMustConnectQueued = PyQt5.QtCore.pyqtSignal()

    # --------------------------------------------------------------------------
    # --                 Construction and Initialization                      --
    # --------------------------------------------------------------------------

    # -------------------------------------------------------------- Constructor
    def __init__(self, argz, settings):
        super().__init__()
        self.argz = argz          # The command-line arguments
        self.app_settings = settings
        self.gl_render_opts = GLRenderOptions()

        if hasattr(settings, 'opengl_render_opts'):
            try:
                opts = settings.opengl_render_opts
                self.gl_render_opts.read_from_string('{}'.format(opts))
            except RuntimeError as e:
                print_err('error loading gl-render-options: {}'.format(e))
                self.gl_render_opts = GLRenderOptions()
            except SystemError as e:
                print_err('error loading gl-render-options: {}'.format(e))
                self.gl_render_opts = GLRenderOptions()
        
        self.clear()
        self.load_files_()
    
    # ------------------------------------------------------------- Reload Files
    def load_files_(self):
        """Load"""

        self.clear()
        has_error = False

        try:
            # Set the number of cameras
            n_cameras = self.argz.scene_desc.n_cameras()
            self.models.resize(n_cameras)
            self.calib_plane_sets.resize(n_cameras)
            
            # Load (initialize) tweaker-parameters for each camera
            for cam_num in range(0, n_cameras):
                camera_id = self.argz.scene_desc.camera_id(cam_num)
                key = 'tweaker_{}'.format(camera_id)
                if hasattr(self.app_settings, key):
                    s = getattr(self.app_settings, key)
                    self.models[cam_num].read_from_string(s)
            
            # Load in the CAD model
            data_source = self.argz.data_source
            
        except RuntimeError as e:
            has_error = True
            print_err('{}'.format(e))
        
        if has_error:            
            self.clear()            
    
    # ------------------------------------------------------------ Clear Cameras
    def clear_cameras(self):
        """Clears (resets) the parameter-block state"""

        self.loaded = False

        # Parameters associated with cameras
        self.cameras = []
        self.camera_params = []
        self.camera_param_filenames = []  
        self.is_dual = []                 # Cameras are either dual or single
        self.videos = []                  
        
        # This is the currently selected camera
        self._camera_id = 0               # Could be out of range
        self._current_frame_no = 0        # Could be out of range        

        # We may have nuked a whole bunch of memory
        gc.collect()                      
    
    # -------------------------------------------------------------------- Clear
    def clear(self):
        """Clears (resets) the parameter-block state"""

        self.clear_cameras()

        # Then we need a parameter file that links together the
        # entire camera system
        self.models  = TweakerParamsVector()
        self.results = TweakerResults()
        self.calib_plane_sets = CalibPlaneSetVector()
        self.cpanel = None

    # --------------------------------------------------------------- Set Cpanel
    def set_cpanel(self, cpanel):
        self.cpanel = cpanel    
        
    # --------------------------------------------------------------------------
    # --                            Getters / Setters                         --
    # --------------------------------------------------------------------------
    # Cameras
    def n_cameras(self):
        return self.argz.scene_desc.n_cameras()    
        
    def has_current_camera(self):
        return not self.cpanel is None \
            and self.current_camera_ind() >= 0 \
            and self.current_camera_ind() < self.n_cameras()
    
    def current_camera(self):
        return self.cameras[self.current_camera_ind()] \
            if self.has_current_camera() else None

    def current_camera_ind(self):
        return self.cpanel.select_image.currentIndex() \
            if not self.cpanel is None else -1
    
    # Camera Name
    def current_camera_name(self):
        return self.current_camera_params().camera_id \
            if self.has_current_camera() else None

    # Camera Parameters
    def current_camera_params(self):
        return self.camera_params[self.current_camera_ind()] \
            if self.has_current_camera() else None

    # Is Dual
    def current_camera_is_dual(self):
        return self.is_dual[self.current_camera_ind()] \
            if self.has_current_camera() else False
    
    # Video
    def has_current_video(self): return not self.current_video() is None    
    def current_video(self):
        return self.videos[self.current_camera_ind()] \
            if self.has_current_camera() else None

    # Frame (of video)
    def has_current_frame_no(self):
        return self.argz.scene_desc.has_current_frame()
    
    def set_current_frame_no(self, frame_no):
        self.argz.scene_desc.seek_frame(frame_no)

    def current_frame_no(self):
        return self.argz.scene_desc.current_frame_no()

    def n_frames(self):
        return self.argz.scene_desc.n_frames()

    def current_plane_set(self):
        cam_ind = self.current_camera_ind()
        if cam_ind is None \
           or cam_ind < 0 or cam_ind >= self.calib_plane_sets.size():
            return None
        return self.calib_plane_sets[cam_ind]

    def current_calib_plane(self):
        cps = self.current_plane_set()
        if cps is None:
            return None
        ind = cps.selected_index
        if ind < 0 and ind >= cps.p3s.size():
            return None
        return cps.p3s[ind]

    # --------------------------------------------------------------------------
    # --                             Managing Cameras                         --
    # --------------------------------------------------------------------------

    # -------------------------------------------------------- Attach New Camera
    def attach_new_camera(self, cam_parameters = None):
        """Attached (appends) a new camera in the camera system."""
        self.cameras.append(None)
        self.camera_params.append(None)
        self.is_dual.append(None)
        self.videos.append(None)
        self.set_camera(self.n_cameras() - 1, cam_parameters)
        return self.n_cameras() - 1

    # --------------------------------------------------------------- Set Camera
    def set_camera(self, camera_id, cam_parameters):
        """Sets (or resets) a specified camera."""  
        if camera_id < 0 or camera_id >= len(self.videos):
            print_err('Index out of range when attaching video feed: {0} >= {1}'
                      ''.format(camera_id, len(self.videos)))
            return

        self.cameras[camera_id]                = None
        self.camera_params[camera_id]          = cam_parameters
        self.is_dual[camera_id]                = True

        return self.n_cameras() - 1

    # ----------------------------------------------------------- Set Video Feed
    def set_video_feed(self, camera_id, video_buffer):
        """Sets (or resets) the video feed of the specified camera."""
        if camera_id < 0 or camera_id >= len(self.videos):
            print_err('Index out of range when attaching video feed: {0} >= {1}'
                      ''.format(camera_id, len(self.videos)))
        elif video_buffer is None:
            self.videos[camera_id] = video_buffer
            gc.collect() # Immediately free RAM
        elif not isinstance(video_buffer, VideoFileBuffer):
            print_err('Must pass a VideoFileBuffer object '
                      'when attaching a video feed')
        else:
            self.videos[camera_id] = video_buffer
            gc.collect() # Immediately free RAM
                
    # --------------------------------------------------------------------------
    # --                         Managing Execution                           --
    # --------------------------------------------------------------------------

    def execute_threaded_callback(self, val):
        self.results = val                          # Save the results        
        self.resultsChangedMustConnectQueued.emit() # Tell anyone whos listen
        
    def execute_threaded(self):

        execute_threaded_with_callback(self.models, # parameters
                                       self.argz.scene_desc,
                                       self.argz.stats,
                                       self.execute_threaded_callback)

