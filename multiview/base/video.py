
import cv2
import pdb
import os

from base import *
from collections import deque
from datetime import datetime

# ------------------------------------------------------------------------------ Video File Iterator

class _VideoFileIterator(object):
    """Video file iterator object that automatically releases the video resource
    when iteration is done."""
     
    def __init__(self, raw_filename, frame_nos = None, start=0, count=-1):
        self.vid_file = None
        self.frame_nos = sorted(set(frame_nos)) if not frame_nos is None else None
        self.ind = 0
        self.start = max(0, start)
        self.frame_no = 0   # The next frame that will be read
        self.count = count
        self.end = self.start + self.count if count >= 0 else -1

        self.cap_width = 0
        self.cap_height = 0
        self.cap_fps = 0
        self.cap_length = 0
        
        filename = str(raw_filename)
        if not os.path.isfile(filename):
            filename = perceive_data_filename(filename)
            
        self.vid_file = cv2.VideoCapture(filename)
        if not self.vid_file.isOpened():
            self.vid_file = None
            raise RuntimeError("Failed to open video file '{0}'".format(filename))

        # 
        self.cap_length = int(self.vid_file.get(cv2.CAP_PROP_FRAME_COUNT))
        self.cap_width = int(self.vid_file.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.cap_height = int(self.vid_file.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.cap_fps = self.vid_file.get(cv2.CAP_PROP_FPS)

    def seek(self, frame_no):
        if self.frame_no != frame_no:
            # print_info("SEEKING {0}".format(frame_no))
            self.vid_file.set(cv2.CAP_PROP_POS_FRAMES, frame_no)
            self.frame_no = frame_no
        
    def __del__(self):
        self.release()

    def is_open(self):
        return not self.vid_file is None

    def __len__(self):
        if not self.frame_nos is None:
            return len(self.frame_nos)
        if self.count >= 0:
            return self.count
        if self.cap_length > 0:
            return self.cap_length
        return self.frame_no
    
    def release(self):
        if not self.vid_file is None:
            self.vid_file.release()
            self.vid_file = None
    
    def __enter__(self):
        pass

    def __exit__(self, type, value, traceback):
        self.release()
    
    def __iter__(self):
        return self

    def __next__(self):
        return self.next()

    def next(self):

        if self.vid_file is None:
            # User can call "release" at any time, and if they do
            # we want iteration to automatically stop
            raise StopIteration()

        # "-1" is an invalid value that will trigger StopIteration
        target_frame = -1

        if self.frame_nos is None:
            # Reading video frame by frame... target-frame is either self.start
            # or simply the next frame
            target_frame = max(self.start, self.frame_no)
        
        else:
            # Read from the set of frame-nos... so read the target-frame from the
            # list of frame-nos
            if self.ind >= len(self.frame_nos) or not self.vid_file.isOpened():
                target_frame = -1
            else:
                target_frame = self.frame_nos[self.ind]
                self.ind += 1

        # Call 'seek' if able to
        if self.frame_no > target_frame and self.end < 0 or target_frame<self.end:
            self.seek(target_frame)
                
        # Read until target frame
        while (self.frame_no <= target_frame) and (self.end < 0 or self.frame_no < self.end):
            ret, frame = self.vid_file.read()
            if ret:
                found_target = (self.frame_no == target_frame)
                self.frame_no += 1
                if found_target:
                    return frame, self.frame_no - 1
            else:
                break
            
        self.release()
        raise StopIteration()

# -------------------------------------------------------------------------------- Video File Buffer

class VideoFileBuffer(object):

    """A buffer for random-access to video frames. Frames
    are loaded on demand, and kept in memory. Don't expect
    miracles if you attempt to load a massive video."""

    def __init__(self, filename, start_frame = 0, end_frame = -1):

        self.filename_ = filename
        self.frame_nos_ = None
        self.reader_ = None
        self.vid_ = _VideoFileIterator(filename, None, start_frame, end_frame)
        self.buffer_ = {}  # {(frame-no, frame), ...}
        self.frame_list_ = deque()
        self.max_buffer_size_ = 100
        
    def n_frames(self):
        """Total number of frames in the buffer."""        
        return len(self.vid_)

    def __len__(self):
        return self.n_frames()

    def valid_frame_no(self, index):
        return index >= 0 and index < self.n_frames()

    def filename(self):
        return self.filename_
    
    def get(self, frame_no):
        """Gets the desired frame, must be >= 0."""
        
        if (frame_no < 0):
            raise IndexError("Index out of range: {0}".format(frame_no))

        # Do we have the frame in our buffer?
        if frame_no in self.buffer_:
            return self.buffer_[frame_no], frame_no

        # The video may have been released
        if not self.vid_.is_open():
            raise RuntimeError("Video file not open")

        # Okay, if we're seekable, let's seek the frame
        self.vid_.seek(frame_no)
            
        try:
            # Keep reading frames until we find the desired frame, or fail
            while self.vid_.frame_no <= frame_no:
                frame, fno = self.vid_.next()
                self.buffer_[fno] = frame
                self.frame_list_.append(fno)
                if len(self.buffer_) > self.max_buffer_size_:
                    self.buffer_.pop(self.frame_list_.popleft())

        except StopIteration:
            pass

        if frame_no >= self.vid_.frame_no:
            raise IndexError("Index out of range: {0}".format(frame_no))
               
        return self.buffer_[frame_no], frame_no

    def __getitem__(self, frame_no):
        return self.get(frame_no)

    def release(self):
        """Release resources in the underlying video-file-iterator object."""
        self.vid_.release()
        self.buffer_ = {}
    
# ------------------------------------------------------------------------ Read video frame-by-frame

def read_video(filename, start=0, end=-1):
    """
    Generator that reads a video file and returns frames (i.e., images) and
    0-indexed frame numbers. Reading starts at `start`, and continues to `end`.

    Example:

        # Export the entire video, writing the frame number of the images
        for image, frame_no in read_video_frames(filename):
            # Image is a cv image, frame_no is an integer
            font = cv2.FONT_HERSHEY_SIMPLEX
            msg = 'Frame #{0:06d}'.format(frame_no)
            cv2.putText(image, msg, (10, 40), font, 1.0, (0,0,255), 1, cv2.LINE_AA)
            cv2.imwrite('/tmp/frame_{0:06d}.png'.format(frame_no), image)

    """

    count = -1 if end <= start else end - start
    return _VideoFileIterator(filename, None, start, count)

# ------------------------------------------------------------------- Read a set of frame from video

def read_video_frames(filename, frame_nos):
    """Generator that reads a video file, and returns frames (i.e., images)
    and 0-indexed frame numbers. The specific frames returned are in the
    list `frame_nos`, which must be a list of integers. `frame_nos` does
    not need to be in order

    Example: 

        # Export frames 0, 4, and 5, writing the frame number of the images
        for image, frame_no in read_video_frames(filename, [0, 4, 5]):
            # ... do something with the image
            font = cv2.FONT_HERSHEY_SIMPLEX
            msg = 'Frame #{0:06d}'.format(frame_no)
            cv2.putText(image, msg, (10, 40), font, 1.0, (0,0,255), 1, cv2.LINE_AA)
            cv2.imwrite('/tmp/frame_{0:06d}.png'.format(frame_no), image)

    """

    return _VideoFileIterator(filename, frame_nos)
    

def perceive_video_get_info(video_name, use_api=False):
    """Parses the filename of a perceive video and returns the information 
    as a key value pair with python datatypes. Optionally reads the information 
    from the api (not implemented yet)"""
    filename = os.path.split(video_name)[1]
    video_str = os.path.splitext(filename)[0]
    i = video_str.split('_')
    info_dict = {'store': int(i[0]),
                 'video_id': int(i[1]),
                 'time_start': datetime.strptime(i[2], '%Y%m%dt%H-%M-%S.%f'),
                 'type': i[3],
                 'duration': int(i[4][:2]),
                 'uuid': i[5] }
    return info_dict

