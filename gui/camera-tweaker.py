#!/usr/bin/python3

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

from PyQt5.QtWidgets import QApplication

os.environ['MULTIVIEW_USE_GL'] = 'true'

# Our module level imports
sys.path.append('./..' if os.path.dirname(__file__) == ''
                else os.path.dirname(__file__) + '/..' )
import multiview as mv
from widgets.main_window import MainWindow

DATA_SOURCE_DEFAULT='PERCEIVE_DATA'

# ------------------------------------------------------------------------- Main

if __name__ == '__main__':

    # ---------------------------------- #
    # -- Parse Command-line Arguments -- #
    # ---------------------------------- #
    parser = argparse.ArgumentParser(
        formatter_class=mv.NullTextHelpFormatter,
        usage=argparse.SUPPRESS,
        description="""

Usage: {0} [Options...] <manifest-file>

  Parameters:

    <manifest-file>       A JSON file that looks like the following:

        {{
            "scene-key":  "anvil_acuco-tests_v1.json",
            "videos":     ["video-path-1.mp4",         
                           "video-path-2.mp4"        ],
            "timestamps": ["20180410t13-23-49.546531", 
                           "20180410t13-23-47.502151"]
        }}

  Options:

    --data-source S3|PERCEIVE_DATA
                          Sets the data source for reading scene-info, 
                          bino-cam-info, etc. Default is {1}.

    --gui-defaults        Restore default application geometry. (Application 
                          geometry is usually preserved between separate 
                          executions of the GUI.)
    --start <int>         Video start frame number. Note the frames are 
                          '1-indexed'. Default is 0. (i.e. no effect.)
    --end <int>           Video end frame number. -1 for the full video. 
                          Default is -1.
    --stats <filename>    Stats file to load.

        """.format(os.path.basename(sys.argv[0]),
                   DATA_SOURCE_DEFAULT))

    parser.add_argument('manifest_filename', type=str)
    parser.add_argument('--data-source', type=str, default=DATA_SOURCE_DEFAULT)
    parser.add_argument('--gui-defaults', action='store_true')
    parser.add_argument('--start', action='store', type=int, default=0)
    parser.add_argument('--end', action='store', type=int, default=-1)
    parser.add_argument('--stats', type=str, default='')
    
    argz = parser.parse_args()

    # -- Run sanity checks
    has_error = False

    if argz.data_source == 'S3':
        argz.data_source = mv.DataSource.S3
    elif argz.data_source == 'PERCEIVE_DATA':
        argz.data_source = mv.DataSource.PERCEIVE_DATA
    else:
        mv.print_err("Invalid data-source given: {} not in "
                     "{{'S3', 'PERCEIVE_DATA'}}"
                     .format(argz.data_source))
        has_error = True

    def try_filename_w_dir(dirname, filename):

        if dirname == '':
            if os.path.isfile(filename):
                return filename
            out_filename = mv.perceive_data_filename(filename)
            if os.path.isfile(out_filename):
                return out_filename
        else:
            out_filename = '{}/{}'.format(dirname, filename)
            if os.path.isfile(out_filename):
                return out_filename
            out_filename = mv.perceive_data_filename(out_filename)
            if os.path.isfile(out_filename):
                return out_filename

        mv.print_err("File not found: '{0}'".format(filename))
        has_error = True
        return ''
            
    def try_filename(filename):
        return try_filename_w_dir('', filename)

    # -- Load the manifest file
    argz.manifest_filename = try_filename(argz.manifest_filename)
    if not has_error:
        argz.scene_desc = mv.make_scene_description(argz.manifest_filename,
                                                    argz.data_source)
        mv.print_info('loaded manifest file {}'.format(argz.manifest_filename))

        stats_fname = argz.stats
        argz.stats = mv.load_stats_file(stats_fname, argz.data_source)
        if not argz.stats is None:
            mv.print_info('loaded stats file {}'.format(stats_fname))
        
        print('{}'.format(argz.scene_desc))
            
    # -- Abort on error
    if has_error:
        exit(1)
        
    # ---------------------------------- #
    # --          Run the GUI         -- #
    # ---------------------------------- #
    app = QApplication(sys.argv)
    ex = MainWindow(argz)
    sys.exit(app.exec_())

