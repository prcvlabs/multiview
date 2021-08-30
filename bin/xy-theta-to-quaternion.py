#!/usr/bin/python3

import errno
import sys
import os
import cv2
import json
import pdb
import argparse
import datetime
import gc
import multiprocessing
import numpy as np
import math
import functools
import random
import argparse

from scipy.spatial.transform import Rotation as R
 
class NullTextHelpFormatter(argparse.RawDescriptionHelpFormatter):
    """Text formatter for argparse help message that 
    (1) Doesn't mangle white space in the description, and
    (2) Doesn't print "helpful" optional and postional argument secionts.

    When using this text formatter, put all usage information in 
    the description field (of the argument parser), and format it
    however you want. Done."""
    def add_argument(self, action):
        pass
    
# ------------------------------------------------------------------------- main

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(
        formatter_class=NullTextHelpFormatter,
        usage=argparse.SUPPRESS,
        description="""

Usage: {0} <theta>

   Reads theta (in degrees) and outputs the quaternion that rotates
   by theta in the XY plane. (i.e., roates about the Z axis.)

""".format(os.path.basename(sys.argv[0]))
    )
    
    parser.add_argument('theta_degrees', type=float)
    args = parser.parse_args()

    theta = math.radians(args.theta_degrees)


    r = R.from_quat([0, 0, np.sin(0.5 * theta), np.cos(0.5 * theta)])

    X = [1, 0, 2]

    print("theta    = {} degrees".format(args.theta_degrees))
    print("let q    = {}".format(r.as_quat()))
    print("let X    = {}".format(X))
    print("...")
    print("q.rot(X) = {}".format(r.apply(X)))
        
    exit(0)
    

