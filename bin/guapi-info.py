#!/usr/bin/python3

import errno
import boto3
import botocore.exceptions 
import sys
import os
import cv2
import json
import pdb
import argparse
import gc
import re
import multiprocessing
import numpy as np
import math
import functools
import random
import subprocess

from datetime import datetime
from datetime import timedelta

AWS_REGION = 'us-east-1'

# ----------------------------------------------------------------------- eprint
#
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

# ------------------------------------------------------------ load guapi-client
#
if 'GUAPI_CODE' in os.environ:
    sys.path.append(os.environ['GUAPI_CODE'])
import guapi_client

def guapi_connect():
    if not 'GUAPI_USER' in os.environ:
        ssm = boto3.client('ssm', region_name=AWS_REGION)
        result = ssm.get_parameter(Name='multiview-worker')
        config_json = result['Parameter']['Value']
        config = json.loads(config_json)
        return guapi_client.Connection(
            user=config['guapi_user'],
            password=config['guapi_password'],
        )
    return guapi_client.Connection()

# ----------------------------------------------------- null-text-help-formatter
# Formatter that doesn't mangle white space in the description
class NullTextHelpFormatter(argparse.RawDescriptionHelpFormatter):
    def add_argument(self, action):
        pass

# ----------------------------------------------------------------------- indent
#
def indent(n_spaces, text):
    return re.sub('^',' ' * n_spaces, text, flags=re.MULTILINE)
    
# ------------------------------------------------------------------------- main
#
if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        formatter_class=NullTextHelpFormatter,
        usage=argparse.SUPPRESS,
        description="""

Usage: {0}

      Outputs a list of known scenes, with start/end dates

""".format(os.path.basename(sys.argv[0])))
        
    # Parse arguments
    args = parser.parse_args()
    
    # Connect to GUAPI
    guapi = guapi_connect()

    # Load the scenes
    scenes = [s for s in guapi.list('hardware', 'scene')]

    for s in scenes:
        name = s['name']
        start = s['valid_starting']
        end =s['valid_ending']
        print('{:67s} | {} | {}'.format(name, start, end))
                               

