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
import json

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

# ---------------------------------------------------------- remove version spec
# Remove version numbers from bcams (if present)
def remove_version_spec(key):
    pos = key.find('_')
    return key if pos == -1 else key[0:pos]

# ------------------------------------------------------------------ list videos
#
def list_videos(guapi, camera_id):
    video_set = [v for v in guapi.list(
        'processing', 'video',
        camera=camera_id,
        type='stereo')]
    return video_set

# ------------------------------------------------------------------------ store
#
def handle_store(guapi, kwargs):
    # Extract query options
    kwargs = {}
    if len(args.store_id) > 0:
        kwargs['id'] = int(args.store_id)
    
    if args.query:
        # vals = list(guapi.list('retail', 'store'))
        vals = list(guapi.list('retail', 'store', **kwargs))
        print(json.dumps(vals))
        exit(0)   # that is, success
    else:
        eprint('invalid action: "{}"'.format(action))
        
# ------------------------------------------------------------------------ scene
#
# 
def handle_scene(guapi, scene):

    
    # Extract query options
    videos = [list_videos(guapi, bcam) for bcam in scene['cameras']]
    print(json.dump(videos[0]))
    return 0
    kwargs = {}
    if len(args.store_id) > 0:
        kwargs['retail_store'] = int(args.store_id)
    if len(args.scene_name) > 0:
        kwargs['name'] = args.scene_name
        
    if args.query:
        vals = list(guapi.list('hardware', 'scene', **kwargs))                    
        def augment_scene_with_in_use(scene):
            # if next(guapi.list('processing', 'video', page_size=1, ...), None):
            scene['in_use'] = False
            return scene            
        vals = [augment_scene_with_in_use(x) for x in vals]
        print(json.dumps(vals))
        exit(0)   # that is, success
    else:
        eprint('invalid action: "{}"'.format(action))

# ----------------------------------------------------------------------- indent
#
def indent(n_spaces, text):
    return re.sub('^',' ' * n_spaces, text, flags=re.MULTILINE)

# ----------------------------------------------------------- json-encode-videos
#
def json_encode_video(bcam_key, video):    
    return """{{
   "camera": {},
   "file": {},
   "timestamp": {},
   "duration": {}
}}""".format(json.dumps(bcam_key),
             json.dumps(video['s3_key']),
             json.dumps(video['timestamp']),
             json.dumps(float(video['duration'])))
    
# ------------------------------------------------------------------------- main
#
if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        formatter_class=NullTextHelpFormatter,
        usage=argparse.SUPPRESS,
        description="""

Usage: {0} command <OPTIONS>

   

   Examples:

      # List all of the stores
      > {0} --query -t store
 
      # Query a store with id '2'
      > {0} --query -t store --store-id 2

      # Query all scenes for a given store
      > {0} --query -t scene --store-id 1

      # Query a scene with a specific key (returns if scene is "in use")
      > {0} --query -t scene --scene-name entrance_office

      # Create a new scene with values, including cameras

      # Update/delete a scene, but only if it is not "in use"


      

""".format(os.path.basename(sys.argv[0])))
    
    parser.add_argument('-q', '--query', action='store_true')
    parser.add_argument('-t', '--table', action='store', type=str, default='')
    parser.add_argument('--store-id', action='store', type=str, default='')
    parser.add_argument('--scene-name', action='store', type=str, default='')

    # Parse arguments
    args = parser.parse_args()
    table = args.table

    # Connect to GUAPI
    guapi = guapi_connect()

    scenes = [s for s in guapi.list('hardware', 'scene')]

    
    if len(scenes) == 0:
        eprint("failed to find scene with name: '{}'".format(scene))
        exit(1)
    print(scenes[4])

    handle_scene(guapi, scenes[4])
    exit(0)
    
    
    if table == 'store':
        handle_store(guapi, args)
    elif table == 'scene':
        handle_scene(guapi, args)
    else:
        eprint('invalid table: "{}"'.format(table))
    
    exit(0)
    
