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

# ---------------------------------------------------------- remove version spec
# Remove version numbers from bcams (if present)
def remove_version_spec(key):
    pos = key.find('_')
    return key if pos == -1 else key[0:pos]

# ------------------------------------------------------------------- load-scene
#
def load_scene(guapi, scene):
    scenes = [s for s in guapi.list('hardware', 'scene', name=scene)]
    if len(scenes) == 0:
        eprint("failed to find scene with name: '{}'".format(scene))
        exit(1)
    return scenes[0]

# ------------------------------------------------------------------ list videos
#
def list_videos(guapi, camera_id, epoch):
    epoch_end = epoch + timedelta(0, 600)
    video_set = [v for v in guapi.list(
        'processing', 'video',
        camera=camera_id,
        type='stereo',
        timestamp__gte=epoch,
        timestamp__lt=epoch_end.strftime('%Y-%m-%dT%H:%M:%S.%f%z'))]

    if len(video_set) == 0:
        eprint('failed to find video for camera-id={}, epoch={}'
               .format(camera_id, epoch))
        exit(1)
    if len(video_set) != 1:
        eprint('expected precisely 1 video in epoch for a single store+camera')
        eprint(video_set)
    return video_set[0]

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
             json.dumps('{}{}'
                        .format('s3://perceive-stereo/', video['s3_key'])),
             json.dumps(video['timestamp']),
             json.dumps(float(video['duration'])))
    
# ------------------------------------------------------------------------- main
#
if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        formatter_class=NullTextHelpFormatter,
        usage=argparse.SUPPRESS,
        description="""

Usage: {0} [-d <outdir>] <scene> <epoch>

   Examples:

      # Print out the manifest for this scene+epoch
      > {0} glenbrook_2018-11-02_v1 2018-11-02T15:00:00

      # Set up directory 'data' with a manifest and videos, ready for pipeline
      > {0} -d data glenbrook_2018-11-02_v1 2018-11-02T15:00:00

""".format(os.path.basename(sys.argv[0])))
    
    parser.add_argument('scene', type=str, default='')
    parser.add_argument('epoch', type=str, default='')
    parser.add_argument('-d', '--outdir', action='store', type=str, default='')

    # Parse arguments
    args = parser.parse_args()
    outdir = args.outdir
    scene = args.scene
    epoch_str = args.epoch

    # Find the [begin, end) of the epoch
    epoch = datetime.strptime(epoch_str, '%Y-%m-%dT%H:%M:%S')
    
    # Connect to GUAPI
    guapi = guapi_connect()

    # Load the scene
    scene = load_scene(guapi, scene)
    store_id = scene['retail_store']
    bcam_keys = scene['cameras']

    # Load the videos
    videos = [list_videos(guapi, bcam, epoch) for bcam in bcam_keys]
    
    manifest_text = """
{{
   "store": {},
   "scene-key": {},
   "epoch": {},
   "videos": [
{}
   ]
}}
""".format(json.dumps(store_id),
           json.dumps(scene['name']),
           json.dumps(epoch_str),
           indent(6, ',\n'.join([json_encode_video(k, v)
                                 for k, v in zip(bcam_keys, videos)])))

    
    if outdir == '':    
        print(manifest_text)
        exit(0)

    if not os.path.isdir(outdir):
        os.mkdir(outdir)
    if not os.path.isdir(outdir):
        eprint("failed to locate or create outdir: '{}'".format(outdir))
        exit(1)

    with open('{}/manifest.json'.format(outdir), 'w') as f:
        f.write(manifest_text)

    exit(0)
    
    for video in videos:
        cmd = 'aws s3 cp s3://perceive-stereo/{} {}/'.format(video['s3_key'],
                                                             outdir)
        os.system(cmd)
                               
