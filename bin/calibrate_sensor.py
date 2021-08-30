#!/usr/bin/env python3

from argparse import ArgumentParser, ArgumentTypeError
import boto3
import collections
import re
from lib.multiview_runner import Multiview
import os
import sys
import hashlib
import glob
import shutil
from lib.calibration import *

def sensor_arg(name):
    match = re.match(r'(STR\d{5,})_v(\d+)$', name)
    if not match:
        raise ArgumentTypeError(
            "Invalid sensor name (must be of form STRXXXXX_vX)")

    keys = s3_list(CALIB_BUCKET, '{0}/sensors/{1}_'.format(CALIB_BASE, name))
    if 0 != len(keys):
        raise ArgumentTypeError(
            "Sensor version {0} already exists: {1}.".format(name, keys))

    sensor, version = match.groups()
    return (name, sensor, version)

def main():
    parser = ArgumentParser()
    parser.add_argument('--sensor', '-s', type=sensor_arg, required=True,
                        help='name of sensor, e.g. STR00122_v1.')
    parser.add_argument('--local', action='store_true',
                        help='Run local version of multiview, rather than '
                        'docker container.')
    args = parser.parse_args()
    
    sensor_id, sensor, version = args.sensor

    sensor_dir = '/tmp/{}'.format(sensor_id)
    output_dir = '{}/output'.format(sensor_dir)
    os.makedirs(output_dir, exist_ok=True)
    os.chmod(output_dir, 0o777)

    multiview = Multiview()
    multiview.set_env('MULTIVIEW_ASSET_S3_BUCKET', 's3://' + CALIB_BUCKET)
    multiview.entry_point = 'bin/distortion-calibration.sh'
    
    if args.local:
        multiview.docker = False
    else:
        multiview.pull_docker_image()
    
    multiview.map_volume(sensor_dir)
    multiview.run('bin/distortion-calibration.sh', [
        '-d', output_dir,
        '-s', sensor,
        '-v', version,
        '-y',
    ])

    outfile = os.path.join(output_dir, '{}.json'.format(sensor_id))

    # upload output directory to s3
    output_prefix = '{base}/sensors/{sensor}/'.format(
        base=CALIB_OUTPUT_BASE, sensor=sensor_id)
    s3_upload_dir(CALIB_BUCKET, output_prefix, output_dir)

if '__main__' == __name__:
    main()
