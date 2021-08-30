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
from lib.calibration import *

def camera_image_uris(camera_name):
    image_prefix = '{}/cameras/{}/images/'.format(
        CALIB_DATA_BASE, camera_name)
    image_keys = s3_list(CALIB_BUCKET, image_prefix)
    image_uris = ['s3://{}/{}'.format(CALIB_BUCKET, key) for key in image_keys]
    
    if 0 == len(image_uris):
        print("Can't find images in s3://{}/{}".format(
            CALIB_BUCKET, image_prefix), file=sys.stderr)
        exit(1)
        
    return image_uris

def sensor_arg(name):
    match = re.match(r'STR\d{5,}_v\d+$', name)
    if not match:
        raise ArgumentTypeError(
            "Invalid sensor name (must be of form STRXXXXX_vX)")

    keys = s3_list(CALIB_BUCKET, '{0}/sensors/{1}_'.format(CALIB_BASE, name))
    if 0 == len(keys):
        raise ArgumentTypeError("Sensor {0} not found".format(name))
    elif 1 < len(keys):
        raise ArgumentTypeError(
            "Multiple calibrations found for {0}: {1}.".format(name, keys))
    filename = keys[0].split('/')[-1]
    return re.sub(r'\.json$', '', filename)

def camera_arg(name):
    match = re.match(r'C\d{7,}_v\d+$', name)
    if not match:
        raise ArgumentTypeError(
            "Invalid camera name (must be of form CXXXXXXX_vX)")
    keys = s3_list(CALIB_BUCKET, '{0}/binocular/{1}_'.format(CALIB_BASE, name))
    if 0 != len(keys):
        raise ArgumentTypeError(
            'Camera {0} already exists: {1}'.format(name, keys))
    return name

def main():
    parser = ArgumentParser()
    parser.add_argument('--left-sensor', '-l', type=sensor_arg, required=True,
                        help='name of left sensor, e.g. STR00122_v1.')
    parser.add_argument('--right-sensor', '-r', type=sensor_arg, required=True,
                        help='name of right sensor, e.g. STR00122_v1.')
    parser.add_argument('--camera', '-c', type=camera_arg, required=True,
                        help='name of camera, e.g. C0001024_v2.')
    parser.add_argument('--aruco-cube',
                        help='name of aruco cube to use.')
    parser.add_argument('--local', action='store_true',
                        help='Run local version of multiview, rather than '
                        'docker container.')
    args = parser.parse_args()

    output_dir = '/tmp/{}/output'.format(args.camera)
    os.makedirs(output_dir, exist_ok=True)
    os.chmod(output_dir, 0o777)

    image_paths = camera_image_uris(args.camera)

    multiview = Multiview()
    multiview.set_env('MULTIVIEW_ASSET_S3_BUCKET', 's3://' + CALIB_BUCKET)
    if args.local:
        multiview.docker = False
    else:
        multiview.pull_docker_image()
    multiview.map_volume(output_dir)
    
    cmd_args = [
        'no-cuda', 'release', 'phase_stereo_calib',
        '-d', output_dir,
        '--cam-id', args.camera,
        '-s0', args.left_sensor,
        '-s1', args.right_sensor,
    ]
    if args.aruco_cube:
        cmd_args.extend(['--aruco-cube', args.aruco_cube])
    cmd_args.extend(image_paths)
    
    multiview.run('multiview/multiview_cpp/run.sh', cmd_args)

    outfile = os.path.join(output_dir, '{}.json'.format(args.camera))

    # make hashed filename & upload calibration file
    output_key = '{}/binocular/{}'.format(
        CALIB_BASE, hashed_filename(outfile, args.camera))
    s3 = boto3.resource('s3')
    s3.Object(CALIB_BUCKET, output_key).upload_file(outfile)
    print('Uploaded calibration to s3://{}/{}'.format(CALIB_BUCKET, output_key))

    # upload output directory to s3
    output_prefix = '{base}/binocular/{camera}/'.format(
        base=CALIB_OUTPUT_BASE, camera=args.camera)
    s3_upload_dir(CALIB_BUCKET, output_prefix, output_dir)

if '__main__' == __name__:
    main()
