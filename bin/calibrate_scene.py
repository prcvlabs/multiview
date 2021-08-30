#!/usr/bin/env python3

from argparse import ArgumentParser, ArgumentTypeError
import boto3
import collections
import re
from lib.multiview_runner import Multiview, MultiviewError
import os
import sys
import hashlib
import glob
import shutil
from lib.calibration import *

def valid_scene_filename(scene_name, filename):
    scene_filename_re = r'{scene}_{hash}.json$'.format(scene=scene_name,
                                                       hash='[0-9a-z]{32}')
    return re.match(scene_filename_re, filename)

Image = collections.namedtuple(
    'Image', ['camera', 'calibration', 'cube_position', 'uri', 'cube_name'])

def get_images(scene, overrides):
    s3 = boto3.resource('s3')
    prefix = CALIB_DATA_BASE + '/scenes/' + scene + '/images/'
    keys = s3_list(CALIB_BUCKET, prefix)
    # if s3 data specifies which aruco cube we're using, read it for each
    # aruco position.
    cubes = {}
    for key in keys:
        suffix = key[len(prefix):]
        parts = suffix.split('/')
        if len(parts) == 2:
            cube_pos, filename = parts
            if 'aruco-cube.txt' == filename:
                rawbytes = s3.Object(CALIB_BUCKET, key).get()['Body'].read()
                cubes[cube_pos] = rawbytes.decode('utf-8').strip()

    # get all the image files
    images = []
    for key in keys:
        suffix = key[len(prefix):]
        parts = suffix.split('/')
        if len(parts) == 2:
            cube_pos, filename = parts
            match = re.match(r'stereo-[Cc](\d+)-', filename)
            if match:
                cam = 'C' + match.group(1).rjust(7, '0')
                images.append(Image(
                    camera=cam,
                    calibration=get_camera_calibration(cam, overrides),
                    cube_position=cube_pos,
                    uri='s3://{}/{}'.format(CALIB_BUCKET, key),
                    cube_name=cubes.get(cube_pos)))

    return images

def locate(image, multiview, location_dir):
    '''Locate one image relative to the aruco cube'''
    filename = image.uri.split('/')[-1]
    ts = re.sub(r'^stereo-.*-(.*)\.jpg$', '\\1', filename)
    view_name = '{}_{}_{}'.format(image.camera, image.cube_position, ts)
    output_dir = os.path.join(location_dir, view_name)
    pos_file = os.path.join(output_dir, 'positions.json')

    os.makedirs(output_dir, exist_ok=True)
    args = [
        'release', 'no-cuda', 'phase_camera_extrinsic',
        '-d', output_dir,
        '--cam', image.calibration,
    ]
    if image.cube_name:
        args.extend(['--aruco-cube', image.cube_name])
    args.append(image.uri)
    try:
        multiview.run('multiview/multiview_cpp/run.sh', args)
    except MultiviewError:
        return None
    else:
        if not os.path.exists(pos_file):
            raise RuntimeError(
                'Error, position file not produced: {}'.format(pos_file))
        return pos_file

def make_scene(
        scene, image_positions, origin_cube_position, multiview, scene_dir):
    '''Given a list of positions, run the scene maker to create the scene'''
    cmd_args = [
        'release', 'no-cuda', 'scene_maker',
        '--scene-id', scene,
        '-d', scene_dir,
    ]

    ref_cam_found = False
    for (image, position_file) in image_positions:
        pos_arg = '--pos'
        if image.cube_position == origin_cube_position and not ref_cam_found:
            ref_cam_found = True
            pos_arg = '--pos*'
        cmd_args.extend([
            pos_arg,
            image.cube_position,
            image.calibration,
            position_file,
            image.uri])

    if not ref_cam_found:
        raise RuntimeError('No located image for origin cube position!')

    multiview.run('multiview/multiview_cpp/run.sh', cmd_args)

    outfiles = [fname for fname in os.listdir(scene_dir)
                if valid_scene_filename(scene, fname)]

    if 0 == len(outfiles):
        raise RuntimeError('No scene files found!')
    elif 1 < len(outfiles):
        raise RuntimeError(
            'Multiple scene files found in output directory: {}'.format(
                outfiles))

    return os.path.join(scene_dir, outfiles[0])

def camera_calibration_arg(name):
    match = re.match(r'C\d{7,}_v\d+$', name)
    if not match:
        raise ArgumentTypeError(
            "Invalid camera name (must be of form CXXXXXXX_vX)")
    
    prefix = '{0}/binocular/{1}_'.format(CALIB_BASE, name)
    keys = s3_list(CALIB_BUCKET, prefix)
    fullnames = [parse_camera_calibration(key)[0]
                 for key in keys if is_camera_calibration(key)]
    if 0 == len(fullnames):
        raise ArgumentTypeError(
            "Camera calibration {0} not found in s3://{0}/{1}".format(
                CALIB_BUCKET, prefix))
    elif 1 < len(fullnames):
        raise ArgumentTypeError(
            'Multiple camera calibrations for {0} (this should not happen): '
            '{1}'.format(name, keys))

    return fullnames[0]

def scene_arg(full_name):
    if not re.match(r'(.*)_v(\d+)$', full_name):
        raise ArgumentTypeError('Scene must be of format scene_name_vXX')
    source_scene = full_name
    dest_scene = source_scene + '-DEV'
    keys = s3_list(CALIB_BUCKET, '{0}/scenes/{1}_'.format(
        CALIB_BASE, dest_scene))
    for key in keys:
        fname = key.split('/')[-1]
        if valid_scene_filename(dest_scene, fname):
            raise ArgumentTypeError(
                "Scene {0} already exists: s3://{1}/{2}.".format(
                    dest_scene, CALIB_BUCKET, key))
    return dest_scene, source_scene

def main():
    parser = ArgumentParser()
    parser.add_argument(
        '--scene',
        required=True,
        type=scene_arg,
        help='Name of the scene to produce and upload to s3.')
    parser.add_argument(
        '--origin-cube-position',
        required=True,
        help='Named cube position that determines the origin of the scene.')
    parser.add_argument(
        '--source-scene',
        help='Name of scene used when uploading images. Only necessary if this '
        'name is different than the argument to --scene.')
    parser.add_argument(
        '--binocular-version',
        type=camera_calibration_arg,
        default=[],
        action='append',
        help='Version of calibration for a camera to use. This is only '
        'necessary if you want to use something other than the highest '
        'version that exists. E.g. C0001024v2.')
    parser.add_argument(
        '--local', action='store_true',
        help='Run local version of multiview, rather than docker container.')
    args = parser.parse_args()

    dest_scene, default_source_scene = args.scene
    source_scene = (default_source_scene
                    if None is args.source_scene
                    else args.source_scene)
        
    output_dir = '/tmp/scenes/{}'.format(dest_scene)
    scene_dir = '{}/scene-maker/'.format(output_dir)
    location_dir = '{}/locations'.format(output_dir)

    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(scene_dir, exist_ok=True)
    os.makedirs(location_dir, exist_ok=True)

    images = get_images(source_scene, args.binocular_version)
    if not images:
        print('No images found!', file=sys.stderr)
        exit(1)

    multiview = Multiview()
    multiview.set_env('MULTIVIEW_ASSET_S3_BUCKET', 's3://' + CALIB_BUCKET)
    if args.local:
        multiview.docker = False
    else:
        multiview.pull_docker_image()
    multiview.map_volume(output_dir)

    image_positions = []
    for image in images:
        location = locate(image, multiview, location_dir)
        if location:
            image_positions.append((image, location))

    print('Located images:')
    for image, position in image_positions:
        print('  ', image.uri, 'FAILED' if None is position else 'SUCCESS')

    outfile = make_scene(dest_scene,
                         image_positions,
                         args.origin_cube_position,
                         multiview,
                         scene_dir)

    # upload calibration file
    output_key = '{}/scenes/{}'.format(CALIB_BASE, os.path.basename(outfile))
    s3 = boto3.resource('s3')
    s3.Object(CALIB_BUCKET, output_key).upload_file(outfile)
    print('Uploaded calibration to s3://{}/{}'.format(CALIB_BUCKET, output_key))

    # upload output directory to s3
    output_prefix = '{base}/scenes/{scene}/'.format(
        base=CALIB_OUTPUT_BASE, scene=dest_scene)
    s3_upload_dir(CALIB_BUCKET, output_prefix, output_dir)

    fname = os.path.basename(outfile)
    scene_key = re.sub(r'.json$', '', fname)
    print('Produced scene: {}'.format(scene_key))
    
if '__main__' == __name__:
    main()
