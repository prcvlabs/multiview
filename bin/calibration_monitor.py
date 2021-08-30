#!/usr/bin/env python3

import argparse
import boto3
import subprocess as sp
import os
import sys
import botocore
import getpass
from datetime import datetime, timedelta, timezone
import time
import re
import functools
import json
import shutil
import threading
from lib.calibration import *
from lib.multiview_runner import Multiview
import collections
import logging

logger = logging.getLogger(__name__)

CHANNEL = '#calibration_bot'

def cam_serial(short_name):
    match = re.match(r'[Cc](\d+)$', short_name)
    if not match:
        raise RuntimeError('Invalid camera short name: {}'.format(short_name))
    return 'C' + match.group(1).rjust(7, '0')
    
def get_scene_dir(scene_name):
    return '/tmp/scenes/{}'.format(scene_name)

def slack_send(message, pos_prefix):
    import slacker

    images = [
        ('Left Sensor', 'ddd_detect_0-0.png'),
        ('Right Sensor', 'ddd_detect_0-1.png'),
    ]
            
    left_img = '{}/ddd_detect_0-0.png'.format(pos_prefix)
    right_img = '{}/ddd_detect_0-1.png'.format(pos_prefix)
    
    client = slacker.Slacker(os.environ['SLACK_BOT_TOKEN'])
    response = client.chat.post_message(channel=CHANNEL, text=message)

    for title, filename in images:
        path = os.path.join(pos_prefix, filename)
        if os.path.exists(path):
            logger.info('Sending image: {}'.format(path))
            with open(path, 'rb') as f:
                client.files.upload(
                    title=title,
                    file_=f,
                    channels=CHANNEL,
                    thread_ts=response.body['ts'])
        else:
            logger.info('image not found: {}'.format(path))

Image = collections.namedtuple(
    'Image', ['camera', 'calibration', 'cube_position', 'uri', 'cube_name'])

def locate_one_image(obj_summary, args):
    '''
    Download image, and locate camera relative to aruco cube. Send notification
    of success/failure via Slack
    '''

    s3 = boto3.resource('s3')
    
    multiview = Multiview()
    multiview.set_env('MULTIVIEW_ASSET_S3_BUCKET', 's3://{}'.format(
        CALIB_BUCKET))
    if args.local:
        multiview.docker = False
    else:
        multiview.pull_docker_image()
    multiview.map_volume('/tmp/scenes')
    multiview.capture_output = True
    multiview.raise_on_error = False

    try:
        scene_name, _, cube_position, filename = obj_summary.key.split('/')[-4:]
        cam_name = cam_serial(filename.split('-')[1])
    except Exception as e:
        logger.error("Can't parse object key: {}".format(obj_summary.key))
        return

    cube_prefix = re.sub(r'/[^/]*$', '', obj_summary.key)
    cube_key = '{}/aruco-cube.txt'.format(cube_prefix)
    try:
        rawbytes = s3.Object(CALIB_BUCKET, cube_key).get()['Body'].read()
        cube_name = rawbytes.decode('utf-8').strip()
    except s3.meta.client.exceptions.NoSuchKey:
        cube_name = None

    #result = locate(multiview, scene_dir, op, [], True)
    location_dir = '/tmp/scenes/{}/locations'.format(scene_name)
    os.makedirs(location_dir, exist_ok=True)
    image = Image(
        camera=cam_name,
        calibration=get_camera_calibration(cam_name),
        cube_position=cube_position,
        uri='s3://{}/{}'.format(CALIB_BUCKET, obj_summary.key),
        cube_name=cube_name)
    logger.info('Locating image: {}'.format(image))
    result, pos_prefix = locate(image, multiview, location_dir)

    output_summary = ''
    if result.stdout.strip():
        output_summary += "\n\nstdout:\n```\n{}\n```".format(
            result.stdout.decode('utf-8'))
        
    if result.stderr.strip():
        output_summary += "\n\nstderr:\n```\n{}\n```".format(
            result.stderr.decode('utf-8'))

    def notify(message, pos_prefix):
        if args.no_slack:
            print('NOTIFY', message, pos_prefix)
        else:
            slack_send(message, pos_prefix)
            
    if result.returncode == 0:
        template = (
            'Location completed successfully for {cam}, {aruco}{output}')
        message = template.format(
            cam=cam_name, aruco=cube_position, output=output_summary)

        notify(message, pos_prefix)
    else:
        message = "Location failed for {cam}, {aruco}!{output}".format(
            cam=cam_name, aruco=cube_position, output=output_summary)
        notify(message, pos_prefix)

def locate(image, multiview, location_dir):
    '''Locate one image relative to the aruco cube'''
    filename = image.uri.split('/')[-1]
    ts = re.sub(r'^stereo-.*-(.*)\.jpg$', '\\1', filename)
    view_name = '{}_{}_{}'.format(image.camera, image.cube_position, ts)
    output_dir = os.path.join(location_dir, view_name)
    pos_file = os.path.join(output_dir, 'positions.json')

    try:
        shutil.rmtree(output_dir)
    except FileNotFoundError:
        pass
    
    os.makedirs(output_dir, exist_ok=True)
    args = [
        'release', 'no-cuda', 'phase_camera_extrinsic',
        '-d', output_dir,
        '--cam', image.calibration,
    ]
    if image.cube_name:
        args.extend(['--aruco-cube', image.cube_name])
    args.append(image.uri)
    result = multiview.run('multiview/multiview_cpp/run.sh', args)
    return result, output_dir
    
def monitor_extrinsic(args):
    '''
    Loop forever, checking s3 for new images, locate relative to aruco box,
    and notify slack about result.
    '''
    s3 = boto3.resource('s3')
    bucket = s3.Bucket(CALIB_BUCKET)

    prefix = '{}/scenes/'.format(CALIB_DATA_BASE)
    known_files = set(obj.key for obj in bucket.objects.filter(Prefix=prefix))
    logger.info('Found {} initial files in s3://{}/{}'.format(
        len(known_files), CALIB_BUCKET, prefix))

    begin_time = datetime.now(tz=timezone.utc)
    while True:
        logger.info('Querying...')
        for summary in bucket.objects.filter(Prefix=prefix):
            is_new = summary.last_modified > begin_time
            if (summary.key not in known_files and is_new
                and summary.key.endswith('.jpg')):
                logger.info('Found new image: s3://{}/{}'.format(
                    CALIB_BUCKET, summary.key))
                locate_one_image(summary, args)
                known_files.add(summary.key)
        time.sleep(10)
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        usage=argparse.SUPPRESS,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="""
Run continuously, monitoring s3 for new extrinsic positioning images, and
run positioning to test face detection. This script expects the environment
variable SLACK_BOT_TOKEN to exist, which it will use to send notifications
to Slack.
    """)
    parser.add_argument('--local',
                        action='store_true',
                        help='Run w/ local multiview, not docker container.')
    parser.add_argument('--no-slack',
                        action='store_true',
                        help='Print messages on stdout instead of sending '
                        'slack messages (useful for debugging)')
    parser.add_argument('--verbose',
                        action='store_true',
                        help='print debug messages')
    parser.add_argument('--test-one-image',
                        help='For debugging, run exactly one image, with s3 '
                        'key specified')
    args = parser.parse_args()

    level = 'INFO' if args.verbose else 'WARNING'
    logging.basicConfig(level=level)
    
    if not args.no_slack and 'SLACK_BOT_TOKEN' not in os.environ:
        print('Must set SLACK_BOT_TOKEN!', sys.stderr)
        exit(1)

    if args.test_one_image:
        s3 = boto3.resource('s3')
        bucket = s3.Bucket(CALIB_BUCKET)
        objs = list(bucket.objects.filter(Prefix=args.test_one_image))
        if len(objs) != 1:
            print("Can't find unique s3 object with key {}".format(
                args.test_one_image))
            exit(1)
        locate_one_image(objs[0], args)
    else:
        monitor_extrinsic(args)
