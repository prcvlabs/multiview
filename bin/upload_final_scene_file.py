#!/usr/bin/env python3

from argparse import ArgumentParser, ArgumentTypeError
import boto3
import collections
import re
import os
import sys
from lib.calibration import *
import guapi_client
import json
import io
from datetime import datetime, timezone

def die(msg):
    print('ERROR: ' + msg, file=sys.stderr)
    exit(1)

DATETIME_FMTS = [
    '%Y-%m-%dT%H:%M:%S.%f%z',
    '%Y-%m-%dT%H:%M:%S.%f',
    '%Y-%m-%dT%H:%M:%S',
    '%Y-%m-%dT%H:%M',
    '%Y-%m-%dT%H',
    '%Y-%m-%d',
]

def format_date(dt):
    if dt is None:
        return None
    return dt.strftime(DATETIME_FMTS[0])

def date_arg(datestr):
    for fmt in DATETIME_FMTS:
        try:
            return datetime.strptime(datestr, fmt)
        except ValueError:
            pass

    raise argparse.ArgumentTypeError('Invalid date: {}'.format(datestr))

def main():
    guapi = guapi_client.Connection()
    
    parser = ArgumentParser(
        description='Create final scene from the dev scene. Edits the scene '
        'to add the AABB, and changes the scene key to the production one. '
        'Uploads to S3, and creates scene in GUAPI.')
    parser.add_argument(
        '--scene',
        required=True,
        help='Name of the scene (format scene_name_vXX).')
    parser.add_argument(
        '--aabb',
        required=True,
        help='Histogram AABB, in JSON format, e.g. [0.0, 0.0, 1.0, 1.0].')
    parser.add_argument(
        '--environment',
        required=True,
        help='Name of the environment in the API to associate the scene with. '
        'If this is "production", incoming videos will be processed by the '
        'pipeline automatically.')
    parser.add_argument(
        '--valid-starting',
        required=True,
        type=date_arg,
        help='Date when the scene becomes active, e.g. 2020-01-01T00:00:00.')
    parser.add_argument(
        '--store-id',
        type=int,
        help='id of the store to associate the scene with.')
    parser.add_argument(
        '--store-name', help='Name of the store to associate the scene with.')
    parser.add_argument(
        '--valid-ending',
        type=date_arg,
        help='Date when the scene becomes inactive, e.g. 2020-01-01T00:00:00.')
        
    args = parser.parse_args()

    # lookup environment in GUAPI
    all_envs = list(guapi.list('processing', 'environment'))
    envs = list(guapi.list(
        'processing', 'environment', name=args.environment))
    if 0 == len(envs):
        die("no environment found for name {} (valid choices are {})".format(
            args.environment, [e['name'] for e in all_envs]))
    if 1 < len(envs):
        die("multiple environments found for name, this is impossible")
    environment = envs[0]

    
    #lookup store in GUAPI
    if args.store_id and args.store_name:
        die("Can't specify both --store-id and --store-name.")
    elif not (args.store_id or args.store_name):
        die("Must specify --store-id or --store-name")
    elif args.store_id:
        try:
            store = guapi.get('retail', 'store', int(args.store_id))
        except guapi_client.QueryError:
            die("store {} not found".format(args.store_id))
    else:
        stores = list(guapi.list('retail', 'store', name=args.store_name))
        if 0 == len(stores):
            die("No stores found for name {}".format(args.store_name))
        elif 1 < len(stores):
            die("multiple stores found for name, this is impossible")

        store = stores[0]

    # parse scene name
    if not re.match(r'(.*)_v(\d+)$', args.scene):
        die('Scene must be of format scene_name_vXX')
        
    initial_scene = args.scene + '-DEV'
    input_keys = s3_list(CALIB_BUCKET, '{0}/scenes/{1}_'.format(
        CALIB_BASE, initial_scene))

    if 0 == len(input_keys):
        die('No scene file found in s3 for {}'.format(args.scene))
    elif 1 < len(input_keys):
        die('Multiple scenes found for {}: {}'.format(
            args.scene, input_keys))

    final_scene = args.scene + '-PROD'
    final_keys = s3_list(CALIB_BUCKET, '{0}/scenes/{1}_'.format(
        CALIB_BASE, final_scene))

    # Parse aabb
    try:
        aabb = json.loads(args.aabb)
    except json.JSONDecodeError:
        die('Invalid AABB format (json parse error)')

    if not isinstance(aabb, list) or len(aabb) != 4:
        die('AABB must be a list of size 4')

    try:
        aabb = [float(el) for el in aabb]
    except ValueError:
        die('AABB must contain valid numbers')

    # load scene data
    s3 = boto3.resource('s3')
    input_bytes = s3.Object(CALIB_BUCKET, input_keys[0]).get()['Body'].read()
    try:
        scene_data = json.loads(input_bytes.decode('utf-8'),
                                object_pairs_hook=collections.OrderedDict)
    except Exception as e:
        die("Failed to parse scene file: {}".format(e))

    # Update scene
    scene_data['hist-aabb'] = aabb
    scene_data['scene-key'] = final_scene

    # upload to correct hashed filename
    output_bytes = json.dumps(scene_data, indent=4).encode('utf-8')
    key = '{}/scenes/{}'.format(
        CALIB_BASE,
        hashed_filename_from_data(output_bytes, final_scene))

    if len(final_keys) > 1:
        die("Multiple versions of scene exists: {}".format(final_keys))
    elif 1 == len(final_keys) and key != final_keys[0]:
        die("Scene already exists, with different hash: {}".format(
            final_keys[0]))

    s3.Object(CALIB_BUCKET, key).put(Body=output_bytes)
    print('Uploaded scene to s3://{}/{}'.format(CALIB_BUCKET, key))

    
    # Create scene in GUAPI
    cams = [key.split('_')[0] for key in scene_data['bcam-keys']]
    result = guapi.create(
        'hardware', 'scene',
        name=final_scene,
        valid_starting=format_date(args.valid_starting),
        valid_ending=format_date(args.valid_ending),
        retail_store=store['id'],
        environment=environment['id'],
        cameras=cams)

    print('Created scene:')
    print(json.dumps(result, indent=4))
    
if '__main__' == __name__:
    main()
