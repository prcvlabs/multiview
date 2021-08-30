import os, boto3
import hashlib
import re

def get_asset_bucket():
    calib_bucket_uri = os.environ.get('MULTIVIEW_ASSET_S3_BUCKET',
                                         's3://perceive-multiview')
    uri_prefix = 's3://'
    if not calib_bucket_uri.startswith(uri_prefix):
        raise RuntimeError('invalid asset bucket: {}, should be of form '
                           '{}bucket-name'.format(calib_bucket_uri, uri_prefix))

    return calib_bucket_uri[len(uri_prefix):]

CALIB_BUCKET = get_asset_bucket()
CALIB_BASE = 'calibration'
CALIB_DATA_BASE = 'calibration-data'
CALIB_OUTPUT_BASE = 'calibration-output'

def s3_upload_dir(bucket, prefix, local_dir):
    local_prefix = local_dir.rstrip(os.sep) + os.sep
    s3 = boto3.resource('s3')

    for parent_path, dirnames, filenames in os.walk(local_dir):
        for filename in filenames:
            local_path = os.path.join(parent_path, filename)
            assert local_path.startswith(local_prefix)
            suffix = local_path[len(local_prefix):]
            key = prefix + suffix
            s3.Object(bucket, key).upload_file(local_path)
            print('Uploaded file to s3://{}/{}'.format(bucket, key))

def s3_list(bucket, prefix):
    s3 = boto3.resource('s3')
    return [obj.key for obj in s3.Bucket(bucket).objects.filter(Prefix=prefix)]

def s3_list_prefixes(bucket, prefix):
    ''' 
    return common prefixes, delimited by '/'. Roughly, does similar to what
    'aws s3 ls' does, but returns full key prefixes.
    '''
    print(bucket, prefix)
    s3 = boto3.client('s3')
    paginator = s3.get_paginator('list_objects')
    results = []
    for page in paginator.paginate(Bucket=bucket, Prefix=prefix, Delimiter='/'):
        if 'CommonPrefixes' in page:
            results.extend(r['Prefix'] for r in page['CommonPrefixes'])
        if 'Contents' in page:
            results.extend(r['Key'] for r in page['Contents'])
    return results

def hashed_filename(filename, base, extension='.json'):
    with open(filename, 'rb') as f:
        return hashed_filename_from_data(f.read(), base, extension)

def hashed_filename_from_data(contents, base, extension='.json'):
    md5 = hashlib.md5()
    md5.update(contents)
    output_hash = md5.hexdigest()
    return '{}_{}{}'.format(base, output_hash, extension)

camera_calib_re = re.compile(r'/((C\d{7,})_v(\d+)_([0-9a-f]{32})).json$')
def is_camera_calibration(key):
    return None is not camera_calib_re.search(key)

def parse_camera_calibration(key):
    match = camera_calib_re.search(key)
    if not match:
        raise ValueError("Invalid camera calibration: {}".format(key))
    fullname, cam, version, md5 = match.groups()
    return (fullname, cam, int(version), md5)
    
def get_camera_calibration(camera, overrides=[]):
    '''
    return latest calibration, unless overrides contains the name of a
    calibration with for the correct camera, in which case that is returned
    '''
    match = re.match(r'C\d{7,}$', camera)
    if not match:
        raise ValueError(
            "Invalid camera name (must be of form CXXXXXXX_vX)")

    fmt = re.compile(r'/({camera}_v(\d+)_[0-1a-f].json)$'.format(
        camera=re.escape(camera)))

    for fullname in overrides:
        if fullname.startswith(camera + '_'):
            return fullname

    prefix = '{0}/binocular/{1}_v'.format(CALIB_BASE, camera)
    latest_version = -1
    latest_calibration = None
    for key in s3_list(CALIB_BUCKET, prefix):
        if is_camera_calibration(key):
            fullname, cam, version, md5 = parse_camera_calibration(key)
            if version > latest_version:
                latest_version = version
                latest_calibration = fullname
                
    if None is latest_calibration:
        raise RuntimeError("camera calibration not found for {}".format(camera))
    return latest_calibration

