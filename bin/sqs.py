#!/usr/bin/python3

import sys
from tempfile import NamedTemporaryFile
import boto3
import botocore.exceptions 
import argparse
import json
import os
import logging
from time import sleep
from datetime import datetime, timedelta
import signal
import multiprocessing
import threading
import time
import requests

logger = logging.getLogger(__name__)

transfer_dir = os.path.dirname(__file__)
code_dir = os.path.dirname(transfer_dir)

sys.path.extend([
    os.path.join(code_dir, 'multiview'),
])

import multiview
import guapi_client

TRACKS_BUCKET = os.environ.get('TRACKS_BUCKET', 'perceive-tracks')

INCOMING_QUEUE = os.environ.get('INCOMING_QUEUE', 'multiview')
AWS_REGION = 'us-east-1'
logging.basicConfig(level=logging.INFO)

AWS_DATETIME_FMT = '%Y-%m-%dT%H:%M:%SZ'
DATETIME_FMT = '%Y-%m-%dT%H:%M:%S.%f%z'
S3_FILE_DATETIME_FMT = '%Y-%m-%dt%H-%M-%S.%f%z'
FAKE_MULTIVIEW = os.environ.get('FAKE_MULTIVIEW')

FAKE_TRACK_DATA = {
  "track_results": 
   {
      "width": 78,
      "height": 188,
      "frames": 189,
      "radius": 3,
      "costs": [-1003.651123],
      "entrances": [67],
      "exits": [],
      "paths": [[[77, 91, 42], [77, 91, 43], [77, 91, 44], [77, 91, 45], [77, 90, 46], [76, 90, 47], [75, 90, 48], [74, 90, 49], [73, 90, 50], [72, 90, 51], [71, 90, 52], [70, 90, 53], [69, 90, 54], [68, 91, 55], [67, 91, 56], [66, 92, 57], [65, 92, 58], [64, 92, 59], [63, 92, 60], [61, 92, 61], [60, 92, 62], [58, 92, 63], [57, 92, 64], [56, 92, 65], [55, 92, 66], [54, 92, 67], [53, 92, 68], [52, 92, 69], [51, 92, 70], [50, 92, 71], [49, 92, 72], [48, 92, 73], [47, 92, 74], [46, 92, 75], [44, 92, 76], [43, 93, 77], [41, 93, 78], [40, 93, 79], [39, 93, 80], [38, 93, 81], [37, 93, 82], [36, 93, 83], [35, 92, 84], [33, 92, 85], [32, 92, 86], [31, 92, 87], [30, 92, 88], [29, 91, 89], [28, 90, 90], [28, 90, 91], [27, 89, 92], [26, 88, 93], [25, 87, 94], [24, 86, 95], [23, 85, 96], [23, 84, 97], [22, 83, 98], [22, 82, 99], [22, 81, 100], [21, 80, 101], [21, 79, 102], [21, 78, 103], [20, 77, 104], [20, 76, 105], [19, 75, 106], [19, 75, 107], [18, 74, 108], [17, 73, 109], [16, 72, 110], [16, 72, 111], [15, 71, 112], [14, 71, 113], [13, 70, 114], [12, 70, 115], [11, 70, 116], [10, 70, 117], [9, 70, 118], [9, 70, 119], [8, 70, 120], [7, 70, 121], [7, 70, 122], [5, 70, 123]]]
   }
}

camera_precedence = [
    ['C0001008', 'C0001010'],
    ['C0001000', 'C0001003', 'C0001006'],
    ['C0001001', 'C0001010'],
]

def get_camera_precedence(serial_number):
    for lst in camera_precedence:
        if serial_number in lst:
            return len(lst) - lst.index(serial_number)
    return 0

# set by signal handler if we have received SIGTERM
received_sigterm = False
def term_handler(signum, frame):
    global received_sigterm
    logger.warn('Received signal %s', signal.Signals(signum).name)
    received_sigterm = True

def task_exiting():
    url = 'http://169.254.169.254/latest/meta-data/spot/instance-action'
    action_time = None
    try:
        response = requests.get(url, timeout=0.5)
        response.raise_for_status()
        data = response.json()
        action_timestr = data['time']
        action_time = datetime.strptime(action_timestr, AWS_DATETIME_FMT)
    except:
        #logger.info('Error while querying instance metadata', exc_info=True)
        pass

    if None is not action_time:
        time_until_stop = action_time - datetime.utcnow()
        if time_until_stop < timedelta(hours=1):
            return True
    
    return received_sigterm

def make_scene(manifest_data):
    scene = multiview.SceneDescription()
    with NamedTemporaryFile(mode='w', suffix='.json') as manifest_file:
        json.dump(manifest_data, manifest_file)
        manifest_file.flush()
        scene.init_from_filename(manifest_file.name)
        return scene

def guapi_connect():
    ssm = boto3.client('ssm', region_name=AWS_REGION)
    result = ssm.get_parameter(Name='traffic-worker')
    config_json = result['Parameter']['Value']
    config = json.loads(config_json)
    guapi = guapi_client.Connection(
        user=config['guapi_user'],
        password=config['guapi_password'],
    )
    return guapi

class MessageManager(object):
    def __init__(self, message, wait_seconds=240, margin_seconds=60):
        self._wait_seconds = wait_seconds
        self._margin_seconds = margin_seconds
        self._message = message
        self._last_refresh = None

    def beat(self):
        now = time.time()
        if (None is self._last_refresh or
            now - self._last_refresh >= self._wait_seconds
        ):
            self._refresh_message()
            self._last_refresh = now

    def _refresh_message(self):
        try:
            logger.info('Refreshing message visibility: %s', self._message)
            self._message.change_visibility(
                VisibilityTimeout=self._wait_seconds + self._margin_seconds)
        except botocore.exceptions.ClientError:
            logger.exception(
                'Failed to change visibility of message %s',
                self._message
            )

class PrematureShutdown(RuntimeError):
    '''
    raised when we are in the middle of a job and receive some sort of
    shutdown signal (SIGTERM, spot instance shutdown notification, etc), or
    when multiview process returns nonzero
    '''
    pass

class TrafficJob(object):
    def __init__(self, guapi, message):
        self._guapi = guapi
        self._message = message
        self._manifest = json.loads(message.body)
        sqs = boto3.resource('sqs', region_name=AWS_REGION)
        self._preprocessor_queue = sqs.get_queue_by_name(
            QueueName='preprocessor')

    def _preferred_video(self, when):
        best = None
        
        for video in self._manifest['videos']:
            start = datetime.strptime(video['timestamp'], S3_FILE_DATETIME_FMT)
            end = start + timedelta(seconds=float(video['duration']))
            if when >= start and when < end:
                if None is best:
                    best = video
                else:
                    best_precedence = get_camera_precedence(best['camera'])
                    this_precedence = get_camera_precedence(video['camera'])
                    if this_precedence > best_precedence:
                        best = video

        return best
                
    def _set_video_status(self, status, force=True):
        kwargs = {}
        if force:
            kwargs['force_status_change'] = 'true'
        for video in self._manifest['videos']:
            self._guapi.update(
                'processing', 'video', video['id'],
                status=status,
                **kwargs
            )

    def _make_traffic(self, traffic_type, frame, version):
        ''' Create an entrance or exit in GUAPI'''
        start_dt = datetime.strptime(
            self._manifest['epoch'], S3_FILE_DATETIME_FMT)
        frame_time = start_dt + timedelta(seconds=frame / 15)

        # create person
        person = self._guapi.create('cognitive', 'person', type='customer')
        logger.info('Person created: %s', person)

        # create traffic
        traffic = self._guapi.create(
            'cognitive', 'traffic',
            type='exit',
            timestamp=frame_time.strftime(DATETIME_FMT),
            source='multiview',
            person=person['id'],
            store=self._manifest['store'],
            analyst_status='generated', #I don't even know what this is
            source_version=version,
        )
        logger.info('Traffic created: %s', traffic)

        if 'entrance' == traffic_type:
            # Find video containing frame, preferring cameras earlier in
            # camera_precedence, and send preprocessor message to extract
            # frame
            best_vid = self._preferred_video(frame_time)
            if None is best_vid:
                logger.error(
                    'No video found for preprocessing! time=%s, videos=%s',
                    frame_time,
                    self._manifest['videos'])
            else:
                message = {
                    'bucket': 'entrance',
                    'entrance_id': traffic['id'],
                    'video_id': best_vid['id'],
                    'source_app': 'entrance'
                }
                logger.info('Will send preprocessor: %s', message)
                self._preprocessor_queue.send_message(
                    MessageBody=json.dumps(message)
                )

    def _create_traffic(self, tracks_file):
        '''upload track data to GUAPI'''
        with open(tracks_file, 'r') as f:
            output_data = json.load(f)

        track_data = output_data['track_results']

        logger.info(
            'creating traffic, %d entrances, %d exits',
            len(track_data['entrances']),
            len(track_data['exits']),
        )

        version = output_data.get('version', '')
        
        for frame in track_data['entrances']:
            self._make_traffic('entrance', frame, version)

        for frame in track_data['exits']:
            self._make_traffic('exit', frame, version)

        try:
            self._upload_tracks_file(tracks_file)
        except:
            logger.info('Failed to upload tracks file', exc_info=True)

    def _upload_tracks_file(self, tracks_file):
        s3 = boto3.client('s3')
        key = '%s_%s.json' % (self._manifest['store'], self._manifest['epoch'])
        s3.upload_file(tracks_file, TRACKS_BUCKET, key)

    def _run_multiview(self, tracks_file_name, stats_file_name):
        params = multiview.TweakerParams()
        scene = make_scene(self._manifest)
        multiview.pipeline_entrypoint(
            scene,
            params,
            '', # in_stats_file
            stats_file_name, # out stats file
            True, # generate tracks
            True, # generate stats
            tracks_file_name, # output file
            False, # make video
            True, #feedback
        )

    def run_multiview(self, tracks_file_name, stats_file_name):
        '''
        Run multiview in child process. Periodically increase message visibility
        as well as check to see if we are being signalled to shut down.
        '''
        proc = multiprocessing.Process(
            target=self._run_multiview,
            args=(tracks_file_name, stats_file_name),
            daemon=True)
        proc.start()

        manager = MessageManager(self._message)
        while proc.is_alive():
            if task_exiting():
                os.kill(proc.pid, signal.SIGKILL)
                raise PrematureShutdown()
            
            manager.beat()
            time.sleep(5)

        if 0 != proc.exitcode:
            raise PrematureShutdown()

    def run(self):
        logger.info(
            'Message received from %s: %s' % (INCOMING_QUEUE, self._manifest))
        
        self._set_video_status('multiviewing')
        
        stats_file = NamedTemporaryFile(mode='r', suffix='.stats')
        tracks_file = NamedTemporaryFile(mode='r', suffix='.json')

        try:
            if FAKE_MULTIVIEW:
                with open(tracks_file.name, 'w') as f:
                    json.dump(FAKE_TRACK_DATA, f)
            else:
                logger.info(
                    'Running multiview on %s, %s',
                    stats_file.name, tracks_file.name)
                self.run_multiview(tracks_file.name, stats_file.name)
                
            self._create_traffic(tracks_file.name)
        except PrematureShutdown:
            logger.error('Premature shutdown!')
            self._set_video_status('synchronized', force=True)
            logger.info('Shutting down: reversed video status')
            raise
        finally:
            stats_file.close()
            tracks_file.close()

        self._set_video_status('multiviewed')

def receive_messages(queue):
    while not received_sigterm:
        yield from queue.receive_messages(WaitTimeSeconds=20)

class AutoScaling(object):
    def __init__(self):
        self._client = boto3.client('autoscaling', region_name=AWS_REGION)
        self._instance_id = None
        self._auto_scaling_group = None

        try:
            response = requests.get(
                'http://169.254.169.254/latest/meta-data/instance-id',
                timeout=0.5)
            response.raise_for_status()
            instance_id = response.text
        except:
            return
        
        result = self._client.describe_auto_scaling_groups()
        for group in result['AutoScalingGroups']:
            for instance in group['Instances']:
                if instance_id == instance['InstanceId']:
                    self._instance_id = instance_id
                    self._auto_scaling_group = group['AutoScalingGroupName']
                    logger.info(
                        'I am instance %s in auto scaling group %s',
                        self._instance_id,
                        self._auto_scaling_group)
                    return

    def set_instance_protection(self, should_protect):
        if self._instance_id and self._auto_scaling_group:
            logger.info(
                'Set protection for instance %s in group %s to %s',
                self._instance_id,
                self._auto_scaling_group,
                should_protect)
            self._client.set_instance_protection(
                InstanceIds=[self._instance_id],
                AutoScalingGroupName=self._auto_scaling_group,
                ProtectedFromScaleIn=should_protect)
        else:
            logger.info(
                'Not in auto scaling group; not setting instance protection=%s',
                should_protect)
            
def polling_loop():
    signal.signal(signal.SIGTERM, term_handler)
    signal.signal(signal.SIGINT, term_handler)

    autoscaling = AutoScaling()
    sqs = boto3.resource('sqs', region_name=AWS_REGION)
    queue = sqs.get_queue_by_name(QueueName=INCOMING_QUEUE)
    guapi = guapi_connect()

    for message in receive_messages(queue):
        autoscaling.set_instance_protection(True)
        
        job = TrafficJob(guapi, message)
        job.run()

        autoscaling.set_instance_protection(False)
        
        message.delete()

if __name__ == '__main__':
    polling_loop()

