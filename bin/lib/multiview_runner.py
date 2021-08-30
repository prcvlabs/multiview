import subprocess
import configparser
import os
from os.path import dirname, realpath, join
import shlex
import boto3
import base64

class MultiviewError(RuntimeError):
    def __init__(self, error_code, command):
        super().__init__('{0} returned  {1}'.format(command, error_code))
        self.command = command
        self.error_code = error_code

class Multiview(object):
    '''
    Object for running multiview in docker container, or locally.
    '''
    def __init__(self):
        self.capture_output = False
        self._docker_args = []
        self._volumes = []
        self.raise_on_error = True
        self.docker = True
        self.docker_image = (
            '096316437681.dkr.ecr.us-east-1.amazonaws.com/multiview:production')
        self._env = {}
        
    def map_volume(self, host_dir, container_dir=None):
        if None is container_dir:
            container_dir = host_dir
        self._volumes.append((host_dir, container_dir))

    def inject_fs_aws_creds(self, aws_dir):
        creds = configparser.ConfigParser()
        creds.read(os.path.join(aws_dir, 'credentials'))
        for var, value in creds['default'].items():
            self._docker_args.extend(['-e', '{}={}'.format(var.upper(), value)])

    def pull_docker_image(self):
        def run(cmd):
            result = subprocess.run(cmd)
            if result.returncode != 0 and self.raise_on_error:
                raise MultiviewError(result.returncode, ' '.join(cmd))
            return result

        ecr = boto3.client('ecr')
        response = ecr.get_authorization_token()
        b64_data = response['authorizationData'][0]['authorizationToken']
        creds = base64.b64decode(b64_data)
        user, token = creds.split(b':')
        endpoint = response['authorizationData'][0]['proxyEndpoint']
        result = run(['sudo', 'docker', 'login',
                      '-u', user,
                      '-p', token,
                      endpoint])
        if result.returncode:
            return result
        else:
            return run(['sudo', 'docker', 'pull', self.docker_image])

    def set_env(self, var, value):
        self._env[var] = value

    def run(self, command_rel, run_args):
        if self.docker:
            cmd = ['sudo', 'docker', 'run',
                   '-t',
                   '--rm']
            
            cmd.extend(self._docker_args)
            for var, value in self._env.items():
                cmd.extend(['-e', '{}={}'.format(var, value)])

            for host_vol, container_vol in self._volumes:
                cmd.extend(['-v', '{}:{}'.format(host_vol, container_vol)])

            aws_vars = ['AWS_ACCESS_KEY_ID', 'AWS_SECRET_ACCESS_KEY']
            if all(var in os.environ for var in aws_vars):
                for var in aws_vars:
                    cmd.extend(['-e', '{}={}'.format(var, os.environ[var])])

            cmd.extend([
                self.docker_image,
                join('/home/acorn/multiview', command_rel)])
        else:
            multiview_dir = dirname(dirname(dirname(realpath(__file__))))
            cmd = [join(multiview_dir, command_rel)]
            
        cmd.extend(run_args)

        print('RUNNING MULTIVIEW:', ' '.join(shlex.quote(p) for p in cmd))

        env = os.environ.copy()
        if not self.docker:
            env.update(self._env)

        kwargs = {}
        if self.capture_output:
            kwargs['stdout'] = kwargs['stderr'] = subprocess.PIPE
        result = subprocess.run(cmd, env=env, **kwargs)
            
        if result.returncode != 0 and self.raise_on_error:
            raise MultiviewError(result.returncode, command_rel)

        return result
