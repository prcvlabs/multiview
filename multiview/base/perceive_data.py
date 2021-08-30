
import os
from base.colors import *
from base.general_utils import *

@static_vars(DATA_DIR__=None)
def perceive_data_dir():
    """
    Returns the PERCEIVE_DATA environment variable.
    Gives a friendly reminder if it has not already been set.
    PERCEIVE_DATA gives the path to various internally shared data files.
    (Currently hosted on dropbox.)
    """
    
    if not isinstance(perceive_data_dir.DATA_DIR__, str):
        perceive_data_dir.DATA_DIR__ = ''
        key = 'PERCEIVE_DATA'
        try:
            perceive_data_dir.DATA_DIR__ = os.environ[key]
            if perceive_data_dir.DATA_DIR__ == '':
                # the user has intentially set this empty
                print_warn("Environment variable '{0}' set to ''".format(key))                
            elif not os.path.isdir(perceive_data_dir.DATA_DIR__):
                print_banner_err("Environment variable {0}={1} does not reference an existing "
                                 "directory".format(key, perceive_data_dir.DATA_DIR__))
            if perceive_data_dir.DATA_DIR__.endswith('/'):
                perceive_data_dir.DATA_DIR__ = perceive_data_dir.DATA_DIR__[:-1]
        except KeyError:
            print_banner_err("Environment variable '{0}' does not exist. Please put\n\n"
                             "   export {0}=/path/to/corpus/directory\n\n"
                             "into your .profile or .bashrc".format(key))

    return perceive_data_dir.DATA_DIR__

def perceive_data_relative_filename(filename):
    """Returns the path to `filename` _relative_ to the PERCEIVE_DATA directory"""
    f = os.path.abspath(filename)
    d = perceive_data_dir()
    if d == '':
        return f    
    if f.startswith(d + '/'):
        return f[len(d)+1:]
    return f

def perceive_data_filename(filename):
    """If `filename` is an absolute path, will just return `filename`.
    Otherwise returns absolute path of $PERCEIVE_DATA/filename"""
    if len(filename) > 0 and filename[0] == '/':
        return filename
    return os.path.abspath(perceive_data_dir() + '/' + filename)
    
