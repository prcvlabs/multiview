
import errno
import sys
import os
import cv2
import json
import pdb
import numpy as np
import glob
import pkgutil
import inspect
import multiprocessing
import subprocess

# ----------------------------------------------------------- set up python path

# The absolute path of the current directory
multiview_py_dir = os.path.abspath(str(__path__[0]))

# Put '__init__.py's directory at the start of the import part
sys.path.insert(0, multiview_py_dir)

# ---------------------------------------- import multiview_cpp, the C++ library

# Build the C++ bindings (if necessary), using multiple CPUs if available

if 'MULTIVIEW_USE_GL' in os.environ and os.environ['MULTIVIEW_USE_GL'] == 'true':
    cmd = "cd {0}/multiview_cpp && ./run.sh release no-cuda multiview_gl_cpp.so".format(multiview_py_dir)
    print(cmd)
                      
    if subprocess.run(cmd, shell=True).returncode != 0:
        raise RuntimeError("Failed to build C++ bindings, aborting")
    from multiview_cpp.multiview_gl_cpp import *
else:
    if subprocess.run("cd {0}/multiview_cpp && ./run.sh release no-cuda multiview_cpp.so"
                      "".format(multiview_py_dir),
                      shell=True).returncode != 0:
        raise RuntimeError("Failed to build C++ bindings, aborting")
    from multiview_cpp.multiview_cpp import *

# -------------------------------------- import child packages in specific order

from base import *

# ------------------------------------------------------ I suppose this is clean

# Good citizens are TIDY
del multiview_py_dir
sys.path.pop(0)

# -------------------------------------- defunct "import" code kept for interest
# The problem is that there's no direct control over the import order.

# Same as:
# from module1 import *
# from module2 import *
# ...etc...
# but for all the .py files (modules) in this directory
# see: https://stackoverflow.com/questions/16852811/python-how-to-import-from-all-modules-in-dir

if 0 > 1:
    __all__ = []
    for loader, name, is_pkg in pkgutil.walk_packages(__path__):
        module = loader.find_module(name).load_module(name)
        for name, value in inspect.getmembers(module):
            if name.startswith('__'):
                continue
            globals()[name] = value
            __all__.append(name)

        
