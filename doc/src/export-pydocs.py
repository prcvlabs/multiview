#!/usr/bin/python3

# https://docs.python.org/3/library/inspect.html

import math
import numpy as np
import pdb
import cv2
import os
import sys

import pydoc
from pydoc import safeimport
from pydoc import ErrorDuringImport
from pydoc import inspect

sys.path.append(os.path.dirname(__file__) + '/../..')

import multiview
MODULE_BASE = os.path.dirname(os.path.abspath(multiview.__file__))

module_header = "# Package \"{}\" Documentation\n"
class_header = "## Class {}"
function_header = "### {}"

def src_filename(f):
    if hasattr(f[1], '__code__'):
        return os.path.abspath(f[1].__code__.co_filename)
    return inspect.getfile(f[1])
    
def not_anon(f):
    return not (f[0].startswith('_') or f[0] == '__init__')

# ------------------------------------------------------------------------------------- Write Markup

def write_markup(outfile, markup):
    outdir = os.path.dirname(outfile)
        
    if not os.path.exists(outdir):
        os.makedirs(outdir)
    with open(outfile, 'w') as fp:
        fp.write(markup)
        
# ---------------------------------------------------------------------------------- Get Function MD

def get_function_md(func):
    output = list()
    output.append(function_header.format(func[0].replace('_', '\\_')))

    is_cpp = pydoc.inspect.isbuiltin(func[1])
    
    argspec = ''
    doc = ''
    
    if not is_cpp and pydoc.inspect.isfunction(func[1]):
        fmt_args = pydoc.inspect.formatargspec(*pydoc.inspect.getfullargspec(func[1]))
        argspec = 'def %s%s\n' % (func[0], fmt_args)
        doc = pydoc.inspect.getdoc(func[1])
    elif is_cpp:
        raw_doc = pydoc.inspect.getdoc(func[1]).replace('\n    ', '\n')
        pos = raw_doc.find(':')
        if pos == -1:
            doc = raw_doc
            argspec = ''
        else:
            # Split off the python signature
            argspec = 'def %s\n' % (raw_doc[0:pos])
            doc = raw_doc[pos+1:]
        
    # Get the signature
    output.append ('```py\n')
    output.append(argspec)
    output.append ('```\n')

    # get the docstring
    if pydoc.inspect.getdoc(func[1]):
        output.append('\n')
        output.append(doc)

    output.append('\n')
    return output

# ------------------------------------------------------------------------------------- Get Class MD

def get_classes_md(clazz):
    output = []
        
    output.append(class_header.format(clazz[0])) 
    output.append(pydoc.inspect.getdoc(clazz[1]))

    # Class functions
    funs = pydoc.inspect.getmembers(clazz[1], inspect.isfunction)
    for func in filter(lambda f: not_anon(f), funs):
        output.extend(get_function_md(func))

    # Class members
    funs = pydoc.inspect.getmembers(clazz[1], inspect.ismethod)
    for func in filter(lambda f: not_anon(f), funs):
        output.extend(get_function_md(func))

    # Class
    
    # Get the functions
    #output.extend(getfunctions(cl[1]))
    # Recurse into any subclasses
    #output.extend(getclasses(cl[1]))
    output.append('\n')
    
    return output

# ------------------------------------------------------------------------------------ Get Module MD

def write_module_header(module, module_name):

    mod_filename = 'docs/{0}.md'.format(module_name)
    with open(mod_filename, 'w') as fp:
        output = [module_header.format(module_name)]
        if module.__doc__:
            output.append(module.__doc__)
        markup = "\n".join(str(x) for x in filter(lambda x: not x is None, output))
        fp.write(markup)
    
def write_module_md(module):

    # Get the absolute path of the module (__init__.py file)
    module_dirname = os.path.dirname(os.path.abspath(module.__file__))

    write_module_header(module, module.__name__)
        
    # Find all of the filenames associated with the module
    funs = pydoc.inspect.getmembers(module, inspect.isfunction) + \
           pydoc.inspect.getmembers(module, inspect.isbuiltin)
    clazzes = pydoc.inspect.getmembers(module, inspect.isclass)
        
    # Gets the relative source file for the given function
    def rel_src_file(func):
        f = src_filename(func)
        return f[len(module_dirname)+1:] if f.startswith(module_dirname + '/') else None    

    # TRUE iff the function is defined in the passed filename
    def func_in_file(func, filename):
        return rel_src_file(func) == filename

    def is_relevant(f):
        return not_anon(f) and not rel_src_file(f) is None
    
    # TODO, this is only for 'isfunction', and probably needs to be extended
    filenames = sorted(list(set([rel_src_file(f) for f in filter(is_relevant, funs)])))
    
    # Output functions, by filename
    for filename in filenames:
        output = []
        for func in filter(lambda f: not_anon(f) and func_in_file(f, filename), funs):
            output.extend(get_function_md(func))
        output.append("***\n")
        for clazz in filter(lambda f: not_anon(f) and func_in_file(f, filename), clazzes):
            output.extend(get_classes_md(clazz))
        markup = "\n".join(str(x) for x in filter(lambda x: not x is None, output))

        basedir = 'docs/' + module_dirname[len(MODULE_BASE)+1:]
        outfile = basedir + '/' + filename[0:len(filename)-3] + '.md'
        write_markup(outfile, markup)
    
# ------------------------------------------------------------------------------------ Generate Docs

def write_docs(module):

    try:
        # Attempt import
        mod = safeimport(module)

        if mod is None:
           print("Module '{0}' not found".format(module))
           
        # Module imported correctly, let's create the docs
        write_module_md(mod)

    except ErrorDuringImport as e:
        print("Error while trying to import " + module)

# ---------------------------------------------------------------------- Generate Multiview Cpp Docs

def write_cpp_funs(module, filename, funcs):

    lookup = dict(pydoc.inspect.getmembers(module))

    output = []
    for func in funcs:
        if not func in lookup:
            print("Module does not contain function '{0}'".format(func))
            exit(1)
        output.extend(get_function_md( (func, lookup[func]) ))
    
    markup = "\n".join(str(x) for x in filter(lambda x: not x is None, output))
    
    outfile = 'docs/multiview_cpp/' + filename + '.md'
    write_markup(outfile, markup)

def write_multiview_cpp():

    module = 'multiview'
    module_name = 'multiview.multiview_cpp'

    try:
        mod = safeimport(module)

        # Write the module header
        write_module_header(mod, module_name)

        # Write 'utils'
        write_cpp_funs(mod, 'utils', ['tick', 'tock'])
        write_cpp_funs(mod, 'cost_functions', ['estimate_distortion_center',
                                               'hartley_kang',
                                               'hartley_kang_distort',
                                               'hartley_kang_undistort',
                                               'claus_2005_rational'])
        
        if mod is None:
            print("Module '{0}' no found".format(module))

    except ErrorDuringImport as e:
        print("Error while trying to import " + module)
            
# --------------------------------------------------------------------------------------------- main

if __name__ == '__main__': 

    write_multiview_cpp()
    write_docs('multiview.base')
    write_docs('multiview.calibrate')
        
