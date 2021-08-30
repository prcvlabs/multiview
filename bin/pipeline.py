#!/usr/bin/python3

import errno
import sys
import os
import cv2
import json
import pdb
import argparse
import datetime
import gc
import multiprocessing
import numpy as np
import math
import functools
import random

sys.path.append('./..' if os.path.dirname(__file__) == '' else os.path.dirname(__file__) + '/..' )
import multiview as mv

# ------------------------------------------------------------------------- main

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(
        formatter_class=mv.NullTextHelpFormatter,
        usage=argparse.SUPPRESS,
        description="""

Usage: {0} [OPTIONS...] <manifest-filename>

      -p <filename>     Parameters filename. (Default is none.)
      -s <filename>     Stats input file. (Default is none.)
      -o <filename>     Output filename. 
      -y                Allow overwrite of output filename.

      -d <dirname>      Output directory.

      --use-fowlkes <filename>
      --gen-fowlkes <filename>

      --generate-stats <filename>

      --create-video   Video is saved to the output directory.
      --present-video  Video is saved to the output directory.

Examples:

      TODO

      # Generate stats for 'manifest-file.json', saving to '/tmp/stats-out.mv'
      ~ > {0} --generate-stats /tmp/stats-out.mv manifest-file.json

      # Use stats to generate a regular output file '/tmp/tracks.json'
      ~ > {0} -s /tmp/stats-out.mv -o /tmp/tracks.json manifest-file.json 

""".format(os.path.basename(sys.argv[0]))
    )
    
    parser.add_argument('infile', type=str)

    # Output options
    parser.add_argument('-p', '--params-file', action='store', type=str,
                        default='')
    parser.add_argument('-s', '--stats-file', action='store', type=str,
                        default='')
    parser.add_argument('-o', '--outfile', action='store', type=str,
                        default='')
    parser.add_argument('-y', '--overwrite', action='store_true')
    parser.add_argument('-d', '--outdir', action='store', type=str,
                        default='/tmp')
    
    parser.add_argument('-a', '--use-fowlkes', action='store', type=str,
                        default='')
    parser.add_argument('-b', '--gen-fowlkes', action='store', type=str,
                        default='')
    
    parser.add_argument('-g', '--generate-stats', type=str, default='')
    parser.add_argument('-v', '--create-video', action='store_true')
    parser.add_argument('-z', '--present-video', action='store_true')

    
    args = parser.parse_args()
    
    feedback = True
    params_file = args.params_file
    stats_file = args.stats_file
    outfile = args.outfile
    generate_tracks = not args.outfile == ''
    allow_overwrite = args.overwrite
    outdir = args.outdir
    use_fowlkes_file = args.use_fowlkes
    gen_fowlkes_file = args.gen_fowlkes
    out_stats_file = args.generate_stats
    generate_stats = not out_stats_file == ''
    make_video = args.create_video
    make_present = args.present_video
    manifest = args.infile 
    
    # Do we have an error in the passed command line arguments?
    has_error = False
    
    # Manifest file must exist
    if not os.path.isfile(manifest):
        mv.print_err("Failed to find manifest file '{}'".format(manifest))
        has_error = True

    # Output directory must exist
    if not os.path.isdir(outdir):
        mv.print_err("Failed to find output directory '{}'".format(outdir))
        has_error = True

    # Must specify stats or specify a stats file to load
    if not generate_stats and stats_file == '':
        mv.print_err("Must either generate stats, or supply a stats file")
        has_error = True

    # stats file must exist if specified
    if not stats_file == '' and not os.path.isfile(stats_file):
        mv.print_err("Failed to find stats file '{}'".format(stats_file))
        has_error = True

    # in and out stats file cannot be the same
    if generate_stats and stats_file == out_stats_file:
        mv.print_err("stats file, and out-stats file cannot be the same")
        has_error = True        

    # Don't overwrite unless -y is specified.
    if os.path.isfile(outfile) and not allow_overwrite:
        mv.print_err("Cowardly refusing to overwrite output file '{0}'."
                     .format(outfile))
        has_error = True

    # Parameters file must exist if specified
    if params_file != '' and not os.path.isfile(params_file):
        mv.print_err("Failed to find parameters file '{}'".format(params_file))
        has_error = True

    # Cannot generate and load fowlkes at the same time
    if use_fowlkes_file != '' and gen_fowlkes_file != '':
        mv.print_err("Cannot generate and use a fowlkes file at the same time")
        has_error = True

    # Fowlkes file must exist if specified
    if use_fowlkes_file != '' and not os.path.isfile(use_fowlkes_file):
        mv.print_err("Failed to find fowlkes file '{}'"
                     .format(use_fowlkes_file))
        has_error = True

    # Cannot overwrite fowlkes file unless 'allow_overwrite'
    if gen_fowlkes_file != '' \
       and os.path.isfile(gen_fowlkes_file) \
       and not allow_overwrite:
        mv.print_err("Cowardly refusing to overwrite output file '{}'".
                     gen_fowlkes_file)
        has_error = True

    # Cannot overwrite out_stats file unless 'allow_overwrite'
    if out_stats_file != '' \
       and os.path.isfile(out_stats_file) \
       and not allow_overwrite:
        mv.print_err("Cowardly refusing to overwrite output file '{}'".
                     out_stats_file)
        has_error = True
        
    # Cannot overwrite out_stats file unless 'allow_overwrite'
    if outfile != '' \
       and os.path.isfile(outfile) \
       and not allow_overwrite:
        mv.print_err("Cowardly refusing to overwrite output file '{}'".
                     out_stats_file)
        has_error = True
        
    if has_error:
        print("Aborting due to previous errors")
        exit(1)        

    scene_desc = mv.SceneDescription()
    scene_desc.init_from_filename(manifest)
    
    params = mv.TweakerParams()
    if params_file != '':
        params.load(params_file)
    if not outdir == '':
        params.out_dir = outdir
    params.gen_fowlkes_file = gen_fowlkes_file != ''
    params.fowlkes_file = gen_fowlkes_file \
                          if params.gen_fowlkes_file else use_fowlkes_file
    

    # Run the pipeline
    mv.pipeline_entrypoint(scene_desc,
                           params,
                           stats_file,
                           out_stats_file,
                           generate_tracks,
                           generate_stats,
                           outfile,
                           make_video,
                           make_present,
                           feedback)        

    exit(0)
    
