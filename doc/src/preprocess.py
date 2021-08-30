#!/usr/bin/python3

import re
import math
import pdb
import argparse

# --------------------------------------------------------------------------------------------- main

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='#include preprocessor.')
    parser.add_argument('filename', type=str)

    args = parser.parse_args()

    regex = re.compile("#include\s+\"([^\"]*)\"")
    regex = r"#include\s+\"([^\"]*)\""
    
    raw = None
    with open(args.filename, 'r') as fp:
        raw = fp.read()

    # raw= ("here\n\n"
    #     "a\n"
    #     "#include \"sample/text.py\"\n\n"
    #     "asdf\n\n"
    #     "d\n")

    matches = re.finditer(regex, (raw))

    last_end = 0

    for match_no, match in enumerate(matches):
        filename = match.groups()[0]
        text = ''
        with open(filename, 'r') as fp:
            text = fp.read()

        start = match.start()
        end = match.end()

        print(raw[last_end:start])
        print(text)
        
        last_end = end
        
    print(raw[last_end:])
    
