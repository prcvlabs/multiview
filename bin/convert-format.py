#!/usr/bin/env python3

import sys
import re
import pdb

format_re = r"format\((\"([^\"]|\\.)*?\"|R\"V0G0N\((.|\n)*?\)V0G0N\")"
placeholder_re = r"%([-+ 0]+)?([0-9]+)?(\.[0-9]+)?(hh|h|l|ll|L|z|j|t)?([%diufFeEgGxXcpaAns])"

def convert_format(raw_fmt):

    # Encode '{' -> '{{' and '}' -> '}}'
    raw_fmt.replace("{", "{{")
    raw_fmt.replace("}", "}}")

    # Build the result in 'parts'
    parts = []

    # Find C string placeholder specifiers
    pos = 0
    matches = re.finditer(placeholder_re, raw_fmt, re.MULTILINE)    
    for match_num, match in enumerate(matches, start = 1):
        start_pos = match.start()
        end_pos = match.end()
        raw = match.group()

        parts.append(raw_fmt[pos:start_pos])
        pos = end_pos

        flag = match.group(1)
        width = match.group(2)
        precision = match.group(3)
        length = match.group(4)
        type_s = match.group(5)

        width_s = '' if width is None else width
        precision_s = '' if precision is None else precision
        length_s = '' if length is None else length
        
        if type_s == "%":
            parts.append("%")
            continue

        fill_and_align = ''
        if len(fill_and_align) == '':
            if width_s == '':
                print('NO WIDTH for FILL AND ALIGN')
                exit(1)
            align = '<' if flag.find('-') >= 0 else '>'
            fill = '0' if flag.find('0') >= 0 else ''
            fill_and_align = fill + align
        
        type_p = ''
        if type_s == "d" or type_s == "i" or type_s == "u":
            type_p = "d"
        elif type_s == "x" or type_s == "X":
            type_p = type_s
        elif type_s == "e" or type_s == "E":
            type_p = type_s
        elif type_s == "g" or type_s == "G":
            type_p = type_s
        elif type_s == "f" or type_s == "F":
            type_p = type_s
        elif type_s == "a" or type_s == "A":
            type_p = type_s
        elif type_s == "p":
            type_p = "p"
        elif type_s == "c":
            type_p = "c"
        elif type_s == "s":
            type_p = "s"
        else:
            print('UNKNOWN TYPE: {}'.format(type_s))
            exit(1)        

        parts.append("{{:{}{}{}{}}}"
                     .format(fill_and_align, width_s, precision_s, type_p))

    # Append the remaining part of the format string
    parts.append(raw_fmt[pos:])

    return ''.join(parts)
        
def main():

    raw = sys.stdin.read()

    matches = re.finditer(format_re, raw, re.MULTILINE)

    pos = 0
    for match_num, match in enumerate(matches, start = 1):
        start_pos = match.start()
        end_pos = match.end()
        raw_fmt = match.group()
        converted_fmt = convert_format(raw_fmt)
        print("{}{}".format(raw[pos:start_pos], converted_fmt), end="")
        pos = end_pos           

    print("{}".format(raw[pos:]), end="")

if '__main__' == __name__:
    main()
