#!/usr/bin/python3

############################
# Install gspread: pip3 install gspread
#
# Get credentials from service_account.json (see Everett)
# NOTE: THIS FILE MUST BE PUT IN ~/.config/gspread/service_account.json
# See detailed instructions here: 
# https://gspread.readthedocs.io/en/latest/oauth2.html#for-bots-using-service-account
#
# 
# This code appends a row to this Google Sheet (Testing and Data Review):
# https://docs.google.com/spreadsheets/d/10SjGSJwzGYvPNtfCKzTrQJbJoqAz6NRWbUrFP2lcLHk/edit#gid=0
# 
# Inspect the columns in the Test Cases tab for each of the data written below
##########################

import gspread
import argparse
import pdb
import pathlib
import sys
import os
import copy

# ----------------------------------------------------- null-text-help-formatter
# Formatter that doesn't mangle white space in the description
class NullTextHelpFormatter(argparse.RawDescriptionHelpFormatter):
    def add_argument(self, action):
        pass

# --------------------------------------------------------------- parse collated
#
def to_fields(raw_output):
    lines = raw_output.split('\n')
    o = [list(filter(lambda x: len(x) > 0, line.split(' '))) for line in lines]
    return o

def extract_data(fields, name):
    l = list(filter(lambda x: len(x) >= 3 and x[1] == name, fields))
    assert(len(l) == 1)
    return ' '.join(l[0][2:-1])

def extract_tests(fields):
    return [extract_data(fields, 'Date:'),   \
            extract_data(fields, 'Tag:'),    \
            extract_data(fields, 'Commit:'), \
            extract_data(fields, 'FPS:')]

def extract_all_tests(fields):
    return [extract_data(fields, 'Date:'),          \
            extract_data(fields, 'Tag:'),           \
            extract_data(fields, 'Commit:'),        \
            extract_data(fields, 'FPS:'),           \
            extract_data(fields, 'Frames:'),        \
            extract_data(fields, 'Speed:'),         \
            extract_data(fields, 'Avg-precision:'), \
            extract_data(fields, 'Avg-recall:')]

def build_url(name):
    return 'https://s3.console.aws.amazon.com/s3/buckets/perceive-multiview/testcases/{}/?region=us-east-1&tab=overview'.format(name)

def file_get_contents(filename):
    with open(filename) as f:
        return f.read()
    
def get_expected_output(name):
    homedir = pathlib.Path.home()
    fname = '{}/.cache/multiview-data/testcases/{}/expected-output.text'.format(homedir, name)
    if os.path.isfile(fname):
        return file_get_contents(fname).strip()
    return 'file-not-found: {}'.format(fname)

def make_row(one_row):
    if len(one_row) < 10 or one_row[0] != '|' or one_row[1] == 'Testcase':
        return []
    testcase_name = one_row[1]
    precision     = one_row[2]
    recall        = one_row[3]
    f1            = one_row[4]
    gt_tracks     = one_row[5]
    n_tracks      = one_row[6]
    theta_bias    = one_row[7]
    theta_err     = one_row[8]
    testcase_url  = build_url(testcase_name)
    return [testcase_name, precision, recall, f1, \
            get_expected_output(testcase_name), \
            gt_tracks, n_tracks, theta_bias, theta_err, testcase_url]

# ---------------------------------------------------------------- collated data
#
def get_collated_data(testcase_table_string):
    fields    = to_fields(testcase_table_string)
    test_data = extract_tests(fields)
    rows      = list(filter(lambda x: len(x) > 0, \
                            [make_row(one_row) for one_row in fields]))    
    
    def extend_ret(l1, l2):
        l1.extend(l2)
        return l1
    
    return [extend_ret(test_data[:], row) for row in rows]

# ----------------------------------------------------------------- summary data
#
def get_summary_data(testcase_table_string):
    fields    = to_fields(testcase_table_string)
    test_data = extract_all_tests(fields)
    return [test_data]

# ----------------------------------------------- append-testing-and-data-review
#
def append_testing_and_data_review(filename, sh):
    table_data = get_collated_data(file_get_contents(filename))        
    worksheet = sh.worksheet('Test Cases')
    worksheet.append_rows(table_data)

# ---------------------------------------------------------- append-summary-data
#
def append_summary_data(filename, sh):
    table_data = get_summary_data(file_get_contents(filename))
    worksheet = sh.worksheet('Release Results Summary')
    worksheet.append_rows(table_data)
    
# ------------------------------------------------------------------------- main
#
if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        formatter_class=NullTextHelpFormatter,
        usage=argparse.SUPPRESS,
        description="""

Usage: {0} <filename>

    Reads <filename>, which should be a (text) table of data
    produced by 'all-testcases.sh'. This script then parses the
    table into rows of data, which are then uploaded to the
    spreadsheet: 'Testing and Data Review:Test Cases'.

""".format(os.path.basename(sys.argv[0])))

    parser.add_argument('filename', type=str)
    
    # Parse arguments
    args = parser.parse_args()
    
    if not os.path.isfile(args.filename):
        print('Failed to find input file "{}"'.format(args.filename))
        exit(1)

    gc = gspread.service_account()
    sh = gc.open("Testing and Data Review")

    append_testing_and_data_review(args.filename, sh)
    append_summary_data(args.filename, sh)
        

