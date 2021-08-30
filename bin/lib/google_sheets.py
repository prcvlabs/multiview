############################
# Install gspread: pip3 install gspread
#
# Get credentials from service_account.json (file should be sent with this example code)
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

gc = gspread.service_account()

sh = gc.open("Testing and Data Review")
worksheet = sh.worksheet('Test Cases')

row1 = [
    '42069_perceive-office', 
    'standing around', 
    '07/30/2020', 
    'P>.95 + R>.95 + F1>.9', 
    '5.1.0', 
    'P=1 + R=1 + F1=1',
    1,
    'Note about running test',
    'https://s3.console.aws.amazon.com/s3/buckets/perceive-multiview/testcases/008_cal-closets-standing/',
    'Tracks subject to reprojection error issues.',
]

row2 = [
    '42070_perceive-office', 
    'standing around', 
    '07/30/2020', 
    'P>.95 + R>.95 + F1>.9', 
    '5.1.0', 
    'P=1 + R=1 + F1=1',
    1,
    'Note about running test',
    'https://s3.console.aws.amazon.com/s3/buckets/perceive-multiview/testcases/008_cal-closets-standing/',
    'Tracks subject to reprojection error issues.',
]

# This API call takes a list of lists
# You can see the API reference here:
# https://gspread.readthedocs.io/en/latest/api.html#gspread.models.Worksheet.append_rows
worksheet.append_rows([row1, row2])
