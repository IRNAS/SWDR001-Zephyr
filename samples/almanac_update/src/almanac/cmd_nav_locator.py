"""
This python script will take nav payload from lr1110 chip, assistance location
and time in gps format. It will make post request to lora cloud server and in case of correct parameters it will return location.
Working example:
python3 cmd_nav_locator.py  --nav 010106D9430816a202a82aeb61df9b2d5031d5367d072aa05209c0c7ae7651d5b8e7203c09a3caba8ea4bf8c8c16558652c17787636a776c44f9294cd5a54356ff4900 --lat 46.452761 --lon 15.641854 --gpstime 1271763033
python cmd_nav_locator.py --nav 01763C430816622AA915764B45040B541B0C08CE182A0D0AB042849AFAD0B862C501721516322C8908 --lat 46.452761 --lon 15.641854 --gpstime 1281954911
"""

import json
import os
import sys
import argparse
from argparse import RawTextHelpFormatter
import requests

URL = 'https://gls.loracloud.com/api/v3/solve/gnss_lr1110_singleframe'
AUTH_TOKEN = 'AQEAdv9jq5dthzVHYZMRxDEg6WRUIaz590FXPuAfP+1jddncGODv'

### Command line argument parser

# Intro message
intro = "This is NAV locator!\n\n"
intro += "Input NAV payload, assistance position and time in gps format to receive position"# Parser setup
parser = argparse.ArgumentParser(description=intro, formatter_class=RawTextHelpFormatter)
parser.add_argument("--nav", type=str,required=True, help="This is nav payload, make sure that is in hexadecimal form")
parser.add_argument("--lat", type=str,required=True, help="This is latitude coordinate of assistance position")
parser.add_argument("--lon", type=str,required=True, help="This is longitude coordinate of assistance position")
parser.add_argument("--gpstime", type=str,required=True, help="This is GSP time in epoch format")

# Display help message if no arguments are provided
if len(sys.argv)== 1:
    parser.print_help(sys.stderr)
    sys.exit(1)
args = parser.parse_args()

#print("This is payload" + nav)
json_dict = {
        "payload": args.nav,
        # Duration is in milliseconds, we change it to seconds
        "gnss_capture_time": int(args.gpstime),
        "gnss_capture_time_accuracy": 15,   # Perl script ima 15
        "gnss_assist_position" : [float(args.lat), float(args.lon)]
        }
response = requests.post(URL,
        json=json_dict,
        headers = {'Accept':'application/json',
            'Ocp-Apim-Subscription-Key': AUTH_TOKEN})
print()
print("Response from lora cloud:")

json_content = response.json()
if json_content["result"] == None:
    print("No results, error:")
    print(json_content["errors"])
    print()
    state = "unknown"
else:
    print("Results available:")

    lat = json_content["result"]["llh"][0]
    lon = json_content["result"]["llh"][1]
    alt = json_content["result"]["llh"][2]
    print("Latitude: " + str(lat))
    print("Longitude: " + str(lon))
    print("Altitude: " + str(alt))