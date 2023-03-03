import requests
from requests.auth import HTTPBasicAuth
import json
from pathlib import Path
from urllib3.exceptions import InsecureRequestWarning
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter 
requests.packages.urllib3.disable_warnings(category=InsecureRequestWarning)

parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("-u", "--user", type=str, default=None, required=True, help="user for ouster gemini")
parser.add_argument("-p", "--password", type=str, required=True, help="password for ouster gemini")
parser.add_argument("-ip",type=str, required=True, help="ip address for ouster gemini connection")
parser.add_argument("-o", "--outfile", type=str, default = "./extrinsics.json", required=False, help="path to sensor transformations output file")
args = vars(parser.parse_args())

user = args["user"] 
password = args["password"] 
ip = args["ip"] 
out_file = Path(args["outfile"]) 

auth = HTTPBasicAuth(user, password)
extrinsics = json.loads(requests.get(f"https://{ip}/perception/api/v1/extrinsics", verify=False, auth=auth).text)

"""
##### example response #####

"transforms": [
  {
    "destination_frame": "world",
    "p_x": 43.73017883300781,
    "p_y": -2.257012367248535,
    "p_z": 7.608055591583252,
    "q_w": 0.2536286413669586,
    "q_x": 0.019166436046361923,
    "q_y": 0.9670660495758057,
    "q_z": 0.009399726055562496,
    "source_frame": "122216001766"
  }
]

"""
sensor_transformations = []
for transform in extrinsics['transforms']:
    sensor_transformation = {}
    # change condition for sensors you want to use
    if transform["source_frame"] == "122216001766" or transform["source_frame"] == "122222002441": 
        sensor_transformation = {
            "name": transform["source_frame"],
            "quaternion": [transform["q_w"],transform["q_x"],transform["q_y"],transform["q_z"]],
            "position": [transform["p_x"],transform["p_y"],transform["p_z"]]
        }
        sensor_transformations.append(sensor_transformation) 

with open(str(out_file),'w') as f:
    json.dump(sensor_transformations, f, separators=(',', ':'))