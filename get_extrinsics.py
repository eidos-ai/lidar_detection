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

# Print out the different source_frames (sensors) and number them
print("Available sensors:")
for i, transform in enumerate(extrinsics['transforms']):
    print(f"{i}: {transform['source_frame']}")

# Prompt the user to select source_frames
selected_indices = input("Enter the indices of the desired sensors (comma-separated): ").split(",")
selected_source_frames = [extrinsics['transforms'][int(idx)]['source_frame'] for idx in selected_indices]

# Initialize the sensor_transformations list
sensor_transformations = []

# Loop over the transforms and include only the selected source_frames
for transform in extrinsics['transforms']:
    if transform["source_frame"] in selected_source_frames:
        sensor_transformation = {
            "name": transform["source_frame"],
            "quaternion": [transform["q_w"], transform["q_x"], transform["q_y"], transform["q_z"]],
            "position": [transform["p_x"], transform["p_y"], transform["p_z"]]
        }
        sensor_transformations.append(sensor_transformation)

# Save the selected sensor_transformations
with open(str(out_file),'w') as f:
    json.dump(sensor_transformations, f, separators=(',', ':'))
print(f"Saved sensor transformations for {selected_source_frames}")