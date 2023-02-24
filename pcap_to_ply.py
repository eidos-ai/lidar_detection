import subprocess
import os
from pathlib import Path
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("-p", "--path", default=None, required=True, help="path to pcap folder")
parser.add_argument("-n", "--scan_num", default=None, required=True, help="number of frames from 0 to n")
args = vars(parser.parse_args())

pcap_dir = args["path"]
scan_num = args["scan_num"]

pcap_basename = os.path.basename(pcap_dir)
pcap_file = f"{pcap_dir}/{pcap_basename}.pcap"
# get list of sensors 
sensors = [file[20:-5] for file in os.listdir(pcap_dir) if file.endswith(".json")] 

for sensor in sensors:
    # make directory per sensor
    os.system(f"mkdir {pcap_dir}/{sensor}")
    # move json config to sensor dir
    os.system(f"cp {pcap_dir}/{pcap_basename}_{sensor}.json {pcap_dir}/{sensor}")

    json_file = f"{pcap_dir}/{sensor}/{pcap_basename}_{sensor}.json"
    # call the command to convert pcap to ply 
    subprocess.call(["python3", "-m", "ouster.sdk.examples.pcap", pcap_file, json_file, "pcap-to-ply", "--scan-num", scan_num])
    # Move the generated PLY files to the output directory
    os.system(f"mv *.ply {pcap_dir}/{sensor}")
    print(f"saved ply files to {pcap_dir}/{sensor}/")


# use python3 pcap_to_ply.py --scan_num 5 to run 