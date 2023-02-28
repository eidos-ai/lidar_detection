import subprocess
import os
from pathlib import Path
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("-p", "--path", default=None, required=True, help="path to pcap folder")
parser.add_argument("-n", "--scan_num", default=None, required=True, help="number of frames from 0 to n")
args = vars(parser.parse_args())

path_to_pcaps = args["path"]
scan_num = args["scan_num"]

pcap_dirs = os.listdir(path_to_pcaps)
for pcap_dir in pcap_dirs:
    pcap_basename = os.path.basename(pcap_dir)
    pcap_file = Path(path_to_pcaps, pcap_dir, f"{pcap_basename}.pcap")
    # get list of sensors 
    sensors = [file[20:-5] for file in os.listdir(Path(path_to_pcaps, pcap_dir)) if file.endswith(".json")] 

    for sensor in sensors:
        # make directory per sensor
        dir_per_sensor = Path(path_to_pcaps, pcap_dir, sensor)
        dir_per_sensor.mkdir(exist_ok=True, parents=True)
        # get json config file path
        json_file = Path(path_to_pcaps, pcap_dir, f"{pcap_basename}_{sensor}.json") 
        # call the command to convert pcap to ply 
        subprocess.call(["python3", "-m", "ouster.sdk.examples.pcap", pcap_file, json_file, "pcap-to-ply", "--scan-num", scan_num])
        # Move the generated PLY files to the output directory
        os.system(f"mv *.ply {dir_per_sensor}")
        print(f"saved ply files to {str(dir_per_sensor)}")
