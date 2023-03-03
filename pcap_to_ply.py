import subprocess
import os
from pathlib import Path
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import traceback 
import logging

logging.basicConfig(level=logging.INFO,
                    format= '[%(asctime)s] {%(pathname)s:%(lineno)d} %(levelname)s - %(message)s',
                    datefmt='%H:%M:%S',
                    force=True)

parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument("-p", "--path", type=str, required=True, help="Path to pcap folder")
parser.add_argument("-n", "--scan_num", type=int, required=True, help="Amount of frames to convert (from 0 to n, max 999999)")
args = vars(parser.parse_args())

path_to_pcaps = Path(args["path"])
scan_num = args["scan_num"]

pcap_dirs = os.listdir(path_to_pcaps)
# Iterate through each directory and process the pcap files within
for pcap_dir in pcap_dirs:
    try:
        pcap_basename = os.path.basename(pcap_dir)
        pcap_file = Path(path_to_pcaps, pcap_dir, f"{pcap_basename}.pcap")
        
        # Get list of sensors from json config file name with format {prefix}_{sensor_id}.json
        sensors = [file[file.rfind('_')+1:-5] for file in os.listdir(Path(path_to_pcaps, pcap_dir)) if file.endswith(".json")]
        for sensor in sensors:
            # Create a directory to store the output ply files for this sensor
            dir_per_sensor = Path(path_to_pcaps, pcap_dir, sensor)
            dir_per_sensor.mkdir(exist_ok=True, parents=True)
            
            # Get the full path to the JSON configuration file for this sensor
            json_file = Path(path_to_pcaps, pcap_dir, f"{pcap_basename}_{sensor}.json") 
            
            # Run the command to convert the pcap files to ply files using the ouster SDK
            subprocess.call(["python3", "-m", "ouster.sdk.examples.pcap", pcap_file, json_file, "pcap-to-ply", "--scan-num", str(scan_num)])
            
            # Move the generated ply files to the output directory for this sensor
            os.system(f"mv *.ply {dir_per_sensor}")
            
            # Print a message indicating where the ply files were saved
            logging.info(f"saved ply files to {str(dir_per_sensor)}")
    # If an exception occurs during processing, print an error message and continue processing the next directory
    except Exception as e:
        logging.exception(f"Error processing {pcap_dir}: {e}")
        traceback.print_exc()
        pass

