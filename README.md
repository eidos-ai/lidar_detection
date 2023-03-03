# PCAP-to-PLY Conversion and Point Cloud Merging
This repo provides a set of scripts for automating the conversion of pcap files to ply format and merging the resulting point clouds from multiple sensors. The scripts are designed to work with data from the Ouster lidar sensor.

## Key Features

- Converts pcap files to ply format
- Merges point clouds from multiple sensors into a single point cloud
- Transforms point clouds to align them based on extrinsics data
- Visualizes the merged point cloud for easy inspection
- Supports integration with the [labelCloud](https://github.com/ch-sa/labelCloud) point cloud labeling tool

![point cloud merge](assets/point_cloud_merge.png)

### File Structure
The following file structure is assumed:
```
pcaps/: path to all pcap recordings
|--- pcap_prefix/: this folder contains 1 pcap file and i json configuration files 
|    |--- pcap_prefix_sensor_id_i.json: configuration file for each lidar sensor
|    |--- pcap_prefix.pcap: pcap recording 
|--- (...)
```

## Setup and Usage
To use the scripts, follow these steps:

1. Install the required dependencies using `$ pip install -r requirements.txt`.
2. Get extrinsics data for each ouster sensor by running: 

    `$ python get_extrinsics.py --user USER --password PSWD -ip 11.22.33.44`

    Note that the name of the sensor, the quaternion and the position vector for each sensor will be saved to `./extrinsics.json` by default (unless  `--outfile` is specified).

3. To use the [pcap_to_ply.py](pcap_to_ply.py) script, run the following command:

    `$ python pcap_to_ply.py --scan_num N --path /path/to/pcaps`

    For each pcap file in the specified path this will:

    - Create a subdirectory for each sensor.
    - Use the Ouster command to convert the pcap file to N ply files, specifying the number of frames N.
    - Move the generated ply files to the corresponding sensor subdirectory.

4. To use the [merge_ply_files.py](merge_ply_files.py) script, run the following command:

    `$ python merge_ply_files.py --sample S --frames N --path /path/to/pcaps --output /path/to/output --visualize`

    This will process each pcap file in the specified path as follows:
    - Randomly sample a specified amount of pcap files from all pcaps in path. 
    - For each pcap, randomly sample a specified amount of ply files (frames) from each sensor to merge. 
    - Transform the point clouds by using the rotation matrix from quaternions and translating the position, imported from `extrinsics.json`.
    - Merge the point clouds and save the result to the output path (default is `./merged_clouds/`).
    - Optionally visualize the merged point cloud. 

5. Load the merged point clouds into labelCloud for further processing.


