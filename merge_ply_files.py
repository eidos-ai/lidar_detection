# original code from https://github.com/petefitz19/DigitalTwinOpen3D

import os
import open3d as o3d
import numpy as np
from tqdm import tqdm
import logging 
import re
from pathlib import Path
import random
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter 
import traceback 
import logging
import json 

logging.basicConfig(level=logging.INFO,
                    format= '[%(asctime)s] {%(pathname)s:%(lineno)d} %(levelname)s - %(message)s',
                    datefmt='%H:%M:%S',
                    force=True)

def main(frame, sensor_transformations, pcap_dir, path_to_ply_files_per_sensor, output_dir, visualize):
    """
    Main function to register and merge point clouds.

    Args:
        frame (int): The frame number.
        sensor_transformations (list): A list of dictionaries containing rotation quaternion and 
                                        translation vector for each point cloud.
        pcap_dir (str): The name of the pcap directory.
        path_to_ply_files_per_sensor (list): A list of directories containing the ply files for each sensor.
        output_dir (str): The directory to save the merged point cloud.
        visualize (bool): If True, visualizes the merged point cloud.

    Returns:
        None
    """
    def load_point_clouds(voxel_size=0.0):
        """
        Loads and preprocesses point clouds.

        Args:
            voxel_size (float): Voxel size used for downsampling.

        Returns:
            list: A list of preprocessed point clouds.
        """       
        pcds = []
        filenames = [str(Path(ply_file_path, f"ply_out_{frame}.ply")) for ply_file_path in path_to_ply_files_per_sensor]
        logging.info(f"Files: {filenames}")
        for file in filenames:
            pcd = o3d.io.read_point_cloud(file)
            pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
            pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.02 * 2, max_nn=30))
            pcds.append(pcd_down)

        # Translate or rotate pcd files 
        for i, pcd in enumerate(pcds):
            q = sensor_transformations[i]['quaternion']
            p = sensor_transformations[i]['position']
            R = pcd.get_rotation_matrix_from_quaternion(q)
            pcd = pcd.rotate(R, center=(0, 0, 0)).translate(p)
            pcds[i] = pcd
        return pcds

    def pairwise_registration(source, target):
        """
        Registers a pair of point clouds using point-to-plane ICP.

        Args:
            source (open3d.geometry.PointCloud): The source point cloud.
            target (open3d.geometry.PointCloud): The target point cloud.

        Returns:
            tuple: A tuple containing the transformation matrix and information matrix.
        """
        logging.info("Apply point-to-plane ICP")
        icp_coarse = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance_coarse, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        icp_fine = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance_fine,
            icp_coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        transformation_icp = icp_fine.transformation
        information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            source, target, max_correspondence_distance_fine,
            icp_fine.transformation)
        return transformation_icp, information_icp

    def full_registration(pcds, max_correspondence_distance_coarse,
                        max_correspondence_distance_fine):
        """
        Performs full registration of a list of point clouds using pairwise registration and pose graph optimization.

        Args:
            pcds (list): A list of open3d.geometry.PointCloud objects to register.
            max_correspondence_distance_coarse (float): The maximum correspondence distance for coarse ICP.
            max_correspondence_distance_fine (float): The maximum correspondence distance for fine ICP.

        Returns:
            o3d.pipelines.registration.PoseGraph: A pose graph containing the optimized transformation matrices.
        """
        pose_graph = o3d.pipelines.registration.PoseGraph()
        odometry = np.identity(4)
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
        n_pcds = len(pcds)
        for source_id in range(n_pcds):
            for target_id in range(source_id + 1, n_pcds):
                transformation_icp, information_icp = pairwise_registration(
                    pcds[source_id], pcds[target_id])
                logging.info("Build o3d.pipelines.registration.PoseGraph")
                if target_id == source_id + 1:  # odometry case
                    odometry = np.dot(transformation_icp, odometry)
                    pose_graph.nodes.append(
                        o3d.pipelines.registration.PoseGraphNode(
                            np.linalg.inv(odometry)))
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(
                            source_id,
                            target_id,
                            transformation_icp,
                            information_icp,
                            uncertain=False))
                else:  # loop closure case
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(
                            source_id,
                            target_id,
                            transformation_icp,
                            information_icp,
                            uncertain=True))
        return pose_graph
        
    # Set voxel size for downsampling
    voxel_size = 0.02

    # Load point clouds and downsample using voxelization
    pcds_down = load_point_clouds(voxel_size)

    # Perform full registration using pairwise registration and build pose graph
    max_correspondence_distance_coarse = voxel_size * 150
    max_correspondence_distance_fine = voxel_size * 15
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph = full_registration(pcds_down, max_correspondence_distance_coarse, max_correspondence_distance_fine)

    # Optimize pose graph using global optimization
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=.25,
        reference_node=0)
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option)

    # Concatenate the point clouds
    merged_pcd = o3d.geometry.PointCloud()
    for pcd in pcds_down:
        merged_pcd += pcd

    # Save the merged point cloud
    merged_point_cloud_path = str(Path(output_dir, f"merged_{pcap_dir}_{frame}.ply"))
    o3d.io.write_point_cloud(merged_point_cloud_path, merged_pcd)
    if visualize == True:
        # Read the merged point cloud from file
        merged_cloud = o3d.io.read_point_cloud(merged_point_cloud_path)

        # Create a visualization window
        vis = o3d.visualization.Visualizer()
        vis.create_window()

        # Add the merged point cloud to the visualization
        vis.add_geometry(merged_cloud)

        # Run the visualization
        vis.run()

        # Close the visualization window
        vis.destroy_window()

if __name__ == '__main__': 
    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("-p", "--path", type=str, default=None, required=True, help="Path to all pcaps")
    parser.add_argument("-o", "--output", type=str, default="merged_clouds", required=False, help="Output path to merged ply files")
    parser.add_argument("-f", "--frames", type=int, required=True, help="Amount of ply files (frames) to select randomly from each pcap path")
    parser.add_argument("-s", "--sample", type=int, required=True, help="Amount of pcap files to select randomly from all pcaps")
    parser.add_argument("-e", "--extrinsics", type=str, default="./extrinsics.json",required=False, help="Path to extrinsics file to define sensor_transformations")
    feature_parser = parser.add_mutually_exclusive_group(required=False)
    feature_parser.add_argument('--visualize', dest='visualize', action='store_true')
    feature_parser.add_argument('--no-visualize', dest='visualize', action='store_false')
    parser.set_defaults(feature=True)
    args = vars(parser.parse_args())
    
    # Define sensor transformations from extrinsics file 
    extrinsics_path = Path(args["extrinsics"])
    with open(extrinsics_path) as file:
        sensor_transformations = json.load(file)
    
    # Create output directory
    output_dir = Path(args["output"])
    output_dir.mkdir(exist_ok=True, parents=True)
    
    visualize = args["visualize"]
    path_to_pcaps = args["path"]
    n_pcap_dirs = args["sample"]

    pcap_dirs = os.listdir(path_to_pcaps)
    sample_pcap_dirs = random.sample(pcap_dirs, n_pcap_dirs)

    # Loop over each pcap directory and process each frame
    for pcap_dir in tqdm(sample_pcap_dirs):
        try:
            # Get path to ply files for each sensor
            path_to_ply_files_per_sensor = []
            for count, transformation in enumerate(sensor_transformations):
                path_to_ply_files_per_sensor.append(Path(path_to_pcaps, pcap_dir, transformation['name']))
            
            # Check that each sensor has the same number of ply files and that frame numbers match
            pattern = re.compile(r'^ply_out_(\d{6}).ply$')
            frames = [pattern.match(filename).group(1) for filename in os.listdir(path_to_ply_files_per_sensor[0]) if pattern.match(filename)]
            n_frames = args["frames"]
            sample_frames = random.sample(frames, n_frames)
            
            # Randomly sample frames and process each frame
            for frame in tqdm(sample_frames): 
                main(frame, sensor_transformations, pcap_dir, path_to_ply_files_per_sensor, output_dir, visualize)

        except Exception as e:
            logging.exception(f"Error processing {pcap_dir}: {e}")
            traceback.print_exc()
            pass