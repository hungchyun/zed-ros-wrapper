#!/bin/bash

while getopts i:o: flag
do
    case "${flag}" in
        i) input=${OPTARG};;
        o) output=${OPTARG};;
    esac
done
echo "Input directory (use -i /input_directory): $input";
echo "Out directory (use -o /output_directory): $output";

rosbag filter $input $output "topic == '/zed/zed_node/imu/data' or \
                              topic == '/zed/zed_node/odom' or \
                              topic == '/zed/zed_node/left/image_rect_color' or \
                              topic == '/zed/zed_node/depth/depth_registered' or \
                              topic == '/zed/zed_node/left/camera_info' or \
                              topic == '/zed/zed_node/depth/camera_info' or \
                              topic == '/zed/zed_node/point_cloud/cloud_registered'"