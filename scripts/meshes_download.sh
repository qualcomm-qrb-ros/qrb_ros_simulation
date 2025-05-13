#!/bin/bash

# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# Check for required command
if ! command -v curl &> /dev/null; then
    echo "Error: curl is required but not installed. Please install curl first."
    exit 1
fi

declare -A BASE_URLS=(
    ["rml_63_arm"]="https://github.com/RealManRobot/ros2_rm_robot/raw/089cde56dc5ba84f6c465dbf97bb12c216d7c503/rm_description/meshes/rm_63_arm/"
    ["gemini335_336"]="https://github.com/orbbec/OrbbecSDK_ROS2/raw/f29d69ad8f81d0b6914ecb712c98e2a20fab7b92/orbbec_description/meshes/gemini335_336/"
)

current_dir=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
save_path_rml_63="${current_dir}/../qrb_ros_sim_description/meshes/rml_63_gripper_arm"
save_path_gemini335_336="${current_dir}/../qrb_ros_sim_description/meshes/gemini335_336"
# Format: ["BASE_KEY:relative/path/filename"]="local_save_path"
declare -A FILE_MAPPINGS=(
    # rml_63_arm
    ["rml_63_arm:base_link.STL"]="${save_path_rml_63}/base_link.STL"
    ["rml_63_arm:link1.STL"]="${save_path_rml_63}/link1.STL"
    ["rml_63_arm:link2.STL"]="${save_path_rml_63}/link2.STL"
    ["rml_63_arm:link3.STL"]="${save_path_rml_63}/link3.STL"
    ["rml_63_arm:link4.STL"]="${save_path_rml_63}/link4.STL"
    ["rml_63_arm:link5.STL"]="${save_path_rml_63}/link5.STL"
    ["rml_63_arm:link6.STL"]="${save_path_rml_63}/link6.STL"
    # gemini335_336
    ["gemini335_336:camera_bottom_screw_frame.STL"]="${save_path_gemini335_336}/camera_bottom_screw_frame.STL"
    ["gemini335_336:camera_color_frame.STL"]="${save_path_gemini335_336}/camera_color_frame.STL"
    ["gemini335_336:camera_color_optical_frame.STL"]="${save_path_gemini335_336}/camera_color_optical_frame.STL"
    ["gemini335_336:camera_depth_frame.STL"]="${save_path_gemini335_336}/camera_depth_frame.STL"
    ["gemini335_336:camera_depth_optical_frame.STL"]="${save_path_gemini335_336}/camera_depth_optical_frame.STL"
    ["gemini335_336:camera_infra2_frame.STL"]="${save_path_gemini335_336}/camera_infra2_frame.STL"
    ["gemini335_336:camera_infra2_optical_frame.STL"]="${save_path_gemini335_336}/camera_infra2_optical_frame.STL"
    ["gemini335_336:camera_infra_1_frame.STL"]="${save_path_gemini335_336}/camera_infra_1_frame.STL"
    ["gemini335_336:camera_infra_1_optical_frame.STL"]="${save_path_gemini335_336}/camera_infra_1_optical_frame.STL"
    ["gemini335_336:camera_link.STL"]="${save_path_gemini335_336}/camera_link.STL"
)

# Download function with enhanced error handling
download_file() {
    local full_url="$1"
    local save_path="$2"
    
    # Conditional directory creation
    local save_dir=$(dirname "$save_path")
    if [[ ! -d "$save_dir" ]]; then
        echo "Creating directory: $save_dir"
        if ! mkdir -p "$save_dir"; then
            echo "Error: Failed to create directory '$save_dir'"
            return 1
        fi
    fi

    echo "Downloading attempt $attempt: $full_url"
    if curl -# -L --fail --retry 2 --retry-delay 5 "$full_url" -o "$save_path"; then
        echo "Success: Saved to $save_path"
        return 0
    fi

    echo "Error: Failed to download: $full_url"
    return 1
}

# Main execution flow
exit_code=0

for file_key in "${!FILE_MAPPINGS[@]}"; do
    # Parse base URL key and relative path
    IFS=':' read -r base_key relative_path <<< "$file_key"
    save_path="${FILE_MAPPINGS[$file_key]}"
    
    # Validate configuration
    if [[ -z "${BASE_URLS[$base_key]}" ]]; then
        echo "Error: Invalid base URL key '$base_key' in mapping '$file_key'"
        exit_code=1
        continue
    fi
    
    # Skip existing files
    if [[ -f "$save_path" ]]; then
        echo "Status: File exists at '$save_path', skipping download"
        continue
    fi

    # Construct full URL
    base_url="${BASE_URLS[$base_key]}"
    if [[ "$base_url" != */ ]]; then
        base_url+="/"
    fi
    full_url="${base_url}${relative_path}"
    
    # Execute download
    if ! download_file "$full_url" "$save_path"; then
        exit_code=1
    fi
done

exit $exit_code
