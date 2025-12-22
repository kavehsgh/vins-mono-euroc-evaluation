#!/bin/bash
# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
OUTPUT_FILE="$SCRIPT_DIR/MH_01_easy.bag"
echo "Downloading EuRoC MH_01_easy dataset to $OUTPUT_FILE..."
wget -O "$OUTPUT_FILE" "https://huggingface.co/datasets/kavehsgh/EuRoC_MAV_Dataset_Machine_Hall_Easy_01/resolve/main/MH_01_easy.bag?download=true"
echo "Download complete."
