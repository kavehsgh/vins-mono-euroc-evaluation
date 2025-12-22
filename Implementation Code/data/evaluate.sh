#!/bin/bash
set -e
source /root/catkin_ws/devel/setup.bash

OUTPUT_DIR="/root/catkin_ws/data/output"
mkdir -p $OUTPUT_DIR

echo "----------------------------------------------------------------"
echo "Phase C: Quantitative Analysis & Evaluation"
echo "----------------------------------------------------------------"

# 1. Install Evaluation Tools (evo)
echo "[1/4] Installing evo package..."
apt-get update > /dev/null
apt-get install -y python3-pip python3-tk > /dev/null
pip3 install python-dateutil --upgrade --no-cache-dir > /dev/null
pip3 install evo --upgrade --no-cache-dir > /dev/null

# 2. Run VINS-Mono to generate trajectory (if not already done)
# We run it in background, play bag, then kill it.
echo "[2/4] Running VINS-Mono on MH_01_easy..."
roslaunch vins_estimator euroc.launch config_path:=/root/catkin_ws/config_overrides/euroc_tuned.yaml &
VINS_PID=$!
sleep 5

# Play bag at 2x speed to save time (VINS can usually handle 2x on modern PCs)
rosbag play /root/catkin_ws/data/MH_01_easy.bag -r 2.0

# Wait for VINS to finish processing the buffer
echo "Waiting for VINS to finish processing..."
sleep 15

# Kill VINS
kill $VINS_PID
sleep 2

# 3. Extract Ground Truth from Bag
echo "[3/4] Extracting Ground Truth..."
# Based on rosbag info, the topic is /leica/position
# This topic is geometry_msgs/PointStamped (Position only, no Orientation)
# evo_traj bag can extract this, but ATE usually requires full pose (SE3).
# However, for position-only ATE (APE), evo can handle it if we specify --plot_mode xyz

cd $OUTPUT_DIR
evo_traj bag /root/catkin_ws/data/MH_01_easy.bag /leica/position --save_as_tum
mv leica_position.tum ground_truth.tum

# 4. Evaluate
echo "[4/4] Calculating Metrics..."

# The VINS output is usually named 'vio_loop.csv' or similar in the output folder.
# VINS-Mono outputs: "vins_result_no_loop.csv" (odometry) and "vins_result_loop.csv" (loop closed)
# We will use vins_result_loop.csv if available, else vins_result_no_loop.csv
if [ -f "vins_result_loop.csv" ]; then
    EST_FILE="vins_result_loop.csv"
    echo "Using Loop-Closed Trajectory"
else
    EST_FILE="vins_result_no_loop.csv"
    echo "Using Odometry Trajectory (No Loop Closure)"
fi

# Convert CSV to TUM format using Python script for robustness
# This handles timestamp conversion (ns->s), column reordering, and cleanup
python3 /root/catkin_ws/scripts/convert_results.py "$EST_FILE" "vins_tum.txt"
EST_FILE="vins_tum.txt"

# ATE (Absolute Trajectory Error)
# Note: Since ground truth is position-only (PointStamped), we must use -a (align) and --correct_scale (if monocular scale drift exists, though VINS-Mono + IMU should have metric scale)
# We use --pose_relation trans_part (translation part only) because we don't have GT orientation.
echo "Generating ATE Plot..."
evo_ape tum ground_truth.tum $EST_FILE -va --t_max_diff 0.1 --pose_relation trans_part --plot --plot_mode xyz --save_plot ./ate_plot.png --save_results ./ate_results.zip

# RPE (Relative Pose Error)
echo "Generating RPE Plot..."
evo_rpe tum ground_truth.tum $EST_FILE -va --t_max_diff 0.1 --pose_relation trans_part --plot --plot_mode xyz --save_plot ./rpe_plot.png --save_results ./rpe_results.zip

echo "----------------------------------------------------------------"
echo "Evaluation Complete!"
echo "Results saved in: $OUTPUT_DIR"
echo "----------------------------------------------------------------"
