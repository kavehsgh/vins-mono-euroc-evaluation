# Visual-Inertial SLAM for GPS-Denied Drone Navigation

## 1. Overview
This repository contains a production-ready implementation of a Visual-Inertial Odometry (VIO) pipeline using **VINS-Mono**. The system is designed to estimate the state (position and orientation) of a drone in GPS-denied environments using only a monocular camera and an IMU.

The solution is fully containerized using Docker to ensure consistent performance across different development environments.

**[📄 Read the Full Technical Report (PDF Content)](Technical%20Report/REPORT.md)**

## 2. Prerequisites
To run this project, you need the following installed on your system:

*   **Docker Engine (v19+)**
*   **NVIDIA Container Toolkit** (optional but recommended for GPU acceleration)
*   **X Server** (For visualization via Rviz)
    *   **Windows:** [VcXsrv (XLaunch)](https://sourceforge.net/projects/vcxsrv/)
    *   **Linux:** Native Xorg/Wayland (usually pre-installed)
    *   **macOS:** [XQuartz](https://www.xquartz.org/)

## 3. Build & Run Instructions

### Option A: Windows (Recommended)
We provide a one-click script for Windows users (requires WSL2 backend for Docker).

1.  **Start VcXsrv (XLaunch)**:
    *   Select "Multiple windows".
    *   Select "Start no client".
    *   **Crucial:** Check **"Disable access control"** on the final settings page.
2.  **Run the Script**:
    Navigate to the `Implementation Code` folder and double-click `run_windows.bat` or run it from PowerShell:
    ```powershell
    cd "Implementation Code"
    .\run_windows.bat
    ```
    *This will build the Docker image and drop you into the container shell.*

### Option B: Linux (Ubuntu/Debian)
We provide a helper script that handles X11 forwarding permissions automatically.

1.  **Navigate to the implementation folder:**
    ```bash
    cd "Implementation Code"
    ```
2.  **Make the script executable and run:**
    ```bash
    chmod +x run_linux.sh
    ./run_linux.sh
    ```

### Option C: macOS (Intel/M1)
macOS requires **XQuartz** to handle the display.
1.  Open XQuartz settings -> Security -> Check "Allow connections from network clients".
2.  Run:
    ```bash
    cd "Implementation Code"
    xhost + 127.0.0.1
    docker build -t vins_mono_node docker/
    docker run -it --rm \
        -e DISPLAY=host.docker.internal:0 \
        -v $(pwd)/data:/root/catkin_ws/data \
        -v $(pwd)/config:/root/catkin_ws/config_overrides \
        -v $(pwd)/scripts:/root/catkin_ws/scripts \
        vins_mono_node bash
    ```

## 4. Running the Demo
Once you are inside the Docker container shell (`root@...`), follow these steps:

### Step 1: Download the Dataset
We use the **EuRoC MAV Dataset (MH_01_easy)** for validation.
```bash
bash data/download_dataset.sh
```

### Step 2: Run VINS-Mono
This script launches the estimator, the Rviz visualization, and plays the dataset bag file.
```bash
bash data/run_demo.sh
```
*You should see the Rviz window appear on your host machine showing the drone's trajectory.*

## 5. Evaluation
To reproduce the quantitative results (ATE/RMSE) presented in the report:

```bash
bash data/evaluate.sh
```
*   This script runs the pipeline in the background (at 2x speed).
*   It compares the estimated trajectory against Ground Truth using the `evo` package.
*   **Outputs:** Results and plots are saved to the `data/output/` directory on your host machine.

## 6. Repository Structure

*   `Implementation Code/`: Contains all source code, scripts, and configuration files.
    *   `docker/`: Dockerfile and entrypoint scripts.
    *   `config/`: Custom YAML configuration files.
    *   `data/`: Helper scripts and datasets.
    *   `data/output/`: Contains the evaluation results and plots.

*   `Technical Report/`: Contains the project report.
    *   `REPORT.md`: The full technical report.
    *   `REPORT.pdf`: The full technical report in PDF format.
*   `Visual Proof/`: Contains visual evidence.
*   `README.md/`: Instructions on how to build and run.

## 7. Script Details
Here is a detailed breakdown of the helper scripts provided in the `data/` folder:

### `data/download_dataset.sh`
*   **Purpose:** Automates the retrieval of the testing data.
*   **Action:** Downloads the **EuRoC MH_01_easy** dataset (Machine Hall 01) from a cloud mirror and saves it as `MH_01_easy.bag`. This ensures every user tests on the exact same data.

### `data/run_demo.sh`
*   **Purpose:** Visual demonstration of the system in action.
*   **Action:**
    1.  Starts the **VINS-Mono Estimator** node with our tuned configuration (`euroc_tuned.yaml`).
    2.  Launches **Rviz** (ROS Visualization) pre-configured to show the point cloud, camera pose, and trajectory.
    3.  Plays the rosbag file to simulate a live camera feed.
    4.  Automatically cleans up all processes when the dataset finishes.

### `data/evaluate.sh`
*   **Purpose:** Quantitative analysis for the technical report.
*   **Action:**
    1.  Installs the **evo** Python package (a standard tool for SLAM evaluation).
    2.  Runs VINS-Mono in the background (without GUI) to generate a trajectory file (`vins_result_loop.csv`).
    3.  Extracts the ground truth position data from the bag file.
    4.  Calculates the **Absolute Trajectory Error (ATE)** by aligning the estimated path with the ground truth.
    5.  Generates error plots and statistics in the `data/output/` folder.

### `scripts/convert_results.py`
*   **Purpose:** Data format conversion helper.
*   **Action:**
    *   Converts the raw CSV output from VINS-Mono into the standard **TUM trajectory format** required by the `evo` evaluation tool.
    *   **Transformations:**
        *   Converts timestamps from nanoseconds to seconds.
        *   Reorders quaternion components from `[w, x, y, z]` to `[x, y, z, w]`.
        *   Changes the delimiter from comma to space.

## 8. Configuration Details (`config/euroc_tuned.yaml`)
This file defines the critical parameters for the VINS-Mono estimator. Below is a detailed breakdown of why specific values were chosen for this evaluation project:

### 1. Sensor & I/O
| Parameter | Value | Purpose & Project Context |
| :--- | :--- | :--- |
| `imu_topic` | `/imu0` | Matches the IMU topic in the EuRoC dataset bag files. |
| `image_topic` | `/cam0/image_raw` | Matches the left camera topic. We use monocular mode, so only one camera is needed. |
| `output_path` | `/root/catkin_ws/data/output/` | Maps to the `Implementation Code/data/output` folder on your host machine via Docker volumes, allowing you to access results without entering the container. |

### 2. Extrinsic Parameters (Camera-IMU)
| Parameter | Value | Purpose & Project Context |
| :--- | :--- | :--- |
| `estimate_extrinsic` | `2` | **Crucial for Robustness.** We set this to "Unknown" to force the system to calibrate the Camera-to-IMU transformation online. This demonstrates VINS-Mono's ability to recover from mechanical changes without offline calibration tools. |

### 3. Feature Tracking
| Parameter | Value | Purpose & Project Context |
| :--- | :--- | :--- |
| `max_cnt` | `150` | **Performance Balance.** Tracks up to 150 features. This is high enough for accurate motion estimation but low enough to run in real-time (20Hz+) inside the Docker container. |
| `min_dist` | `30` | **Feature Distribution.** Enforces a minimum pixel distance between features. This prevents features from bunching up in one high-contrast area, ensuring the estimator can detect rotation vs. translation effectively. |
| `equalize` | `1` (True) | **Lighting Robustness.** The "Machine Hall" environment has dark corners. Histogram equalization enhances contrast, allowing the system to track features even in poor lighting conditions. |

### 4. Optimization & Keyframes
| Parameter | Value | Purpose & Project Context |
| :--- | :--- | :--- |
| `max_solver_time` | `0.04` (40ms) | **Real-Time Guarantee.** Limits the optimization time per frame. If the solver takes longer, it terminates early to prevent lag, ensuring the drone doesn't "fly blind." |
| `keyframe_parallax` | `10.0` | **Triangulation Accuracy.** Only creates a new keyframe if the camera has moved enough (10 pixels parallax). This ensures that 3D points are triangulated with a wide enough baseline to be accurate. |

### 5. IMU Noise Modeling (ADIS16448)
| Parameter | Value | Purpose & Project Context |
| :--- | :--- | :--- |
| `acc_n` / `gyr_n` | `0.08` / `0.004` | **Sensor Specific.** These represent the "white noise" of the accelerometer and gyroscope. We use slightly inflated values compared to the datasheet to account for vibrations on the MAV. |
| `acc_w` / `gyr_w` | `0.00004` / `2.0e-6` | **Bias Instability.** Models how the IMU bias drifts over time. Correctly modeling this prevents the estimated trajectory from curling or drifting during slow motion. |

### 6. Loop Closure
| Parameter | Value | Purpose & Project Context |
| :--- | :--- | :--- |
| `loop_closure` | `1` (True) | **Drift Correction.** Enables the system to recognize previously visited locations. When a loop is detected, the system optimizes the entire pose graph, snapping the trajectory back to consistency. This is vital for achieving the low ATE (Absolute Trajectory Error) shown in the report. |

## 9. Visual Proof
Click the screenshot below to watch a short screen recording of the implementation:

[![Watch the video](Visual%20Proof/Screenshot%202025-12-22.png)](https://youtu.be/GLPZsICRgsA)



