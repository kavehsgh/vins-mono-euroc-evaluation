# Visual-Inertial SLAM for GPS-Denied Drone Navigation

## 1. Overview
This repository contains a production-ready implementation of a Visual-Inertial Odometry (VIO) pipeline using **VINS-Mono**. The system is designed to estimate the state (position and orientation) of a drone in GPS-denied environments using only a monocular camera and an IMU.

The solution is fully containerized using Docker to ensure consistent performance across different development environments.

**[📄 Read the Full Technical Report (PDF Content)](Technical%20Report/REPORT.md)**

## 2. Dependencies
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

## 7. Visual Proof
Click the screenshot below to watch a short screen recording of the implementation:

[![Watch the video](Visual%20Proof/Screenshot%202025-12-22.png)](https://youtu.be/GLPZsICRgsA)

## 8. Script Details
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

## 9. Complete Configuration Reference
Below is a comprehensive explanation of every parameter found in `config/euroc_tuned.yaml`.

### Common Parameters
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `imu_topic` | `/imu0` | ROS topic name for the IMU stream. |
| `image_topic` | `/cam0/image_raw` | ROS topic name for the camera image stream. |
| `output_path` | `/root/catkin_ws/data/output/` | Path inside the container where results (trajectory CSVs) are saved. |

### Camera Calibration
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `model_type` | `PINHOLE` | The camera projection model. EuRoC uses a standard pinhole model. |
| `camera_name` | `camera` | Identifier for the camera. |
| `image_width` | `752` | Width of the input image in pixels. |
| `image_height` | `480` | Height of the input image in pixels. |
| `distortion_parameters` | `k1, k2, p1, p2` | Radial (`k`) and tangential (`p`) distortion coefficients. Used to undistort images. |
| `projection_parameters` | `fx, fy, cx, cy` | Intrinsic matrix parameters: Focal lengths (`fx`, `fy`) and principal point (`cx`, `cy`). |

### Extrinsic Parameters
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `estimate_extrinsic` | `2` | **0:** Trust provided extrinsics. **1:** Optimize initial guess. **2:** Unknown extrinsics (calibrate online). |
| `extrinsicRotation` | `[Matrix]` | Rotation matrix ($R_{bc}$) from camera frame to IMU frame. |
| `extrinsicTranslation` | `[Vector]` | Translation vector ($t_{bc}$) from camera frame to IMU frame. |

### Feature Tracker Parameters
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `max_cnt` | `150` | Maximum number of features to track per frame. |
| `min_dist` | `30` | Minimum pixel distance between two features to ensure distribution. |
| `freq` | `10` | Frequency (Hz) to publish tracking results to the backend estimator. |
| `F_threshold` | `1.0` | RANSAC threshold (in pixels) for outlier rejection during tracking. |
| `show_track` | `1` | If 1, publishes an image topic showing tracked features (useful for debugging). |
| `equalize` | `1` | If 1, applies histogram equalization to images before processing (helps in low light). |
| `fisheye` | `0` | If 1, applies a mask to remove edge noise (specific to fisheye lenses). |

### Optimization Parameters
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `max_solver_time` | `0.04` | Max time (seconds) allowed for the solver per frame to maintain real-time performance. |
| `max_num_iterations` | `8` | Max number of iterations for the non-linear optimization solver. |
| `keyframe_parallax` | `10.0` | Minimum parallax (pixels) required to select a new keyframe. |

### IMU Parameters
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `acc_n` | `0.08` | Accelerometer white noise density. |
| `gyr_n` | `0.004` | Gyroscope white noise density. |
| `acc_w` | `0.00004` | Accelerometer bias random walk (instability). |
| `gyr_w` | `2.0e-6` | Gyroscope bias random walk (instability). |
| `g_norm` | `9.81007` | Magnitude of gravity in the target environment. |

### Loop Closure Parameters
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `loop_closure` | `1` | If 1, enables the loop closure module (place recognition + pose graph optimization). |
| `load_previous_pose_graph` | `0` | If 1, loads a pre-built map from disk at startup. |
| `fast_relocalization` | `0` | If 1, enables quick relocalization (useful for MAVs re-entering a known map). |
| `pose_graph_save_path` | `...` | Path to save/load the pose graph data. |

### Unsynchronization Parameters
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `estimate_td` | `0` | If 1, estimates the time offset ($t_d$) between camera and IMU online. |
| `td` | `0.0` | Initial guess for the time offset (seconds). |

### Rolling Shutter Parameters
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `rolling_shutter` | `0` | **0:** Global shutter. **1:** Rolling shutter (enables RS model). |
| `rolling_shutter_tr` | `0` | Readout time (seconds) of the rolling shutter sensor. |

### Visualization Parameters
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `save_image` | `1` | If 1, saves images in the pose graph (required for loop closure visualization). |
| `visualize_imu_forward` | `0` | If 1, outputs high-freq IMU propagation for low-latency visualization. |
| `visualize_camera_size` | `0.4` | Scale of the camera wireframe model in Rviz. |



