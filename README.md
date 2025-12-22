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
*   `Visual Proof/`: Contains visual evidence
*   `README.md/`: Instructions on how to build and run

## 7. Visual Proof
Click the screenshot below to watch a short screen recording of the implementation:

[![Watch the video](Visual%20Proof/Screenshot%202025-12-22.png)](https://youtu.be/GLPZsICRgsA)



