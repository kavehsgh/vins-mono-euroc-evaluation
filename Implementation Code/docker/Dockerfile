FROM osrf/ros:noetic-desktop-full

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    nano \
    dos2unix \
    libceres-dev \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Setup Catkin Workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# Clone VINS-Mono
# We clone into src
RUN cd src && \
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git

# Fix OpenCV 4 compatibility issues (VINS-Mono was designed for OpenCV 3)
# 1. Update CMakeLists.txt to use C++14 (often helps with newer libraries)
RUN sed -i 's/set(CMAKE_CXX_FLAGS "-std=c++11")/set(CMAKE_CXX_FLAGS "-std=c++14")/g' src/VINS-Mono/camera_model/CMakeLists.txt
RUN sed -i 's/set(CMAKE_CXX_FLAGS "-std=c++11")/set(CMAKE_CXX_FLAGS "-std=c++14")/g' src/VINS-Mono/feature_tracker/CMakeLists.txt
RUN sed -i 's/set(CMAKE_CXX_FLAGS "-std=c++11")/set(CMAKE_CXX_FLAGS "-std=c++14")/g' src/VINS-Mono/pose_graph/CMakeLists.txt
RUN sed -i 's/set(CMAKE_CXX_FLAGS "-std=c++11")/set(CMAKE_CXX_FLAGS "-std=c++14")/g' src/VINS-Mono/vins_estimator/CMakeLists.txt

# 2. Patch source files for OpenCV 4 constants
# CV_FONT_HERSHEY_SIMPLEX -> cv::FONT_HERSHEY_SIMPLEX
# CV_LOAD_IMAGE_GRAYSCALE -> cv::IMREAD_GRAYSCALE
# CV_BGR2GRAY -> cv::COLOR_BGR2GRAY
RUN find src/VINS-Mono -type f -name "*.cpp" -print0 | xargs -0 sed -i 's/CV_FONT_HERSHEY_SIMPLEX/cv::FONT_HERSHEY_SIMPLEX/g'
RUN find src/VINS-Mono -type f -name "*.cpp" -print0 | xargs -0 sed -i 's/CV_LOAD_IMAGE_GRAYSCALE/cv::IMREAD_GRAYSCALE/g'
RUN find src/VINS-Mono -type f -name "*.cpp" -print0 | xargs -0 sed -i 's/CV_BGR2GRAY/cv::COLOR_BGR2GRAY/g'
RUN find src/VINS-Mono -type f -name "*.cpp" -print0 | xargs -0 sed -i 's/CV_GRAY2BGR/cv::COLOR_GRAY2BGR/g'

# Build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source the workspace in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
