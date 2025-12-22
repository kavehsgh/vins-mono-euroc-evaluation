#!/bin/bash
set -e

# Fix line endings for mounted scripts to ensure they run correctly inside Docker
# This handles cases where files are checked out with CRLF on Windows
if [ -d "/root/catkin_ws/data" ]; then
    find /root/catkin_ws/data -name "*.sh" -exec dos2unix {} + 2>/dev/null || true
fi

if [ -d "/root/catkin_ws/scripts" ]; then
    find /root/catkin_ws/scripts -name "*.py" -exec dos2unix {} + 2>/dev/null || true
    find /root/catkin_ws/scripts -name "*.sh" -exec dos2unix {} + 2>/dev/null || true
fi

# Source ROS environment
source "/opt/ros/noetic/setup.bash"
if [ -f "/root/catkin_ws/devel/setup.bash" ]; then
    source "/root/catkin_ws/devel/setup.bash"
fi

exec "$@"
