#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
export RESOURCE_NAME="IsaacSim"
export OLD_PYTHONPATH=$PYTHONPATH

# Check args for a flag to disable ROS environment setup
NO_ROS_ENV=false
for arg in "$@"; do
    if [ "$arg" == "--no-ros-env" ]; then
        NO_ROS_ENV=true
        echo "Skipping automatic ROS environment setup"
        break
    fi
done

# Source ROS environment setup script if flag was not found
if [ "$NO_ROS_ENV" == "false" ] && [ -f "$SCRIPT_DIR/setup_ros_env.sh" ]; then
    source "$SCRIPT_DIR/setup_ros_env.sh"
fi

exec "$SCRIPT_DIR/kit/kit" "$SCRIPT_DIR/apps/isaacsim.exp.full.streaming.kit" --no-window  "$@"
