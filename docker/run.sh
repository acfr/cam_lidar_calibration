#!/bin/bash

# Script to run cam_lidar_calibration Docker container using Docker Compose

set -e

# Default settings
CUDA="on"
BUILD="off"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

function usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Run the cam_lidar_calibration development container"
    echo ""
    echo "Options:"
    echo "    -c, --cuda <on|off>      Enable CUDA/GPU support in Docker."
    echo "                             Default: $CUDA"
    echo "    -b, --build              Build the dev image before running."
    echo "                             Default: $BUILD"
    echo "    -r, --distro DISTRO      ROS2 distribution (default: jazzy)"
    echo "    -h, --help               Display this usage and exit."
    echo ""
    echo "Examples:"
    echo "    $0                       # Run with GPU enabled"
    echo "    $0 --cuda off            # Run without GPU"
    echo "    $0 --build               # Build dev image and run"
    echo "    $0 --distro humble       # Run with ROS2 Humble"
}

OPTS=$(getopt --options c:br:h \
         --long cuda:,build,distro:,help \
         --name "$0" -- "$@")

if [ $? != 0 ]; then
    usage
    exit 1
fi

eval set -- "$OPTS"

while true; do
  case "$1" in
    -c|--cuda)
      param=$(echo $2 | tr '[:upper:]' '[:lower:]')
      case "${param}" in
        "on"|"off") CUDA="${param}" ;;
        *) echo "Invalid cuda option: $2"; exit 1 ;;
      esac
      shift 2
      ;;
    -b|--build)
      BUILD="on"
      shift
      ;;
    -r|--distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      if [ ! -z $2 ]; then
        echo "Invalid parameter: $2"
        exit 1
      fi
      break
      ;;
    *)
      echo "Invalid option"
      exit 1
      ;;
  esac
done

# Enable X11 forwarding
xhost +local:docker > /dev/null 2>&1

echo "========================================"
echo "  ROS2 Camera-LiDAR Calibration"
echo "========================================"
echo "Distribution: $ROS_DISTRO"
echo "CUDA/GPU: $CUDA"
echo "Build: $BUILD"
echo ""

# Export ROS_DISTRO for docker-compose
export ROS_DISTRO

# Build dev image if requested
if [ "$BUILD" == "on" ]; then
    echo "Building dev image..."
    ./build.sh --dev --distro $ROS_DISTRO
    if [ $? -ne 0 ]; then
        echo "Build failed!"
        exit 1
    fi
    echo ""
fi

# Check if base and dev images exist
if ! docker image inspect cam_lidar_calibration:base-$ROS_DISTRO >/dev/null 2>&1; then
    echo "ERROR: Base image not found!"
    echo "Please build the base image first:"
    echo "    ./build.sh --all --distro $ROS_DISTRO"
    exit 1
fi

if ! docker image inspect cam_lidar_calibration:dev-$ROS_DISTRO >/dev/null 2>&1; then
    echo "ERROR: Dev image not found!"
    echo "Please build the dev image:"
    echo "    ./build.sh --dev --distro $ROS_DISTRO"
    exit 1
fi

# Create docker-compose override for GPU settings
if [ "$CUDA" == "off" ]; then
    echo "Running without GPU support..."
    cat > docker-compose.override.yml <<EOF
version: '3.8'
services:
  dev:
    deploy:
      resources:
        reservations:
          devices: []
EOF
else
    echo "Running with GPU support..."
    # Remove override file if it exists
    [ -f docker-compose.override.yml ] && rm docker-compose.override.yml
fi

# Start container
echo ""
echo "Starting development container..."
docker compose up -d dev

if [ $? -ne 0 ]; then
    echo "Failed to start container!"
    exit 1
fi

echo ""
echo "✓ Container started successfully!"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Quick Commands"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Enter container:"
echo "  docker compose exec dev bash"
echo ""
echo "Build workspace:"
echo "  docker compose exec dev bash -c 'cd /ros2_ws && colcon build --packages-select cam_lidar_calibration'"
echo ""
echo "Run calibration:"
echo "  docker compose exec dev bash -c 'source install/setup.bash && ros2 launch cam_lidar_calibration run_optimiser.launch.py'"
echo ""
echo "Stop container:"
echo "  docker compose down"
echo ""
echo "View logs:"
echo "  docker compose logs -f dev"
echo ""
