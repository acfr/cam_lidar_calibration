#!/bin/bash

# Build script for cam_lidar_calibration Docker images
# Supports building base image (with all dependencies) and dev image separately

set -e

# Default settings
BUILD_BASE="off"
BUILD_DEV="on"
NO_CACHE=""
ROS_DISTRO="jazzy"

function usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Build Docker images for camera-lidar calibration"
    echo ""
    echo "Options:"
    echo "    -b, --base              Build only the base image with dependencies"
    echo "    -d, --dev               Build only the dev image (default)"
    echo "    -a, --all               Build both base and dev images"
    echo "    -n, --no-cache          Build without using cache"
    echo "    -r, --distro DISTRO     ROS2 distribution (default: jazzy)"
    echo "    -h, --help              Display this help message"
    echo ""
    echo "Examples:"
    echo "    $0                      # Build dev image (assumes base exists)"
    echo "    $0 --all                # Build both base and dev images"
    echo "    $0 --base               # Build only base image"
    echo "    $0 --all --no-cache     # Clean build of both images"
    echo "    $0 --all --distro humble # Build for ROS2 Humble"
}

OPTS=$(getopt --options bdanr:h \
         --long base,dev,all,no-cache,distro:,help \
         --name "$0" -- "$@")

if [ $? != 0 ]; then
    usage
    exit 1
fi

eval set -- "$OPTS"

while true; do
  case "$1" in
    -b|--base)
      BUILD_BASE="on"
      BUILD_DEV="off"
      shift
      ;;
    -d|--dev)
      BUILD_DEV="on"
      BUILD_BASE="off"
      shift
      ;;
    -a|--all)
      BUILD_BASE="on"
      BUILD_DEV="on"
      shift
      ;;
    -n|--no-cache)
      NO_CACHE="--no-cache"
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
      shift
      break
      ;;
    *)
      echo "Invalid option"
      exit 1
      ;;
  esac
done

echo "========================================"
echo "  Building cam_lidar_calibration Images"
echo "========================================"
echo ""
echo "ROS2 Distribution: $ROS_DISTRO"
echo ""

# Build base image
if [ "$BUILD_BASE" == "on" ]; then
    echo "Building BASE image with all dependencies..."
    echo "This may take 10-15 minutes on first build..."
    echo ""
    
    docker build $NO_CACHE \
        --build-arg ROS_DISTRO=$ROS_DISTRO \
        -f Dockerfile \
        -t cam_lidar_calibration:base-$ROS_DISTRO \
        .
    
    if [ $? -ne 0 ]; then
        echo "ERROR: Base image build failed!"
        exit 1
    fi
    
    echo ""
    echo "✓ Base image built successfully: cam_lidar_calibration:base-$ROS_DISTRO"
    echo ""
fi

# Build dev image
if [ "$BUILD_DEV" == "on" ]; then
    # Check if base image exists
    if ! docker image inspect cam_lidar_calibration:base-$ROS_DISTRO >/dev/null 2>&1; then
        echo "ERROR: Base image not found!"
        echo "Please build the base image first with:"
        echo "    $0 --base --distro $ROS_DISTRO"
        echo "or build both with:"
        echo "    $0 --all --distro $ROS_DISTRO"
        exit 1
    fi
    
    echo "Building DEV image (extends base)..."
    echo ""
    
    docker build $NO_CACHE \
        -f Dockerfile.dev \
        --build-arg BASE_IMAGE=cam_lidar_calibration:base-$ROS_DISTRO \
        -t cam_lidar_calibration:dev-$ROS_DISTRO \
        .
    
    if [ $? -ne 0 ]; then
        echo "ERROR: Dev image build failed!"
        exit 1
    fi
    
    echo ""
    echo "✓ Dev image built successfully: cam_lidar_calibration:dev-$ROS_DISTRO"
    echo ""
fi

echo "========================================"
echo "  Build Complete!"
echo "========================================"
echo ""

if [ "$BUILD_BASE" == "on" ]; then
    echo "Base image: cam_lidar_calibration:base-$ROS_DISTRO"
    echo "  - Contains all ROS2 and system dependencies"
    echo "  - Rebuild only when dependencies change"
    echo ""
fi

if [ "$BUILD_DEV" == "on" ]; then
    echo "Dev image: cam_lidar_calibration:dev-$ROS_DISTRO"
    echo "  - Extends base image"
    echo "  - Mounts source code for development"
    echo ""
fi

echo "Next steps:"
echo "  1. Start the dev container:"
echo "     ./run.sh"
echo ""
echo "  2. Enter the container:"
echo "     docker compose exec dev bash"
echo ""
echo "  3. Build the workspace:"
echo "     colcon build --packages-select cam_lidar_calibration"
