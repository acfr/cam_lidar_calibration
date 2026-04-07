# Makefile for cam_lidar_calibration Docker management
# Provides convenient shortcuts for common Docker operations

# Default ROS distribution (can be overridden: make build-all ROS_DISTRO=humble)
ROS_DISTRO ?= jazzy

.PHONY: help build-base build-dev build-all run run-no-gpu stop clean exec logs

# Default target
help:
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@echo "  Camera-LiDAR Calibration Docker Commands"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@echo ""
	@echo "Current ROS Distribution: $(ROS_DISTRO)"
	@echo "(Override with: make <target> ROS_DISTRO=humble)"
	@echo ""
	@echo "Build Commands:"
	@echo "  make build-all      Build both base and dev images"
	@echo "  make build-base     Build only base image (dependencies)"
	@echo "  make build-dev      Build only dev image"
	@echo ""
	@echo "Run Commands:"
	@echo "  make run            Start dev container with GPU"
	@echo "  make run-no-gpu     Start dev container without GPU"
	@echo ""
	@echo "Container Management:"
	@echo "  make exec           Enter running container"
	@echo "  make stop           Stop and remove containers"
	@echo "  make logs           View container logs"
	@echo ""
	@echo "Maintenance:"
	@echo "  make clean          Remove all containers and volumes"
	@echo "  make rebuild        Clean rebuild of all images"
	@echo ""

# Build targets
build-all:
	@./build.sh --all --distro $(ROS_DISTRO)

build-base:
	@./build.sh --base --distro $(ROS_DISTRO)

build-dev:
	@./build.sh --dev --distro $(ROS_DISTRO)

rebuild:
	@./build.sh --all --no-cache --distro $(ROS_DISTRO)

# Run targets
run:
	@ROS_DISTRO=$(ROS_DISTRO) ./run.sh

run-no-gpu:
	@ROS_DISTRO=$(ROS_DISTRO) ./run.sh --cuda off

# Container management
exec:
	@ROS_DISTRO=$(ROS_DISTRO) docker compose exec dev bash

stop:
	@ROS_DISTRO=$(ROS_DISTRO) docker compose down

logs:
	@ROS_DISTRO=$(ROS_DISTRO) docker compose logs -f dev

# Cleaning
clean:
	@echo "Stopping containers..."
	@ROS_DISTRO=$(ROS_DISTRO) docker compose down -v
	@echo "Removing images..."
	@docker rmi cam_lidar_calibration:dev-$(ROS_DISTRO) 2>/dev/null || true
	@docker rmi cam_lidar_calibration:base-$(ROS_DISTRO) 2>/dev/null || true
	@echo "Cleanup complete!"

# Workspace operations (executed inside container)
build-pkg:
	@ROS_DISTRO=$(ROS_DISTRO) docker compose exec dev bash -c "cd /ros2_ws && colcon build --packages-select cam_lidar_calibration --symlink-install"

build-ws:
	@ROS_DISTRO=$(ROS_DISTRO) docker compose exec dev bash -c "cd /ros2_ws && colcon build --symlink-install"

clean-ws:
	@ROS_DISTRO=$(ROS_DISTRO) docker compose exec dev bash -c "cd /ros2_ws && rm -rf build install log"

# Launch operations
launch-calibration:
	@ROS_DISTRO=$(ROS_DISTRO) docker compose exec dev bash -c "source /ros2_ws/install/setup.bash && ros2 launch cam_lidar_calibration run_optimiser.launch.py"

launch-assess:
	@ROS_DISTRO=$(ROS_DISTRO) docker compose exec dev bash -c "source /ros2_ws/install/setup.bash && ros2 launch cam_lidar_calibration assess_results.launch.py"
