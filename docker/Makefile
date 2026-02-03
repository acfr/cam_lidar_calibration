# Makefile for cam_lidar_calibration Docker management
# Provides convenient shortcuts for common Docker operations

.PHONY: help build-base build-dev build-all run run-no-gpu stop clean exec logs

# Default target
help:
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@echo "  Camera-LiDAR Calibration Docker Commands"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
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
	@./build.sh --all

build-base:
	@./build.sh --base

build-dev:
	@./build.sh --dev

rebuild:
	@./build.sh --all --no-cache

# Run targets
run:
	@./run.sh

run-no-gpu:
	@./run.sh --cuda off

# Container management
exec:
	@docker compose exec dev bash

stop:
	@docker compose down

logs:
	@docker compose logs -f dev

# Cleaning
clean:
	@echo "Stopping containers..."
	@docker compose down -v
	@echo "Removing images..."
	@docker rmi cam_lidar_calibration:dev-jazzy 2>/dev/null || true
	@docker rmi cam_lidar_calibration:base-jazzy 2>/dev/null || true
	@echo "Cleanup complete!"

# Workspace operations (executed inside container)
build-pkg:
	@docker compose exec dev bash -c "cd /ros2_ws && colcon build --packages-select cam_lidar_calibration --symlink-install"

build-ws:
	@docker compose exec dev bash -c "cd /ros2_ws && colcon build --symlink-install"

clean-ws:
	@docker compose exec dev bash -c "cd /ros2_ws && rm -rf build install log"

# Launch operations
launch-calibration:
	@docker compose exec dev bash -c "source /ros2_ws/install/setup.bash && ros2 launch cam_lidar_calibration run_optimiser.launch.py"

launch-assess:
	@docker compose exec dev bash -c "source /ros2_ws/install/setup.bash && ros2 launch cam_lidar_calibration assess_results.launch.py"
