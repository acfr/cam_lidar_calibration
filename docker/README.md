# Docker Setup for Camera-LiDAR Calibration (ROS2 Jazzy)

This Docker setup provides a complete ROS2 Jazzy environment with perception libraries for running the camera-lidar calibration package.

## Architecture

The setup uses a **two-image approach** for optimal development workflow:

### 1. Base Image (`cam_lidar_calibration:base-jazzy`)
- Contains all system dependencies and ROS2 packages
- Built infrequently (only when dependencies change)
- Large but stable (~4-5GB)
- Based on `osrf/ros:jazzy-desktop-full`

### 2. Development Image (`cam_lidar_calibration:dev-jazzy`)
- Extends the base image
- Lightweight (adds only development scripts)
- Source code is **mounted** from host (not copied)
- Build artifacts persisted in Docker volumes
- Used for daily development

This approach means:
- ✅ Fast rebuilds (dev image rebuilds in seconds)
- ✅ Edit code on host, compile in container
- ✅ Dependencies remain consistent
- ✅ Easy to update only what changes

## Prerequisites

- Docker Engine (>= 20.10)
- Docker Compose (>= 2.0)
- NVIDIA Docker runtime (for GPU support)
- X11 server for GUI applications

### Install NVIDIA Docker Support (Ubuntu)

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

## Quick Start

### Using Makefile (Recommended)

**First Time Setup:**
```bash
cd docker
make build-all  # Builds both base and dev images
```

**Daily Development:**
```bash
make run            # Start container with GPU
make exec           # Enter container
make build-pkg      # Build cam_lidar_calibration
make stop           # Stop container
```

**All Commands:**
```bash
make help           # Show all available commands
```

### Using Shell Scripts Directly

**First Time Setup:**

Build both images (takes 10-15 minutes):

```bash
cd docker
./build.sh --all
```

This builds:
1. Base image with all dependencies
2. Dev image that extends base

### Daily Development Workflow

```bash
# Start the dev container
./run.sh

# In another terminal, enter the container
docker compose exec dev bash

# Inside container: build the package
cd /ros2_ws
colcon build --packages-select cam_lidar_calibration
source install/setup.bash

# Run calibration
ros2 launch cam_lidar_calibration run_optimiser.launch.py
```

## Build Commands

### Build both base and dev images
```bash
./build.sh --all
```

### Build only base image (when dependencies change)
```bash
./build.sh --base
```

### Build only dev image (default, assumes base exists)
```bash
./build.sh --dev
# or simply
./build.sh
```

### Clean rebuild without cache
```bash
./build.sh --all --no-cache
```

## Run Commands

### Start container with GPU support (default)
```bash
./run.sh
```

### Start without GPU
```bash
./run.sh --cuda off
```

### Build dev image and run
```bash
./run.sh --build
```

## Working with the Container

### Enter the container
```bash
docker compose exec dev bash
```

### Build the workspace
Inside container:
```bash
cd /ros2_ws
colcon build --packages-select cam_lidar_calibration
source install/setup.bash
```

Or from host:
```bash
docker compose exec dev bash -c "cd /ros2_ws && colcon build --packages-select cam_lidar_calibration"
```

### Useful aliases (available inside container)
- `build` - Build cam_lidar_calibration package
- `build_all` - Build entire workspace
- `clean` - Remove build/install/log directories
- `src` - Navigate to package source
- `ws` - Navigate to workspace root

## Running the Calibration

```bash
# Launch the calibration tool
ros2 launch cam_lidar_calibration run_optimiser.launch.py

# Or run assessment
ros2 launch cam_lidar_calibration assess_results.launch.py
```

## Docker Compose Commands

### Start container (detached)
```bash
docker compose up -d
```

### Stop container
```bash
docker compose down
```

### View logs
```bash
docker compose logs -f
```

### Rebuild image
```bash
docker compose build --no-cache
```

### Remove volumes (clean build)
```bash
docker compose down -v
```

## Volume Mounts

The following directories are mounted:

| Host Path | Container Path | Purpose |
|-----------|---------------|---------|
| `../` | `/ros2_ws/src/cam_lidar_calibration` | Source code |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 display |
| `~/.Xauthority` | `/root/.Xauthority` | X11 auth |

Build artifacts are stored in Docker volumes for faster rebuilds:
- `cam_lidar_build` → `/ros2_ws/build`
- `cam_lidar_install` → `/ros2_ws/install`
- `cam_lidar_log` → `/ros2_ws/log`

## Customization

### Add Data Directories

Edit `docker-compose.yml` and uncomment:

```yaml
volumes:
  - ${HOME}/datasets:/datasets:rw
```

### Change ROS Domain ID

Edit `docker-compose.yml`:

```yaml
environment:
  - ROS_DOMAIN_ID=42  # Change to your domain
```

### Use Different Base Image

Edit `Dockerfile` first line:

```dockerfile
FROM osrf/ros:jazzy-perception  # Lighter perception-focused image
# OR
FROM osrf/ros:jazzy-desktop     # Standard desktop without extra tools
```

## Troubleshooting

### GUI Applications Not Displaying

```bash
xhost +local:docker
```

### NVIDIA Docker Issues

Check if NVIDIA runtime is available:
```bash
docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi
```

### Permission Issues

If you encounter permission issues with mounted volumes:

```bash
# Add your user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

### Build Cache Issues

Clean rebuild:
```bash
docker compose build --no-cache
docker compose down -v  # Remove volumes
```

## Development Workflow

1. Edit code on host machine (changes reflect immediately in container)
2. Build inside container: `colcon build`
3. Test changes inside container
4. Commit changes from host machine

## Support

For issues specific to the calibration package, see the main README.
For Docker-related issues, check the Docker and Docker Compose documentation.
