#!/bin/bash

# Default setting
CUDA="on"

function usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "    -c,--cuda <on|off>           Enable Cuda support in the Docker."
    echo "                                 Default:$CUDA"
    echo "    -h,--help                    Display the usage and exit."
}

OPTS=`getopt --options c:h \
         --long cuda:,help \
         --name "$0" -- "$@"`
eval set -- "$OPTS"

while true; do
  case $1 in
    -c|--cuda)
      param=$(echo $2 | tr '[:upper:]' '[:lower:]')
      case "${param}" in
        "on"|"off") CUDA="${param}" ;;
        *) echo "Invalid cuda option: $2"; exit 1 ;;
      esac
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
	if [ ! -z $2 ];
      then
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

if [ $CUDA == "on" ]; 
then
    ENVS="--env=NVIDIA_VISIBLE_DEVICES=all
	  --env=NVIDIA_DRIVER_CAPABILITIES=all
	  --env=DISPLAY=$DISPLAY
	  --env=QT_X11_NO_MITSHM=1
	  --gpus all"
	echo "Running docker with Cuda support"
else
	ENVS="--env=XAUTHORITY=/home/$(id -un)/.Xauthority
		  --env=ROS_IP=127.0.0.1
		  --env=DISPLAY=$DISPLAY"
	echo "Running docker for cpu"
fi

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
VOLUMES="--volume=$XSOCK:$XSOCK
		 --volume=$XAUTH:/home/$(id -un)/.Xauthority
		 --volume=${PWD}/..:/catkin_ws/src/cam_lidar_calibration"

xhost +local:docker

docker run \
-it --rm \
$VOLUMES \
$ENVS \
--privileged \
--net=host \
--workdir="/catkin_ws/src" \
darrenjkt/cam_lidar_calibration:latest-melodic /bin/bash