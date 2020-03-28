#!/bin/bash
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

CONTAINER_NAME=$1
if [[ -z "${CONTAINER_NAME}" ]]; then
    CONTAINER_NAME=src
fi

# This specifies a mapping between a host directory and a directory in the
# docker container. This mapping should be changed if you wish to have access to
# a different directory
HOST_DIR=$2
if [[ -z "${HOST_DIR}" ]]; then
    HOST_DIR="${script_dir}/../"
fi

CONTAINER_DIR=$3
if [[ -z "${CONTAINER_DIR}" ]]; then
    CONTAINER_DIR=/home/user/catkin_ws/src/src
fi

IMAGE_NAME=$4
if [[ -z "${IMAGE_NAME}" ]]; then
    IMAGE_NAME=src
fi

echo "Container name     : ${CONTAINER_NAME}"
echo "Host directory     : ${HOST_DIR}"
echo "Container directory: ${CONTAINER_DIR}"
echo "Image name         : ${IMAGE_NAME}"
CONTAINER_ID=`docker ps -aqf "name=^/${CONTAINER_NAME}$"`
if [ -z "${CONTAINER_ID}" ]; then
    echo "Creating new src docker container."
    xhost +local:root
    nvidia-docker run \
      -it \
      --privileged \
      --network=host \
      -v ${HOST_DIR}:${CONTAINER_DIR}:rw \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      --env="DISPLAY" \
      --env="ROS_MASTER_URI" \
      --env="ROS_HOSTNAME" \
      --env="ROS_IP" \
      --env=QT_X11_NO_MITSHM=1 \
      --tty \
      --device /dev/dri:/dev/dri \
      --name=${CONTAINER_NAME} \
      "${IMAGE_NAME}" \
      bash
else
    echo "Found src docker container: ${CONTAINER_ID}."
    # Check if the container is already running and start if necessary.
    if [ -z `docker ps -qf "name=^/${CONTAINER_NAME}$"` ]; then
        xhost +local:${CONTAINER_ID}
        echo "Starting and attaching to ${CONTAINER_NAME} container..."
        docker start ${CONTAINER_ID}
        docker attach ${CONTAINER_ID}
    else
        echo "Found running ${CONTAINER_NAME} container, attaching bash..."
        docker exec -it ${CONTAINER_ID} bash
    fi
fi
