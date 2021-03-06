#!/bin/bash

CONTAINER_NAME=$1
if [[ -z "${CONTAINER_NAME}" ]]; then
    CONTAINER_NAME=srcp2
fi

docker stop $(docker ps -aqf "name=^/${CONTAINER_NAME}$")
docker rm $(docker ps -aqf "name=^/${CONTAINER_NAME}$")
