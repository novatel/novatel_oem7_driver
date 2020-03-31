#!/bin/sh

NAME=ros-kinetic-build
docker build -t $NAME - < docker/Dockerfile.kinetic.build

BLDDIR=/home/build
docker run -ti --mount type=bind,source="$(pwd)",target=$BLDDIR -w $BLDDIR -u `id -u`:`id -g` --group-add sudo --rm $NAME:latest 

