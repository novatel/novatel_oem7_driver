#!/bin/sh

NAME=ros-kinetic-test
docker build -t $NAME - < docker/Dockerfile.kinetic.test

CONT_TESTDIR=/home/test
HOST_TESTDIR=$(pwd)/docker-kinetic-test
mkdir -p $HOST_TESTDIR
docker run -ti --mount type=bind,source=$HOST_TESTDIR,target=$CONT_TESTDIR -w $CONT_TESTDIR -u `id -u`:`id -g` --group-add sudo --rm $NAME:latest 

