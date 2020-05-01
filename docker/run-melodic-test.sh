#!/bin/sh

NAME=ros-melodic-test
docker build -t $NAME - < docker/Dockerfile.melodic.test

CONT_TESTDIR=/home/test
HOST_TESTDIR=$(pwd)/docker-melodic-test
mkdir -p $HOME_TESTDIR
docker run -ti --mount type=bind,source=$HOST_TESTDIR,target=$CONT_TESTDIR -w $CONT_TESTDIR -u `id -u`:`id -g` --group-add sudo --rm $NAME:latest 

