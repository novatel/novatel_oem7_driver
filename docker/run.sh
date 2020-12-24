#!/bin/sh

set -e

CLEAN=
BUILD_DOCKER=
TYPE=build

print_usage()
{
    echo "$0 -h | [-c][-b] ROS_ARCH ROS_DISTRO"
    echo "  where:"
    echo " -h: display help"
    echo " -r: rebuild of docker container"
    echo " -c: full clean rebuild of docker container, --pull --no-cache"
    echo " -t: container used for testing; only ros-base installed; no driver dependencies installed"
    echo ""
    echo " ROS_ARCH:   i386 | amd64 | arm32v7 | arm64v8" 
    echo " ROS_DISTRO: kinetic | melodic | noetic"
    echo " e.g.:"
    echo "  $0 amd64 noetic"
    echo ""
    echo "Only arch/platform combinations avaliable from 'Docker official ROS images' are supported."  
}

build_docker()
{
    if [ $ROS_ARCH = "i386" ]; then
	# Special docker
        docker build $CLEAN -t $NAME - < docker/Dockerfile.i386.$ROS_DISTRO.build
    else
	# Generic docker
        docker build $CLEAN -t $NAME --build-arg=USR=$TYPE --build-arg=ROS_ARCH=$ROS_ARCH --build-arg=ROS_DISTRO=$ROS_DISTRO - < docker/Dockerfile.build
    fi
}


if [ $# = 0 ]; then
    print_usage
    exit 1
fi

while getopts "hcrt" OPT; do
    case ${OPT} in
        h)
            print_usage
            exit 0
            ;;
        c)
            CLEAN='--pull --no-cache'
            ;;
        r)
            BUILD_DOCKER=build_docker
            ;;
	t)
	    TYPE=test
	    ;;
        *)
            print_usage
            exit 1
            ;;
    esac
done

shift $(($OPTIND - 1))
ROS_ARCH=$1
ROS_DISTRO=$2


NAME=$ROS_ARCH-ros-$ROS_DISTRO-novatel-oem7-driver-$TYPE
$BUILD_DOCKER


CONT_DIR=/home/$TYPE

if [ $TYPE = "build" ]; then
   HOST_DIR=$(pwd)
else
   HOST_DIR=$(pwd)/$ROS_ARCH-$ROS_DISTRO-test
   mkdir -p $HOST_DIR
fi

docker run -ti --mount type=bind,source=$HOST_DIR,target=$CONT_DIR -w $CONT_DIR -u `id -u`:`id -g` --group-add sudo --rm $NAME:latest 

