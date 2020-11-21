#!/bin/sh

set -e

CLEAN=
BUILD_DOCKER=

print_usage()
{
    echo "$0 -h | [-c][-b] ROS_DISTRO"
    echo "  where:"
    echo " h: display help"
    echo " b: rebuild of docker container"
    echo " c: full clean rebuild of docker container, --pull --no-cache"
    echo " ROS_DISTRO: kinetic | kinetic.i386 | melodic | noetic"
    echo " e.g.:"
    echo "  $0 melodic"  
}

build_docker()
{
   docker build $CLEAN -t $NAME - < docker/Dockerfile.$DISTRO.build
}


if [ $# = 0 ]; then
    print_usage
    exit 1
fi

while getopts "hcb" OPT; do
    case ${OPT} in
        h)
            print_usage
            exit 0
            ;;
        c)
            CLEAN='--pull --no-cache'
            ;;
        b)
            BUILD_DOCKER=build_docker
            ;;
        *)
            print_usage
            exit 1
            ;;
    esac
done

shift $(($OPTIND -1))
DISTRO=$1
NAME=ros-$DISTRO-novatel-oem7-driver-build
BLDDIR=/home/build

$BUILD_DOCKER

docker run -ti --mount type=bind,source="$(pwd)",target=$BLDDIR -w $BLDDIR -u `id -u`:`id -g` --group-add sudo --rm $NAME:latest 

