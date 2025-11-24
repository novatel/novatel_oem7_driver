#!/bin/sh

set -e

CLEAN=
BUILD_DOCKER=
TYPE=build

HOST_USERNAME=$(id -un)
HOST_UID=$(id -u)
HOST_GROUPNAME=$(id -gn)
HOST_GID=$(id -g)

print_usage()
{
    echo "$0 -h | [-c][-b] ROS_ARCH ROS_DISTRO"
    echo "  where:"
    echo " -h: display help"
    echo " -r: rebuild of docker container"
    echo " -c: full clean rebuild of docker container, --pull --no-cache"
    echo " -t: container used for testing; only ros-base installed; no driver dependencies installed"
    echo ""
    echo " ROS_ARCH:   amd64 | arm64v8" 
    echo " ROS_DISTRO: kilted"
    echo " e.g.:"
    echo "  $0 amd64 kilted"
    echo "  $0 arm64 kilted"
    echo ""
    echo "Only arch/platform combinations avaliable from OSRF are supported."  
}

build_docker()
{
    docker build $CLEAN -t $NAME \
        --build-arg=USR=$TYPE \
        --build-arg=ROS_ARCH=$ROS_ARCH \
        --build-arg=ROS_DISTRO=$ROS_DISTRO \
        --build-arg=HOST_USERNAME=$HOST_USERNAME \
        --build-arg=HOST_UID=$HOST_UID \
        --build-arg=HOST_GROUPNAME=$HOST_GROUPNAME \
        --build-arg=HOST_GID=$HOST_GID \
        --file docker/Dockerfile.build .
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