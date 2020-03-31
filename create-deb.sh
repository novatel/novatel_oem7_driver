#!/bin/sh

CALLED_FROM=$PWD
cd $3

rm -rf debian obj-*
bloom-generate rosdebian --os-name ubuntu --os-version $1 --ros-distro $2


fakeroot -- debian/rules binary

cd $CALLED_FROM




