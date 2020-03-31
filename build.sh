#!/bin/bash

print_usage()
{
    echo "Build novatel_oem7_driver"
    echo "Usage: "
    echo "  $0 -h"
    echo "  $0 [-c] [-d | -r] [-p] [-t]"
    echo 
    echo "Where:"
    echo "  -h: Print this help message and exit."
    echo "  -c: Clean all intermediate files."
    echo "  -d: Build debug binaries"
    echo "  -r: Build release binaries; can't be combined with -d"
    echo "  -t: Run tests using local binaries."
    echo "  -p: Build .deb package(s)"
    echo "  -f: Full build, equivalent to '$0 -crtp'"
    echo 
    echo "Examples"
    echo "  '$0'       Build local binaries; used for local development."
    echo "  '$0 -f'    Full build: Used for production artifacts."
    echo "  '$0 -r'    Release build, no tests or .deb packages"
    echo "  '$0 -cd'   Clean Debug build, no packages."
}



build_deb_pkg()
{
   ./create-$ROS_DISTRO-package.sh
}


INSTALL=
CLEAN=
DEBUG_FLAG=
ROSDOC=
BUILD_DEB_PKG=
RUN_TESTS=


while getopts "hcdrpft" OPT; do
    case $OPT in
        h )
            print_usage
            exit 0
            ;;
            
        c )
            CLEAN=clean
            ;;
            
        d )
            INSTALL=install
            DEBUG_FLAG=-DCMAKE_BUILD_TYPE=Debug
            ;;
            
        r )
            INSTALL=install
            DEBUG_FLAG=
        	;;
        	
        p )
            INSTALL=install
            BUILD_DEB_PKG=build_deb_pkg
            ;;
            
	t ) 
	    RUN_TESTS=run_tests
	    ;;
	    	
        f ) 
            CLEAN=clean
            INSTALL=install
            BUILD_DEB_PKG=build_deb_pkg
	    RUN_TESTS=run_tests
     	    ;;
     	    
        * ) 		
	    print_usage
            exit 1
	    ;;
    esac
done



if [[ $CLEAN ]];
then
	# Remove all intermediate and temporary files.
	catkin_make clean
	rm -rf  .ros devel build doc install  
	rm -rf *.deb src/*.ddeb 
	rm -rf src/novatel_oem7_driver/debian src/novatel_oem7_driver/obj-*
	rm -rf src/novatel_oem7_msgs/debian src/novatel_oem7_msgs/obj-*
fi



set -e

. ./envsetup.sh

# Build local artifacts
catkin_make $DEBUG_FLAG $CLEAN $INSTALL $RUN_TESTS

if [[ $INSTALL ]];
then
	rosdoc_lite src/novatel_oem7_driver -o doc
	$BUILD_DEB_PKG
fi
