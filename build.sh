#!/bin/bash

print_usage()
{
    echo "Build novatel_oem7_driver"
    echo "Usage: "
    echo "  $0 -h"
    echo "  $0 [-c] [-d | -r] [-p] [-t] [-g]"
    echo
    echo "Where:"
    echo "  -h: Print this help message and exit."
    echo "  -c: Clean all intermediate files."
    echo "  -d: Build debug binaries"
    echo "  -r: Build release binaries; can't be combined with -d"
    echo "  -t: Run tests using local binaries."
    echo "  -p: Build .deb package(s)"
    echo "  -g: Generate documentation"
    echo "  -f: Full build, equivalent to '$0 -crtgp'"
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

on_invalid_args()
{
    print_usage
    exit 1
}

build()
{
    colcon build $DEBUG_FLAG --base-paths src/
    source install/setup.sh
}

run_tests()
{
   source install/setup.sh
   colcon test --base-paths src/
   colcon test-result
}

rosdoc()
{
    rosdoc2 build --package-path src/novatel_oem7_driver/ -o docs_output/
}

INSTALL=
CLEAN=
DEBUG_FLAG=
ROSDOC=
BUILD_DEB_PKG=
RUN_TESTS=
NO_ARGS=on_invalid_args

while getopts "hcdrpftg" OPT; do

    NO_ARGS=

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
	        BUILD=build
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

        g )
            ROSDOC=rosdoc
     	    ;;

        f )
            CLEAN=clean
            BUILD=build
            INSTALL=install
            BUILD_DEB_PKG=build_deb_pkg
            RUN_TESTS=run_tests
            ROSDOC=rosdoc
     	    ;;

        * )
            on_invalid_args
	        ;;
    esac
done


$NO_ARGS

if [[ $CLEAN ]];
then
	# Remove all intermediate and temporary files.
	rm -rf  .ros  build doc install cross_reference docs_build docs_output
	rm -rf *.deb src/*.ddeb
	rm -rf src/novatel_oem7_driver/debian src/novatel_oem7_driver/.obj-*
	rm -rf src/novatel_oem7_msgs/debian src/novatel_oem7_msgs/.obj-*
	rm -f src/CMakeLists.txt
    # TODO clean docs

	if [[ -z $INSTALL && -z $RUN_TESTS && -z $BUILD_DEB_PKG && -z $ROSDOC ]];
	then
		exit 0
	fi

fi


#-----------------------------------------------------------

set -e


$BUILD
$RUN_TESTS
$ROSDOC

if [[ $INSTALL ]];
then
	$BUILD_DEB_PKG
fi
