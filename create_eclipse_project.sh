#!/bin/sh

catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
awk -f $(rospack find mk)/eclipse.awk build/.project > build/.project_with_env && mv build/.project_with_env build/.project

