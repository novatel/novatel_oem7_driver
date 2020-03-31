#!/bin/sh

docker build -t novatel_oem7_driver .
docker run -ti --mount type=bind,source="$(pwd)",target=/test  --rm  novatel_oem7_driver:latest 
