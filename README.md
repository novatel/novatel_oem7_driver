# NovAtel OEM7 Driver
[**ROS**](https://www.ros.org) Driver for [**NovAtel**](https://www.novatel.com) OEM7 GNSS/SPAN Receivers.  

## Getting Started
This documents how to custom-build the novatel_oem7_driver for ROS from the provided source code. Typical users will prefer to 
install the pre-made binary release that has been published in the ROS distribution.

Refer to our documentation under the ROS community wiki for:
 * Hardware Setup
 * Binary Driver Installation
 * Driver Configuration
 * Driver Runtime Operation
 * Post-Processing data
 * Information on Relevant NovAtel Services and Products
 * Advanced Topics

novatel_oem7_driver documentation on ROS community wiki is located here:
http://wiki.ros.org/novatel_oem7_driver

<HR>

## Building novatel_oem7_driver from source code
### Prerequisites
* Install ROS Noetic, Melodic or Kinetic.
* Obtain OEM7 receiver.  


### Installation
#### Option A: Install binary package
There is substantial documention regarding use of the binary release of this driver on the ROS community wiki, located here:
https://wiki.ros.org/novatel_oem7_driver

The key step is:
```
sudo apt install ros-${ROS_DISTRO}-novatel-oem7-driver
```

Please refer to the Community Wiki for detailed run-time documentation for novatel_oem7_driver (link given above).


#### Option B: Build from source (docker)
These instructions assume that you are using Ubuntu 18.04.

1. Install Docker, add the user you intend on using to the 'docker' group. For example:
   1. Add the current user to the 'docker' group: `sudo usermod -aG docker ${USER}`
   1. Apply the membership changes to the current session: `su - ${USER}`
1. From the base directory of the repository, create container for the desired ROS architecture and distro, e.g. Noetic:  
   `./docker/run.sh -r amd64 noetic`  
   Note: only amd64 architecture is supported at this point.  
1. From within your docker container (where the prompt from above should land), run `./build.sh -f`

#### Option C: Build from source (local environment)
Here are approximate instructions for building this driver with your local ROS development environment. Please note this is for reference. The Docker approach is recommended.

1. Install ROS with developer support to your environment ([**ROS Wiki Ubuntu 18.04**](http://wiki.ros.org/Installation/Ubuntu))
1. Install ROS dependencies using `rosdep install --from-paths src --ignore-src -r -y`
1. Set `ROS_DISTRO` environment variable (Ex: `ROS_DISTRO=noetic`)
1. Run `source /opt/ros/${ROS_DISTRO}/setup.bash`
1. Run `source envsetup.sh`
1. Run build: `./build.sh -f`

#### Install .deb packages 
Building produces two deb package, novatel-oem7-driver and novatel-oem7-msgs.

You can then install these via `apt` or `dpkg`:
```
sudo apt install ./ros-{$ROS_DISTRO}-novatel-oem7*.deb
```

## Next Steps
Refer to the novatel_oem7_driver documentation in the ROS wiki for more information:
http://wiki.ros.org/novatel_oem7_driver


## Authors

* [**NovAtel**](https://www.novatel.com), part of [**Hexagon**](https://hexagon.com)


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details


