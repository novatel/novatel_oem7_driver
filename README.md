# NovAtel OEM7 Driver
[**ROS**](https://www.ros.org) Driver for [**NovAtel**](https://www.novatel.com) OEM7 GNSS Receivers.  

## Getting Started

### Prerequisites
* ROS Kinetic or Melodic, including gps-common and tf ROS packages.
* Obtain OEM7 receiver.  


### Installation
#### Requirements:
novatel_oem7_driver depends on the ROS `gps-common` and `tf` packages. You can install them via the commands:
```
sudo apt install ros-${ROS_DISTRO}-tf
sudo apt install ros-${ROS_DISTRO}-gps-common
```

Note `${ROS_DISTRO}` is expected the evaluate to the name of your ROS distro, such as 'melodic', for example.

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
1. From the base directory of the repository, run `./docker/run-melodic-build.sh` (this may take >10mins)
1. From within your docker container (where the prompt from above should land), run `./build.sh -f`

#### Option C: Build from source (local environment)
Here are approximate instructions for building this driver with your local ROS development environment. Please note this is for reference. The Docker approach is recommended.

1. Install ROS with developer support to your environment ([**ROS Wiki Ubuntu 18.04**](http://wiki.ros.org/Installation/Ubuntu))
1. Install ROS dependencies gps-common, tf
	Ex: `sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-gps-common ros-melodic-tf`
1. Run `source /opt/ros/melodic/setup.bash`
1. Set `ROS_DISTRO` environment variable (Ex: `ROS_DISTRO=melodic`)
1. Run `bash envsetup.sh`
1. Run build: `./build.sh -f`

#### Create .deb from your build and install it
After building, run the `create-${ROS_DISTRO}-package.sh` script to produce the .deb packages (novatel-oem7-driver and novatel-oem7-msgs) for your binary build.

You can then install these via `dpkg`, provided you have installed the required common packages mentioned earlier:
```
sudo dpkg -i ros-{$ROS_DISTRO}-novatel-oem7*.deb
```

## Driver Documentation
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



## Authors

* [**NovAtel**](https://www.novatel.com), part of [**Hexagon**](https://hexagon.com)


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details


