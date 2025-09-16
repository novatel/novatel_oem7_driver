# NovAtel OEM7 Driver
[**ROS**](https://www.ros.org) Driver for [**NovAtel**](https://www.novatel.com) OEM7 GNSS/SPAN Receivers.

## Getting Started
This documents how to custom-build the novatel_oem7_driver for ROS from the provided source code. Typical users will prefer to
install the pre-made binary release that has been published in the ROS distribution.

Refer to our documentation under the ROS community wiki for many topics includings:
 * Getting Started
 * Hardware Setup
 * Binary Driver Installation
 * Driver Configuration
 * Driver Runtime Operation
 * Post-Processing Data
 * Many Frequently Asked Questions
 * Information on Relevant NovAtel Services and Products
 * Advanced Topics

novatel_oem7_driver documentation on is located here:
https://docs.ros.org/en/jazzy/p/novatel_oem7_driver/

<HR>

## Building novatel_oem7_driver from source code
### Prerequisites
* Install ROS2 Jazzy.
* Obtain OEM7 receiver.


### Installation
#### Option A: Install binary package
There is substantial documention regarding use of the binary release of this driver on the novatel_oem7_driver documentation pages, located here: https://docs.ros.org/en/jazzy/p/novatel_oem7_driver/

The key step is:
```
sudo apt install ros-${ROS_DISTRO}-novatel-oem7-driver
```

Please refer to the Wiki for detailed run-time documentation for novatel_oem7_driver (link given above).


#### Option B: Build from source (docker)
These instructions assume that you are using Ubuntu 24.04.

1. Install Docker, add the user you intend on using to the 'docker' group. For example:
   1. Add the current user to the 'docker' group: `sudo usermod -aG docker ${USER}`
   1. Apply the membership changes to the current session: `su - ${USER}`
1. From the base directory of the repository, create container for the desired ROS architecture and distro, e.g. Jazzy:
   `./docker/run.sh -r amd64 jazzy`
   Note: only amd64 and arm64v8 architectures are supported at this point.
1. From within your docker container, use standard ROS2 tools, like colcon.
1. Alternatively, use the build.sh script.

#### Option C: Build from source (local environment)
Here are approximate instructions for building this driver with your local ROS development environment. Please note this is for reference. The Docker approach is recommended.

1. Install ROS with developer support to your environment ([**ROS Wiki Ubuntu**](http://wiki.ros.org/Installation/Ubuntu))
1. Install ROS dependencies using `rosdep install --from-paths src --ignore-src -r -y`
1. Run `source /opt/ros/${ROS_DISTRO}/setup.bash`
1. Run build: `./build.sh -f`
1. After a successful build, source your local environment: . install/setup.sh

#### Install .deb packages
Building produces two deb package, novatel-oem7-driver and novatel-oem7-msgs.

You can then install these via `apt` or `dpkg`:
```
sudo apt install ./ros-{$ROS_DISTRO}-novatel-oem7*.deb
```

## Configuration Parameters

To configure the driver revert to the Configuration of novatel_oem7_driver Getting Started page in the documentation (linked above). The launch files have a variety of parameters that can be set. They are listed below:

* **oem7_position_source**: string. Oem7 log used to obtain the position for GPSFix, NavSatFix, Odometry

   **BESTPOS**: Always use BESTPOS log

   **INSPVAS**: Always use INSPVAS log. Requires SPAN

   unset (default): use INSPVAS unless BESTPOS has better quality

* **oem7_imu_rate**: int. Overrides the IMU data rate.

   Can Used to enable IMU message generation when INSCONFIG is not available, e.g. when feeding input from

   a partial dataset file.

   default: 0; INSCONFIG must be present in input.

* **oem7_odometry_zero_origin**: bool When 'True', use 0,0,0 as Odometry origin; the first valid GPSFix is used
   to set the   origin.

   default: False

* **oem7_odometry_transform**: bool. When 'True', Transform is published, sourced from Odometry.

   default: False

* **oem7_receiver_log_file**: string. Path to a file recording all bytes output by the Oem7 Receiver.

   default: "", no output is captured.

* **oem7_decoder_log_file**: Path to a file recording all bytes output by the Oem7 Decoder. Used for decoder debugging.

   default: "", no output is captured.

* **oem7_strict_receiver_init**:bool. When 'True', strict initialization is used; no position / velocity /attitude data

   is output unless all receiver initialization commands have succeeded.

   default: True

* **oem7_publish_unknown_oem7raw**: bool. When 'True', all Oem7 messages not supported by the decoder are publshed as 'raw'.

    default: False

* **oem7_publish_delay**: double. Seconds to delay before publishing each message.

   Used for debugging, and with file-based input.

   default: 0


## Limitations

* This branch only supports ROS2 Jazzy, other branches contain the versions they support until that version is End of Life. While officially unsupported and functionality is not guranteed, to build with Foxy and older it is recommended to use the Humble branch and revert these changes: https://github.com/novatel/novatel_oem7_driver/pull/104

* Building the driver in its default state requires Boost 1.69 or greater. If working with an older version, revert these changes locally: https://github.com/novatel/novatel_oem7_driver/pull/98


## Next Steps
Refer to the novatel_oem7_driver documentation for more information: https://docs.ros.org/en/jazzy/p/novatel_oem7_driver/

## Authors

* [**NovAtel**](https://www.novatel.com), part of [**Hexagon**](https://hexagon.com)


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details


