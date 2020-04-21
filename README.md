# NovAtel OEM7 Driver
[**ROS**](https://www.ros.org) Driver for [**NovAtel**](https://www.novatel.com) OEM7 GNSS Receivers.  

## Getting Started

### Prerequisites
* ROS Kinetic or Melodic, including gps-common and tf ROS packages.
* Obtain OEM7 receiver.  


### Installation
#### Option A: Install binary package
```
sudo apt-get install ros-${ROS_DISTRO}-novatel-oem7-driver
```

#### Option B: Build from source (docker)
These instructions assume that you are using Ubuntu 18.04.

1. Install Docker, add the user you intend on using to the 'docker' group.
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


## Basic Operation with  OEM7 receiver

### Operation using TCP/IP 

Use default TCP/IP port '3001'. Refer to the OEM7 receiver [manual](https://docs.novatel.com/OEM7/Content/Home.htm) for more details on ICOM ports.  

```shell
roslaunch novatel_oem7_driver oem7_net.launch oem7_ip_addr:=10.128.224.56
```


### Operation using UDP/IP

By default (factory reset), OEM7 receivers do not support UDP connections. An ICOM port must be configured for UDP and **the configuration saved.**

Using a terminal program, connect to OEM7 COM port as described in OEM7 manual and enter :
```
ICOMCONFIG ICOM2 UDP ":3002"
SAVECONFIG
```
Then
```shell
roslaunch novatel_oem7_driver oem7_net.launch oem7_if:=Oem7ReceiverUdp  `oem7_ip_addr:=10.128.224.56 oem7_port:=3002
```
Refer to "ICOMCONFIG" command in OEM7 receiver manual.


### Operation Using Serial Port (TTY)


```shell
roslaunch novatel_oem7_driver oem7_tty.launch oem7_tty_name:=/dev/ttyUSB0
```
  
To (optionally) set a specific baud rate, e.g. 115200, set **oem7_tty_baud**:=115200  
Ensure that the baud rate of receiver port matches; the driver does ***not*** automatically configure receiver baud rate.  
Setting the baud rate is required for physical RS232 ports. 
Note that baud rate does not need to be used for USB ports.

For more USB related information, please refer to the [USB Communications documentation](https://docs.novatel.com/OEM7/Content/Operation/USB_Communications.htm).

In order to use USB ports, the user you're operating as must be allowed to read/write to the /dev/ttyUSB* ports. On Ubuntu
Linux, this requires the user be a member of the 'dialout' group. 

If you are using an older kernel, it might not know to attach your NovAtel sensor to the built-in usbserial kernel module.
You can manually instruct your kernel to do this by either of two methods:

#### Optional: Associate NovAtel USB with Linux usbserial via udev
Note, this is only needed if you are using an older kernel that does not attach your USB-attached NovAtel sensor to virtual
serial ports at /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyUSB2 (etc). Check `dmesg` output when connecting via USB to check if
you are using a kernel that automatically makes this connection.

1. Create a new udev rules file, such as: `/etc/udev/rules.d/z90_novatel.rules`
2. Paste in the following:
```
SUBSYSTEM=="usb", SYSFS{idProduct}=="0100", SYSFS{idVendor}=="09d7",
PROGRAM="/sbin/modprobe usbserial vendor=0x09d7 product=0x0100"
  
BUS=="usb", SYSFS{idProduct}=="0100", SYSFS{idVendor}=="09d7",
SYSFS{product}=="NovAtel GPS Receiver", SYSFS{manufacturer}=="Hexagon NovAtel Inc.", SYMLINK+="gps%n" 
```

#### Optional: Associate NovAtel USB with Linux usbserial via command to live kernel
Note, this is only needed if you are using an older kernel that does not attach your USB-attached NovAtel sensor to virtual
serial ports at /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyUSB2 (etc). Check `dmesg` output when connecting via USB to check if
you are using a kernel that automatically makes this connection.

Execute this command in your shell:
```shell
echo '09d7 0100' > /sys/bus/usb-serial/drivers/generic/new_id
```


#### Optional: Install the USB driver
There is an older dedicated package you may install to add a USB driver, [located here](https://www.novatel.com/support/info/documents/809).


###  ROS Parameter namespace layout

The driver uses standard ROS relative parameter addressing, with some parameters private to the nodelets.

The default namespace is outlined below. The namespace structure can be changed in the .launch and .xml files.  
  

**/novatel/oem7**  
Overall driver namespace; global driver paramerers.  

**/novatel/oem7/receivers**  
OEM7 Receiver instances

**/novatel/oem7/receivers/main**  
OEM7 Receiver instance: interface (TCP/IP, TTY) parameters, initialization commands, published topics, etc.  






## Customization

### Configuring your own initialization commands

The **/novatel/oem7/receivers/main/receiver_init_commands** parameter holds the list of commands to be executed on driver startup.  
There are no semantics checking of any kind; it is the user's responsibility to ensure correctness.  
By default, this list is populated from ***$(find novatel_oem7_driver)/config/std_init_commands.yaml***  

  
This list can be overriden using standard ROS parameter mechanisms.  

  

Notes:

1.  In order to be processed by the driver, logs must be in OEM7 "binary' format, e.g. "BESTPOS**B**"  



### Configuring topic parameters:

Topic name and reference frame are configured by adding the following parameter under /novatel/oem7/receivers/main  

```
$message_name{topic: $topic_name, frame_id:$id, queue_size: "$size"}  
```

INSCONFIG
```
INSCONFIG: {topic: /gps/insconfig, frame_id: gps, queue_size: "10"}  
```
The default configuration is populated from "$(find novatel_oem7_driver)/config/std_msg_topics.yaml"  



### Other Configuration Files

The .launch files includes other configuration files:  

  
$(find novatel_oem7_driver)/config/**std_oem7_raw_msgs.yaml**  
The list of OEM7 messages to logged as blobs for postprocessing. Refer to novatel_oem7_msg/Oem7RawMsg.msg  

  

$(find novatel_oem7_driver)/config/**std_driver_config.xml**  
$(find novatel_oem7_driver)/config/**std_msg_handlers.yaml**  
Various driver plugins to be loaded; there is no need to modify these unless you are developing your own plugins or optimizing the driver.  

  

$(find novatel_oem7_driver)/config/**oem7_msgs.yaml**  
A map of OEM7 message names / IDs.  
Messages not in this list will be ignored by the driver. Typically, this does not need modification, unless you are optimizing the driver.  



### ROS Message Generation

ROS messages are derived ("synthesized") from multiple OEM7 messages.  
Consult documentation in $(find novatel_oem7_driver)/config/std_init_commands.yaml

  

### Runtime Receiver Reconfiguration

The driver implements ROS Service "**/novatel_oem7_driver/receivers/main/Oem7Cmd**" which sends OEM7 Abbreviated ASCII commands from Linux terminal using "rosservice" tool:
```
rosservice call /novatel/oem7/receivers/main/Oem7Cmd "$oem7_abbreviated_ascii_command"
```
OEM7 response is printed out verbatim.

Example:
```
rosservice call /novatel/oem7/receivers/main/Oem7Cmd "LOG BESTPOSB ONTIME 0.1"
rsp: "OK"
```



All standard OEM7 abbreviated ascii commands are supported. The commands are executed "as-is"; the syntax is not modified.




### Error Detection and Recovery

There is no internal recovery mechanism; whenever the driver goes into an error state, message publishing stops. The user is responsible for detection of this condition.  
Recovery is achieved by reloading the driver (Message and Config nodelets).  

  

  

### Built-in Self-Test (BIST)

Standard driver installation supports BIST, which can be used to evaluate if the driver is operating normally.  
BIST is lauched by using rostest instead of roslaunch, by specifying "oem7_bist:=true" parameter with your standard .launch file.

For example, to run BIST with an oem7_net.launch file:

```
**rostest** --text novatel_oem7_driver oem7_net.launch oem7_ip_addr:=10.128.224.56 **oem7_bist:=true**

BIST outputs topic statistics, similar to the "rostopic hz" command:  
  

... topic: '/gps/gps', exp interval= 0.02. samples= 2945:

mean, max, stdev

Bag Recording interval: 0.0200001721311, 0.127890110016, 0.00589703421274

Message publish interval: 0.0200002559501, 0.119542121887, 0.00569331898041

Bag Rec - Message Pub Delta: 0.000302689346676, 0.0227742195129, 0.000916649615432

  

topic: '/novatel/oem7/inspva', exp interval= 0.02. samples= 2918:

mean, max, stdev

Bag Recording interval: 0.0199991217369, 0.0854535102844, 0.0049107521279

Message publish interval: 0.0199991563922, 0.0838820934296, 0.00478976290435

Bag Rec - Message Pub Delta: 0.000299763973647, 0.0198292732239, 0.000763679984502

  

topic: '/novatel/oem7/time', exp interval= 1.0. samples= 57:

mean, max, stdev

Bag Recording interval: 0.999989598989, 1.0087621212, 0.0045717348049

Message publish interval: 0.999993549926, 1.00874257088, 0.00458939860791

Bag Rec - Message Pub Delta: 0.000231772138361, 0.000780344009399, 0.000111371446655

  

topic: '/gps/fix', exp interval= 0.02. samples= 2886:

mean, max, stdev

Bag Recording interval: 0.0200002543021, 0.078773021698, 0.00453926745601

Message publish interval: 0.0200003352901, 0.0766448974609, 0.00433987262471

Bag Rec - Message Pub Delta: 0.000307747992227, 0.0275449752808, 0.00098472728123

  

topic: '/novatel/oem7/bestpos', exp interval= 0.1. samples= 573:

mean, max, stdev

Bag Recording interval: 0.100003437979, 0.144937515259, 0.0112068678576

Message publish interval: 0.100003776017, 0.144729137421, 0.0111951734664

Bag Rec - Message Pub Delta: 0.000227286137419, 0.00631713867188, 0.000422261912593

  

topic: '/gps/imu', exp interval= 0.01. samples= 5922:

mean, max, stdev

Bag Recording interval: 0.00999986636157, 0.127390623093, 0.00506395854586

Message publish interval: 0.0099998703077, 0.118784427643, 0.00485288752033

Bag Rec - Message Pub Delta: 0.000257168738917, 0.0433497428894, 0.00116386292041

  

topic: '/novatel/oem7/inspvax', exp interval= 1.0. samples= 59:

mean, max, stdev

Bag Recording interval: 0.999823825113, 1.02793908119, 0.00850038277189

Message publish interval: 0.999849241355, 1.02734613419, 0.00816920128965

Bag Rec - Message Pub Delta: 0.000262191740133, 0.00299835205078, 0.000491497024532

  

topic: '/novatel/oem7/bestvel', exp interval= 0.1. samples= 587:

mean, max, stdev

Bag Recording interval: 0.0998391059478, 0.159289360046, 0.0127196007736

Message publish interval: 0.0998426366585, 0.151807785034, 0.012558915178

Bag Rec - Message Pub Delta: 0.000218554290434, 0.00764131546021, 0.000507693107715

  

topic: '/novatel/oem7/corrimu', exp interval= 0.01. samples= 5746:

mean, max, stdev

Bag Recording interval: 0.00999978424882, 0.0556886196136, 0.00387268651204

Message publish interval: 0.00999978275482, 0.040646314621, 0.00370057604793

Bag Rec - Message Pub Delta: 0.000260806108575, 0.0385403633118, 0.000954403421047

  

topic: '/novatel/oem7/bestutm', exp interval= 1.0. samples= 58:

mean, max, stdev

Bag Recording interval: 0.999384821507, 1.0226817131, 0.00724017624648

Message publish interval: 0.999406969338, 1.02262759209, 0.00715263035281

Bag Rec - Message Pub Delta: 0.000181029582846, 0.00139737129211, 0.000200204160647

  

topic: '/novatel/oem7/insstdev', exp interval= 1.0. samples= 57:

mean, max, stdev

Bag Recording interval: 1.00011486241, 1.01120710373, 0.00461768586745

Message publish interval: 1.00011816195, 1.01120328903, 0.00446202626225

Bag Rec - Message Pub Delta: 0.000224979300248, 0.00264716148376, 0.000435574278454

  

ok

  

----------------------------------------------------------------------

Ran 2 tests in 106.496s

  

OK

-------------------------------------------------------------

SUMMARY:

* RESULT: SUCCESS

* TESTS: 2

* ERRORS: 0 []

* FAILURES: 0 []
```


### Sample Driver Output

```  
SUMMARY  
========  
  
PARAMETERS  
* /novatel/oem7/oem7_msgs/BDSEPHEMERIS: 1696  
* /novatel/oem7/oem7_msgs/BESTPOS: 42  
* /novatel/oem7/oem7_msgs/BESTUTM: 726  
* /novatel/oem7/oem7_msgs/BESTVEL: 99  
* /novatel/oem7/oem7_msgs/CORRIMUS: 2264  
* /novatel/oem7/oem7_msgs/GALFNAVEPHEMERIS: 1310  
* /novatel/oem7/oem7_msgs/GALINAVEPHEMERIS: 1309  
* /novatel/oem7/oem7_msgs/GLOEPHEMERIS: 723  
* /novatel/oem7/oem7_msgs/HEADING2: 1335  
* /novatel/oem7/oem7_msgs/IMURATECORRIMUS: 1362  
* /novatel/oem7/oem7_msgs/INSCONFIG: 1945  
* /novatel/oem7/oem7_msgs/INSPVAS: 508  
* /novatel/oem7/oem7_msgs/INSPVAX: 1465  
* /novatel/oem7/oem7_msgs/INSSTDEV: 2051  
* /novatel/oem7/oem7_msgs/INSUPDATESTATUS: 1825  
* /novatel/oem7/oem7_msgs/PSRDOP2: 1163  
* /novatel/oem7/oem7_msgs/RANGE: 43  
* /novatel/oem7/oem7_msgs/RAWEPHEM: 41  
* /novatel/oem7/oem7_msgs/RAWIMUSX: 1462  
* /novatel/oem7/oem7_msgs/RXSTATUS: 93  
* /novatel/oem7/oem7_msgs/TIME: 101  
* /novatel/oem7/receivers/main/BESTPOS/frame_id: gps  
* /novatel/oem7/receivers/main/BESTPOS/topic: /novatel/oem7/bes...  
* /novatel/oem7/receivers/main/BESTUTM/frame_id: gps  
* /novatel/oem7/receivers/main/BESTUTM/topic: /novatel/oem7/bes...  
* /novatel/oem7/receivers/main/BESTVEL/frame_id: gps  
* /novatel/oem7/receivers/main/BESTVEL/topic: /novatel/oem7/bes...  
* /novatel/oem7/receivers/main/CORRIMU/frame_id: gps  
* /novatel/oem7/receivers/main/CORRIMU/topic: /novatel/oem7/cor...  
* /novatel/oem7/receivers/main/GPSFix/frame_id: gps  
* /novatel/oem7/receivers/main/GPSFix/topic: /gps/gps  
* /novatel/oem7/receivers/main/HEADING2/frame_id: gps  
* /novatel/oem7/receivers/main/HEADING2/topic: /novatel/oem7/hea...  
* /novatel/oem7/receivers/main/IMU/frame_id: gps  
* /novatel/oem7/receivers/main/IMU/topic: /gps/imu  
* /novatel/oem7/receivers/main/INSCONFIG/frame_id: gps  
* /novatel/oem7/receivers/main/INSCONFIG/queue_size: 10  
* /novatel/oem7/receivers/main/INSCONFIG/topic: /novatel/oem7/ins...  
* /novatel/oem7/receivers/main/INSPVA/frame_id: gps  
* /novatel/oem7/receivers/main/INSPVA/topic: /novatel/oem7/inspva  
* /novatel/oem7/receivers/main/INSPVAX/frame_id: gps  
* /novatel/oem7/receivers/main/INSPVAX/queue_size: 10  
* /novatel/oem7/receivers/main/INSPVAX/topic: /novatel/oem7/ins...  
* /novatel/oem7/receivers/main/INSSTDEV/frame_id: gps  
* /novatel/oem7/receivers/main/INSSTDEV/queue_size: 10  
* /novatel/oem7/receivers/main/INSSTDEV/topic: /novatel/oem7/ins...  
* /novatel/oem7/receivers/main/NavSatFix/frame_id: gps  
* /novatel/oem7/receivers/main/NavSatFix/topic: /gps/fix  
* /novatel/oem7/receivers/main/Oem7RawMsg/frame_id: gps  
* /novatel/oem7/receivers/main/Oem7RawMsg/queue_size: 200  
* /novatel/oem7/receivers/main/Oem7RawMsg/topic: /novatel/oem7/oem...  
* /novatel/oem7/receivers/main/RXSTATUS/frame_id: gps  
* /novatel/oem7/receivers/main/RXSTATUS/queue_size: 10  
* /novatel/oem7/receivers/main/RXSTATUS/topic: /novatel/oem7/rxs...  
* /novatel/oem7/receivers/main/TIME/frame_id: gps  
* /novatel/oem7/receivers/main/TIME/topic: /novatel/oem7/time  
* /novatel/oem7/receivers/main/oem7_if: Oem7ReceiverTcp  
* /novatel/oem7/receivers/main/oem7_ip_addr: 10.128.224.50  
* /novatel/oem7/receivers/main/oem7_msg_decoder: Oem7MessageDecoder  
* /novatel/oem7/receivers/main/oem7_msg_handlers: ['BESTPOSHandler'...  
* /novatel/oem7/receivers/main/oem7_port: 3001  
* /novatel/oem7/receivers/main/oem7_raw_msgs: ['BESTPOS', 'BEST...  
* /novatel/oem7/receivers/main/receiver_init_commands: ['UNLOGALL THISPO...  
* /novatel/oem7/supported_imus/0/name: Unknown  
* /novatel/oem7/supported_imus/0/rate: 0  
* /novatel/oem7/supported_imus/1/name: Honeywell HG1700 AG1  
* /novatel/oem7/supported_imus/1/rate: 100  
* /novatel/oem7/supported_imus/11/name: Honeywell HG1700 ...  
* /novatel/oem7/supported_imus/11/rate: 100  
* /novatel/oem7/supported_imus/12/name: Honeywell HG1700 ...  
* /novatel/oem7/supported_imus/12/rate: 100  
* /novatel/oem7/supported_imus/13/name: iMAR ilMU-FSAS  
* /novatel/oem7/supported_imus/13/rate: 200  
* /novatel/oem7/supported_imus/16/name: KVH CPT IMU  
* /novatel/oem7/supported_imus/16/rate: 200  
* /novatel/oem7/supported_imus/19/name: Northrop Grumman ...  
* /novatel/oem7/supported_imus/19/rate: 200  
* /novatel/oem7/supported_imus/20/name: Honeywell HG1930 ...  
* /novatel/oem7/supported_imus/20/rate: 100  
* /novatel/oem7/supported_imus/26/name: Northrop Grumman ...  
* /novatel/oem7/supported_imus/26/rate: 100  
* /novatel/oem7/supported_imus/27/name: Honeywell HG1900 ...  
* /novatel/oem7/supported_imus/27/rate: 100  
* /novatel/oem7/supported_imus/28/name: Honeywell HG1930 ...  
* /novatel/oem7/supported_imus/28/rate: 100  
* /novatel/oem7/supported_imus/31/name: Analog Devices AD...  
* /novatel/oem7/supported_imus/31/rate: 200  
* /novatel/oem7/supported_imus/32/name: Sensonor STIM300  
* /novatel/oem7/supported_imus/32/rate: 125  
* /novatel/oem7/supported_imus/33/name: KVH1750 IMU  
* /novatel/oem7/supported_imus/33/rate: 200  
* /novatel/oem7/supported_imus/4/name: Honeywell HG1700 ...  
* /novatel/oem7/supported_imus/4/rate: 100  
* /novatel/oem7/supported_imus/41/name: Epson G320N  
* /novatel/oem7/supported_imus/41/rate: 125  
* /novatel/oem7/supported_imus/45/name: KVH 1725 IMU?  
* /novatel/oem7/supported_imus/45/rate: 200  
* /novatel/oem7/supported_imus/5/name: Honeywell HG1700 ...  
* /novatel/oem7/supported_imus/5/rate: 100  
* /novatel/oem7/supported_imus/52/name: Litef microIMU  
* /novatel/oem7/supported_imus/52/rate: 200  
* /novatel/oem7/supported_imus/56/name: Sensonor STIM300,...  
* /novatel/oem7/supported_imus/56/rate: 125  
* /novatel/oem7/supported_imus/58/name: Honeywell HG4930 ...  
* /novatel/oem7/supported_imus/58/rate: 200  
* /novatel/oem7/supported_imus/61/name: Epson G370N  
* /novatel/oem7/supported_imus/61/rate: 200  
* /novatel/oem7/supported_imus/62/name: Epson G320N - 200Hz  
* /novatel/oem7/supported_imus/62/rate: 200  
* /novatel/oem7/supported_imus/8/name: Northrop Grumman ...  
* /novatel/oem7/supported_imus/8/rate: 200  
* /rosdistro: melodic  
* /rosversion: 1.14.3  
  
NODES  
/novatel/oem7/  
driver (nodelet/nodelet)  
/novatel/oem7/receivers/  
main (nodelet/nodelet)  
/novatel/oem7/receivers/main/  
config (nodelet/nodelet)  
  
auto-starting new master  
process[master]: started with pid [95]  
ROS_MASTER_URI=[http://localhost:11311](http://localhost:11311/)  
  
setting /run_id to 3c58ff7a-5825-11ea-af66-0242ac110002  
process[rosout-1]: started with pid [106]  
started core service [/rosout]  
process[novatel/oem7/driver-2]: started with pid [113]  
[ INFO] [1582672757.580037186]: Initializing nodelet with 1 worker threads.  
process[novatel/oem7/receivers/main-3]: started with pid [114]  
[ INFO] [1582672758.427979153]: Loading nodelet /novatel/oem7/receivers/main of type novatel_oem7_driver/Oem7MessageNodelet to manager /novatel/oem7/driver with the following remappings:  
[ INFO] [1582672758.482685569]: /novatel/oem7/receivers/main: Oem7MessageNodelet v.0.1.5; Feb 25 2020 23:10:59  
[ INFO] [1582672758.491468200]: Oem7MessageDecoderLib version: 0.1.0  
[ INFO] [1582672758.501851170]: topic [/novatel/oem7/bestpos]: frame_id: 'gps'; q size: 100  
[ INFO] [1582672758.506864595]: topic [/novatel/oem7/bestvel]: frame_id: 'gps'; q size: 100  
[ INFO] [1582672758.520382178]: topic [/novatel/oem7/bestutm]: frame_id: 'gps'; q size: 100  
[ INFO] [1582672758.522930675]: topic [/novatel/oem7/inspva]: frame_id: 'gps'; q size: 100  
[ INFO] [1582672758.525848469]: topic [/gps/gps]: frame_id: 'gps'; q size: 100  
[ INFO] [1582672758.531813975]: topic [/gps/fix]: frame_id: 'gps'; q size: 100  
[ INFO] [1582672758.539888495]: topic [/gps/imu]: frame_id: 'gps'; q size: 100  
[ INFO] [1582672758.546775428]: topic [/novatel/oem7/corrimu]: frame_id: 'gps'; q size: 100  
[ INFO] [1582672758.561730288]: topic [/novatel/oem7/insstdev]: frame_id: 'gps'; q size: 10  
[ INFO] [1582672758.566235059]: topic [/novatel/oem7/inspvax]: frame_id: 'gps'; q size: 10  
[ INFO] [1582672758.568718145]: topic [/novatel/oem7/insconfig]: frame_id: 'gps'; q size: 10  
[ INFO] [1582672758.575187796]: topic [/novatel/oem7/heading2]: frame_id: 'gps'; q size: 100  
[ INFO] [1582672758.586377975]: topic [/novatel/oem7/rxstatus]: frame_id: 'gps'; q size: 10  
[ INFO] [1582672758.597258756]: topic [/novatel/oem7/time]: frame_id: 'gps'; q size: 100  
[ INFO] [1582672758.608667707]: Oem7 Raw message 'BESTPOS' will be published.  
[ INFO] [1582672758.608896692]: Oem7 Raw message 'BESTVEL' will be published.  
[ INFO] [1582672758.609316891]: Oem7 Raw message 'INSPVAX' will be published.  
[ INFO] [1582672758.609700340]: Oem7 Raw message 'INSUPDATESTATUS' will be published.  
[ INFO] [1582672758.610101790]: Oem7 Raw message 'RAWIMUSX' will be published.  
[ INFO] [1582672758.610435285]: Oem7 Raw message 'RAWEPHEM' will be published.  
[ INFO] [1582672758.610800972]: Oem7 Raw message 'GLOEPHEMERIS' will be published.  
[ INFO] [1582672758.611205708]: Oem7 Raw message 'BDSEPHEMERIS' will be published.  
[ INFO] [1582672758.611586715]: Oem7 Raw message 'GALFNAVEPHEMERIS' will be published.  
[ INFO] [1582672758.611931563]: Oem7 Raw message 'GALINAVEPHEMERIS' will be published.  
[ INFO] [1582672758.612324055]: Oem7 Raw message 'RANGE' will be published.  
[ INFO] [1582672758.612752046]: Oem7 Raw message 'HEADING2' will be published.  
[ INFO] [1582672758.613338666]: Oem7 Raw message 'TIME' will be published.  
[ INFO] [1582672758.619745586]: Oem7 Raw message 'RXSTATUS' will be published.  
[ INFO] [1582672758.620168469]: Oem7 Raw message 'INSCONFIG' will be published.  
[ INFO] [1582672758.625131610]: topic [/novatel/oem7/oem7raw]: frame_id: 'gps'; q size: 200  
[ INFO] [1582672758.645420418]: Oem7Net TCP['10.128.224.50' : 3001]  
[ INFO] [1582672758.648501360]: Oem7Net socket open: '1; OS error= 0  
process[novatel/oem7/receivers/main/config-4]: started with pid [123]  
[ INFO] [1582672759.522221729]: Loading nodelet /novatel/oem7/receivers/main/config of type novatel_oem7_driver/Oem7ConfigNodelet to manager /novatel/oem7/driver with the following remappings:  
[ INFO] [1582672759.532702445]: /novatel/oem7/receivers/main/config: Oem7ConfigNodelet v.0.1.5; Feb 25 2020 23:10:59  
[ INFO] [1582672759.549198962]: AACmd 'UNLOGALL THISPORT' : 'OK'  
[ INFO] [1582672759.555651969]: AACmd 'LOG INSCONFIGB ONCE' : 'OK'  
[ INFO] [1582672759.557937765]: IMU: 'Epson G320N', rate= 125  
[ INFO] [1582672759.563950339]: AACmd 'LOG INSCONFIGB ONTIME 300' : 'OK'  
[ INFO] [1582672759.570031172]: AACmd 'LOG RXSTATUSB ONCHANGED' : 'OK'  
[ INFO] [1582672759.692688310]: AACmd 'LOG BESTPOSB ONTIME 0.1' : 'OK'  
[ INFO] [1582672759.699932134]: AACmd 'LOG BESTVELB ONTIME 0.1' : 'OK'  
[ INFO] [1582672759.708925767]: AACmd 'LOG BESTUTMB ONTIME 1' : 'OK'  
[ INFO] [1582672759.717126078]: AACmd 'LOG HEADING2B ONNEW' : 'OK'  
[ INFO] [1582672759.968983558]: AACmd 'LOG PSRDOP2B ONCHANGED' : 'OK'  
[ INFO] [1582672759.979083113]: AACmd 'LOG INSPVASB ONTIME 0.02' : 'OK'  
[ INFO] [1582672759.989194363]: AACmd 'LOG INSPVAXB ONTIME 1' : 'OK'  
[ INFO] [1582672760.110518011]: AACmd 'LOG CORRIMUSB ONTIME 0.01' : 'OK'  
[ INFO] [1582672760.117430487]: AACmd 'LOG INSSTDEVB ONTIME 1' : 'OK'  
[ INFO] [1582672760.127582549]: AACmd 'LOG TIMEB ONTIME 1' : 'OK'  
[ INFO] [1582672760.250652075]: AACmd 'LOG RAWIMUSXB ONNEW' : 'OK'  
[ INFO] [1582672760.259886196]: AACmd 'LOG INSUPDATESTATUSB ONNEW' : 'OK'  
[ INFO] [1582672760.268288028]: AACmd 'LOG RAWEPHEMB ONNEW' : 'OK'  
[ INFO] [1582672760.441192318]: AACmd 'LOG GLOEPHEMERISB ONNEW' : 'OK'  
[ INFO] [1582672760.452404856]: AACmd 'LOG BDSEPHEMERISB ONNEW' : 'OK'  
[ INFO] [1582672760.463694659]: AACmd 'LOG GALFNAVEPHEMERISB ONNEW' : 'OK'  
[ INFO] [1582672760.476838316]: AACmd 'LOG GALINAVEPHEMERISB ONNEW' : 'OK'  
[ INFO] [1582672760.485394230]: AACmd 'LOG RANGEB ONTIME 1' : 'OK'  
[ INFO] [1582672760.486592625]: Oem7 configuration completed.  
[ INFO] [1582672793.168160450]: Log Statistics:  
[ INFO] [1582672793.170725228]: Logs: 10000; discarded: 37  
[ INFO] [1582672793.171424835]: Log[RAWEPHEM](41):10  
[ INFO] [1582672793.171991321]: Log[BESTPOS](42):335  
[ INFO] [1582672793.172548012]: Log[RANGE](43):33  
[ INFO] [1582672793.173131571]: Log[RXSTATUS](93):1  
[ INFO] [1582672793.173925225]: Log[BESTVEL](99):335  
[ INFO] [1582672793.174466830]: Log[TIME](101):33  
[ INFO] [1582672793.175082019]: Log[INSPVAS](508):1660  
[ INFO] [1582672793.177104558]: Log[GLOEPHEMERIS](723):8  
[ INFO] [1582672793.177702006]: Log[BESTUTM](726):34  
[ INFO] [1582672793.178415540]: Log[PSRDOP2](1163):2  
[ INFO] [1582672793.182107115]: Log[HEADING2](1335):34  
[ INFO] [1582672793.182548463]: Log[RAWIMUSX](1462):4130  
[ INFO] [1582672793.183634882]: Log[INSPVAX](1465):33  
[ INFO] [1582672793.187078494]: Log[INSCONFIG](1945):2  
[ INFO] [1582672793.191874451]: Log[INSSTDEV](2051):33  
[ INFO] [1582672793.192099653]: Log[CORRIMUS](2264):3317
```

#### Output Interpretation:


```
[ INFO] [1582672759.692688310]: AACmd 'LOG BESTPOSB ONTIME 0.1' : 'OK'  
```
Initialization command sent, "OK" response received.  
  
  


```
[ INFO] [1582672759.557937765]: IMU: 'Epson G320N', rate= 12
```
Detected IMU and assumed data rate.  




```
Logs: 10000; discarded: 37  
```
10000 logs received and processed; 37 logs or message fragments discarded.  
Discarded fragments during receiver configuration are normal. However, continuously increasing number of discarded fragments during stable operation indicates misconfiguration.  
Received OEM7 Message statistics are output periodically.



```
Log[BESTPOS](42):335
```
335 BESTPOS logs (ID=42) received.  



## Authors

* [**NovAtel**](https://www.novatel.com), part of [**Hexagon**](https://hexagon.com)


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details


