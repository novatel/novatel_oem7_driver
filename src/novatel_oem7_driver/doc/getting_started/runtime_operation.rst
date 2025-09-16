.. _Runtime_Operation:

Driver Runtime Operation
========================

.. _Run_the_novatel_oem7_driver:

Run novatel_oem7_driver
-----------------------

The following sections illustrate how to launch and use the novatel_oem7_driver by Hexagon | NovAtel for use with your OEM7 type GNSS or SPAN sensor.

.. _launch_tcp_or_udp:

Launching via a TCP/UDP Connection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When connecting to your receiver over a network, you will need to make sure the OEM7 receiver's IP address and intended ICOM port are correctly configured in the novatel_oem7_driver.

The protocol (TCP or UDP) and port numbers used can be configured by editing the file named `oem7_net.launch.py`. When using novatel_oem7_driver via binary package installation, the template for this file is located at the location:

::

   /opt/ros/{$ROS_DISTRO}/share/novatel_oem7_driver/launch/oem7_net.launch.py

For example, Jazzy users would therefore find this template file at:

::

   /opt/ros/jazzy/share/novatel_oem7_driver/launch/oem7_net.launch.py

Users may copy this template to a project directory, edit it and then run it with ``ros2 launch <the edited launch file>``.
To change the default TCP port that novatel_oem7_driver is configured to use, edit the default value of the ``oem7_port`` argument in your copy of the ``oem7_net.launch.py`` file.

Alternatively, users may launch by referring to the template launch file and then overriding parameters (like the IP address) as needed when calling ``ros2 launch``.

.. _Operation_over_TCP:

Operation over TCP/IP
~~~~~~~~~~~~~~~~~~~~~

Assumptions:

-  OEM7 Receiver is reachable over IP networking at 192.168.1.10
-  OEM7 Receiver's internal ICOM ports are configured for access over TCP (default)

.. Important::
   Here is an example of launching the driver with a TCP connection:
   ::

      ros2 launch novatel_oem7_driver oem7_net.launch.py oem7_ip_addr:=192.168.1.10

Suppose that you wanted to also override the default port specified in the ``oem7_net.launch.py`` file to instead be ``3005``, then you could run:

::

   ros2 launch novatel_oem7_driver oem7_net.launch.py oem7_ip_addr:=192.168.1.10 oem7_port:=3005

As another example, let's suppose you copied the template ``oem7_net.launch.py`` file to ``/path/to/my_novatel_oem7_net.launch.py``.
You could then edit the contents of this file with whatever parameters you like and then launch it with ``ros2 launch`` by running:

::

   ros2 launch /path/to/my_novatel_oem7_net.launch.py

.. _Operation_over_UDP:

Operation over UDP/IP
~~~~~~~~~~~~~~~~~~~~~

.. _Configure_OEM7_ICOM_Port_for_UDP:

Configure OEM7 ICOM Port(s) for UDP
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

OEM7 receivers default to TCP-based network communication (on hardware models with networking support).
You can reconfigure your OEM7 receiver to have some or all internal ICOM ports instead accessible via UDP ports.
For this, you will have to manually connect to your receiver and configure it with the `ICOMCONFIG <https://docs.novatel.com/OEM7/Content/Commands/ICOMCONFIG.htm>`__ command, followed by the `SAVECONFIG <https://docs.novatel.com/OEM7/Content/Commands/SAVECONFIG.htm>`__ command.

For example, the internal port ``ICOM2`` could be reconfigured to be
reachable on the receiver via UDP port number ``3002`` as follows:

::

   ICOMCONFIG ICOM2 UDP ":3002"
   SAVECONFIG

ICOM Communications documentation is also relevant and is available `here <https://docs.novatel.com/OEM7/Content/Operation/ICOM_Communications.htm>`__.


.. _Run_novatel_oem7_driver_with_UDP_profile:

Run novatel_oem7_driver with UDP profile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Assuming your receiver is reachable at UDP port 3002 at IP address 192.168.1.10, you can then connect novatel_oem7_driver with the ``ros2 launch`` command.

.. Important::
   Here is an example of launching the driver with a UDP connection:

   ::

      ros2 launch novatel_oem7_driver oem7_net.launch.py oem7_if:=Oem7ReceiverUdp oem7_ip_addr:=192.168.1.10 oem7_port:=3002

.. _Operation_over_USB:

Operation over USB
~~~~~~~~~~~~~~~~~~

The novatel_oem7_driver can be used to communicate with your OEM7 receiver directly over USB, directly over a serial port or a virtual serial port (connected via USB to your receiver).
You must make sure your computer can reach the OEM7 receiver at the given port, or the novatel_oem7_driver won't be able to connect.

Newer Linux Kernels will show your OEM7 receiver when connected over USB (visible via the 'lsusb' command).

.. Note::
   On Ubuntu Linux systems, for a user account to use a serial device, the account must be a member of the 'dialout' user group.

If you do not see your NovAtel receiver listed by 'lsusb' when USB connected and powered-on, then you must configure your Linux system to attach the USB device to the 'usbserial' kernel module.



When your receiver is connected via USB and properly configured the receiver will enumerate under /dev/ttyUSB*, such as /dev/ttyUSB0.

.. Important::
   novatel_oem7_driver could be launched via USB with:
   ::

      ros2 launch novatel_oem7_driver oem7_port.launch.py oem7_port_name:=/dev/ttyUSB0

Note that OEM7 receivers connected via USB do not need the baud rate set.

.. _Operation_over_Serial:

Operation over Serial
~~~~~~~~~~~~~~~~~~~~~

The novatel_oem7_driver can be used to communicate with your OEM7 receiver directly over a serial port.
Ensure your computer can reach the OEM7 receiver at the given port, or the novatel_oem7_driver won't be able to connect.

Note that on Ubuntu Linux systems, for a user account to use a serial device, the account must be a member of the 'dialout' user group.

For serial connections, a baud rate must be set. The default is 9600 to align with the receiver's default.
It is strongly recommended to use a higher baud rate to avoid messages being dropped or delayed.

In novatel_oem7_driver you can optionally set the baud rate such as to
115200, using the argument ``oem7_port_baud:=115200``

.. Important::
   The novatel_oem7_driver can be launched over a serial connection with:
   ::

      ros2 launch novatel_oem7_driver oem7_port.launch.py oem7_port_name:=/dev/ttyS1 oem7_port_baud:=115200

.. Note::

   This baud rate must correspond to your receiver's internal COM port configuration.
   To manually change the connection parameters of a COM port within your OEM7 receiver, please refer to the Serial Port Communications documentation in the OEM7 Documentation Portal `here <https://docs.novatel.com/OEM7/Content/Operation/Serial_Port_Communications.htm>`__.

--------------

.. _Runtime_OEM7_Reconfiguration:

Runtime Reconfiguration
-----------------------

The driver implements ROS Service **/novatel/oem7/Oem7Cmd novatel_oem7_msgs/srv/Oem7AbasciiCmd**.
This service allows the user to send OEM7 Abbreviated ASCII commands from the terminal to the receiver using the ``ros2 service`` tool.

**Oem7Cmd syntax**

::

   ros2 service call /novatel/oem7/Oem7Cmd novatel_oem7_msgs/srv/Oem7AbasciiCmd "{cmd: '$oem7_abbreviated_ascii_command'}""

**Oem7Cmd example**

::

   ros2 service call /novatel/oem7/Oem7Cmd novatel_oem7_msgs/srv/Oem7AbasciiCmd "{cmd: 'EXTERNALPVAS 0.0 0.0 0.0 0.4502 10.54 -0.09598 0.0 0.0 0.0 0.0 0.0 0.0 0.01 0.01 0.01 0.0 0.0 0.0 4000 10'}"
   requester: making request: novatel_oem7_msgs.srv.Oem7AbasciiCmd_Request(cmd='EXTERNALPVAS 0.0 0.0 0.0 0.4502 10.54 -0.09598 0.0 0.0 0.0 0.0 0.0 0.0 0.01 0.01 0.01 0.0 0.0 0.0 4000 10')

   response:
   novatel_oem7_msgs.srv.Oem7AbasciiCmd_Response(rsp='OK')

The above command of example of `EXTERNALPVAS <https://docs.novatel.com/OEM7/Content/SPAN_Commands/EXTERNALPVAS.htm>`__ provides the SPAN engine an external update that would be derived from an external sensor:

Note that the OEM7 response is printed out verbatim.
This is to notify the user if the command was accepted or if there was an error.
All standard OEM7 abbreviated ASCII commands are supported.
The commands are executed as-is, the syntax is not modified.
``Oem7Cmd`` does not support binary or ASCII command input.

--------------

.. _Recording_data_to_a_ROS_bag:

Recording data to a ROS bag
---------------------------

.. _Configuring_OEM7_Logs_in_ROS_bag:

Configuring OEM7 Logs in ROS bag
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The novatel_oem7_driver requests NovAtel OEM7 logs as part of normal operation.
Logs specified in the ``std_oem7_raw_msgs.yaml`` file are recorded in ROS under the ``/novatel/oem7/oem7raw`` topic.

.. _Record_a_ROS_bag:

Record a ROS bag
~~~~~~~~~~~~~~~~

The novatel_oem7_driver does not automatically record the ROS bag.
The user must choose to record the bag as required for the application.
This can be done using the ``ros2 bag`` command-line tool and the ``record`` command.

Recording the bag is required if NovAtel OEM7 logs are needed for troubleshooting or post-processing.
It is highly recommended to collect oem7raw.

For instance, to record all topics with ``ros2 bag``, for later use and processing, run:

::

   ros2 bag record -a

ROS bags containing the ``/novatel/oem7/oem7raw`` topic can then have their data extracted and concatenated to then be loaded into NovAtel Application Suite to convert the data into ASCII, KML, or RINEX formats.
Extracted logs can also be loaded into Inertial Explorer for post-processing.

--------------

.. _Viewing_live_ROS_topic_data:

Viewing live ROS topic data
---------------------------

You may wish to observe data published in a ROS topic at runtime. You can use the ``ros2 topic`` command for this.

To see available topics, you can run:

::

   ros2 topic list

Here are some typical novatel_oem7_topics this command returns when you have the novatel_oem7_driver installed:

::

   /novatel/oem7/bestgnsspos
   /novatel/oem7/bestgnssvel
   /novatel/oem7/bestpos
   /novatel/oem7/bestutm
   /novatel/oem7/bestvel
   /novatel/oem7/corrimu
   /novatel/oem7/fix
   /novatel/oem7/gps
   /novatel/oem7/heading2
   /novatel/oem7/imu/data
   /novatel/oem7/imu/data_raw
   /novatel/oem7/insconfig
   /novatel/oem7/inspva
   /novatel/oem7/inspvax
   /novatel/oem7/insstdev
   /novatel/oem7/odom
   /novatel/oem7/odom_origin
   /novatel/oem7/oem7raw
   /novatel/oem7/ppppos
   /novatel/oem7/rxstatus
   /novatel/oem7/terrastarinfo
   /novatel/oem7/terrastarstatus
   /novatel/oem7/time


Suppose you wanted to then observe BESTPOS topic output, you could run the command:

::

   ros2 topic echo /novatel/oem7/bestpos
