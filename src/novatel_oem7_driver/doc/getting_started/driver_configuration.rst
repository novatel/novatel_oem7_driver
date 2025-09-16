.. _Configuration_of_novatel_oem7_driver:

Configuration of novatel_oem7_driver
====================================

.. _Configuration_Files:

There are two ways that the driver can be configured:

-    Through the .yaml files in ``/config``
-    Through the launch.py files ``/launch``

If the binary driver is installed, the path to the config files is:

::

   /opt/ros/${ROS_DISTRO}/share/novatel_oem7_driver/config/


.. Note::
   A default configuration is preprogrammed into the driver, so minimal configuration on the driver side is required, especially for the very first launch.
   Additional configuration can be done to modify driver behaviour, connection settings, topic publishing, publishing rates, etc.

   Configuration on the receiver side is almost always required for successful operation.
   This includes proper setup of the following:

   -    Communication Protocol Configuration - :ref:`OEM7_Hardware_Communications`
   -    GNSS Positioning Type Configuration - :ref:`Position_Configuration`
   -    SPAN Configuration - Refer to both :ref:`std_init_commands` and our best practices on SPAN setup: `APN-109-SPAN-Overview-and-Integration-Guide <https://hexagondownloads.blob.core.windows.net/public/Novatel/assets/Documents/Papers/APN-109-SPAN-Overview-and-Integration-Guide/APN-109-SPAN-Overview-and-Integration-Guide.pdf>`__

.. _Config_Files:

Config Files
------------

The novatel_oem7_driver by Hexagon | NovAtel generates ROS messages from multiple NovAtel OEM7 logs.
OEM7 logs are formatted in an open log format that is heavily documented at the NovAtel Documentation Portal available here:

`https://docs.novatel.com/OEM7/Content/Logs/OEM7_Core_Logs.htm <https://docs.novatel.com/OEM7/Content/Logs/OEM7_Core_Logs.htm>`__

.. _std_init_commands:

std_init_commands.yaml
~~~~~~~~~~~~~~~~~~~~~~
The list of NovAtel commands and log requests sent to the OEM7 receiver upon ROS driver startup are contained in the ``config/std_init_commands.yaml`` file.
Some of the included log commands are required for proper ROS driver operation.
The logs that are needed for the novatel_oem7_driver to function are categorized under a "do not remove these messages" header.
The user may choose to add additional NovAtel configuration commands or log requests if they are customizing the driver.

.. Important::
   std_init_commands.yaml contains additional commented out SPAN configuration commands (i.e. SETINSPROFILE, SETINSTRANSLATION, etc..).
   If additional SPAN configuration is required (i.e. the receiver has not been configured yet for SPAN or its desirable to have the ROS driver do this upon each start), these lines can be uncommented and used. Otherwise the receiver can be configured in advance using manual receiver communication and saving with SAVECONFIG.

oem7_msgs.yaml
~~~~~~~~~~~~~~~~~~~
Messages listed here that are logged (likely via std_init_commands) will be published into the oem7raw topic.
oem7raw is used for post processing and troubleshooting.
As of novatel_oem7_driver version 20.6.0, std_msg_topics.yaml is generated with all supported logs by novatel_oem7_driver so no modifications are required.
Prior to version 20.6.0 in order to add additional logs, list the OEM7 log names along with the corresponding message ID's.
The message ID for each log can be found in the OEM7 manual.

.. Warning::
   Not all NovAtel logs will be supported by the ROS driver.
   The only logs supported by the ROS driver are listed in oem7_msgs.yaml starting with Humble 20.6.0.
   If you would like an additional log to be published into either oem7raw or into a ROS topic, please contact Support to request it.

oem7_supported_imus.yaml
~~~~~~~~~~~~~~~~~~~~~~~~
Supported IMUs for many common setups have been configured in this file. To add an additional IMU modify oem7_support_imus primarily using the information found in the IMU Type table on this page:

`https://docs.novatel.com/OEM7/Content/SPAN_Commands/CONNECTIMU.htm <https://docs.novatel.com/OEM7/Content/SPAN_Commands/CONNECTIMU.htm>`__

std_msg_handlers.yaml
~~~~~~~~~~~~~~~~~~~~~~
After a message from the receiver is decoded by the ROS driver, it is directed to a handler to control the publishing of some topics.
There is no need to modify this file unless adding or removing a handler.

std_msg_topics.yaml
~~~~~~~~~~~~~~~~~~~~~~~~
This file controls which topics are published. To disable a specific topic from publishing you can comment it out here.

.. Important::
   It's highly recommended to publish and collect oem7raw for troubleshooting purposes or to use with post-processing software like Inertial Explorer.

.. Note::
   The RAWIMUSX log/topic is disabled by default due to it's high bandwidth and similarities to imu/data_raw.

.. _Launch_Files:

Launch Files
------------
When launching the driver, you call one of three launch files (or a custom one of your design):

.. container::

   +-----------------------------------+-----------------------------------+
   | **Launch File**                   | **Connection Type**               |
   +-----------------------------------+-----------------------------------+
   | oem7_net.launch.py                | TCP and UDP Connections           |
   +-----------------------------------+-----------------------------------+
   | oem7_port.launch.py               | USB and Serial Connections        |
   +-----------------------------------+-----------------------------------+
   | oem7_file.launch.py               | File Based Input (playback)       |
   +-----------------------------------+-----------------------------------+

More information about how to call the launch file and arguments specific to each launch file can be found at: :doc:`runtime_operation`

There are general parameters in the launch files to modify the behaviour of the driver.
Information about each parameter can be found in the README of the :ref:`Source_Code_of_novatel_oem7_driver`.


.. _Driver_Plugins_std_driver_config.xml_and_std_msg_handlers.yaml:

Driver Plugins std_driver_config.xml and std_msg_handlers.yaml
--------------------------------------------------------------

The novatel_oem7_driver for ROS contains two plugins to be loaded.
There is no need to modify these unless you are developing your own plugins or optimizing the driver footprint.

.. _OEM7_Firmware_And_Model_Considerations:

OEM7 Firmware & Model Considerations
------------------------------------

.. _OEM7_Firmware_for_ROS:

OEM7 Firmware
~~~~~~~~~~~~~

To use the novatel_oem7_driver for ROS with an OEM7 receiver, make sure you are running a current OEM7 firmware for your specific hardware.
To update your OEM7 firmware, go to the NovAtel support site, locate your OEM7 hardware and follow the update instructions.

**SPAN-Enabled receivers**
`novatel.com/support/span-gnss-inertial-navigation-systems <https://novatel.com/support/span-gnss-inertial-navigation-systems>`__

**Smart Antennas**
`novatel.com/support/smart-antennas <https://novatel.com/support/smart-antennas>`__

**GNSS Receivers (Non-SPAN)**
`novatel.com/support/high-precision-gnss-gps-receivers <https://novatel.com/support/high-precision-gnss-gps-receivers>`__

If you require assistance, you may contact NovAtel Customer Support.
Please be sure to include a text copy and paste of your OEM7 receiver's output to the command:

::

   LOG VERSION

You may contact NovAtel Customer Support `here <https://shop.novatel.com/s/contactsupport>`__.

.. _OEM7_Models:

OEM7 Models
~~~~~~~~~~~

NovAtel receivers are able to run a range of functions which are determined by what model is authorized on the OEM7 receiver.
It is possible to purchase upgrades to your model once you have your OEM7 hardware. ROS users may consider this to add functionality such as:

-  Extra GNSS constellation support (Add Support for Glonass, Galileo, Beidou,
   etc.)
-  Add PPP/RTK correction support
-  Add SPAN support (sensor fusion positioning such as to integrate an
   IMU)

To view the latest models please view (note the table on the last page):

`https://novatel.com/products/model-list-and-discontinued-products <https://novatel.com/products/model-list-and-discontinued-products>`__

To purchase a model upgrade, please contact your NovAtel Sales representative.
To request a temporary model upgrade, please contact `NovAtel Customer Support <https://shop.novatel.com/s/contactsupport>`__.
