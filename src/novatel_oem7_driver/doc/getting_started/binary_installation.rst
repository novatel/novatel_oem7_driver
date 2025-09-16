.. _Installing_the_novatel_oem7_driver_Binary_Driver:

Installing novatel_oem7_driver Binary Driver
============================================

There is a pre-built binary release of the novatel_oem7_driver by Hexagon | NovAtel.
It can be installed for ROS, and is available through the ROS distribution.
Typical ROS users will prefer to use this installation method.

Alternatively the driver can be built from source code. The documentation below covers both options.

--------------

.. _Binary_Driver_ROS_Requirements:

Binary Driver ROS Requirements
------------------------------

The novatel_oem7_driver binary installation from the ROS build farm is only available on the supported ROS versions listed here: :ref:`Supported_Versions`

--------------

.. _Binary_Driver_Installation:

Binary Driver Installation
--------------------------

The novatel_oem7_driver is published in the ROS distribution.
Therefore, installation of the binary driver simply requires installing the ``novatel-oem7-driver`` package.

.. important::
   This driver can be installed by the command:

   ::

      sudo apt install ros-${ROS_DISTRO}-novatel-oem7-driver

For example, on a ROS2 Jazzy installation, the command to install the driver would be:

::

   sudo apt install ros-jazzy-novatel-oem7-driver

--------------

.. _Building_the_driver_from_source_code:

Building the Driver from Source Code
------------------------------------

If you cannot use the binary driver, you may consider building the driver from the source code.
The instructions to build from the source code are bundled with the code provided on GitHub `here <https://github.com/novatel/novatel_oem7_driver>`__.

.. note::
   The official branches on GitHub intended for use are named after the distribution they support, e.g. Use branch Humble to build the driver for ROS2 Humble