.. _Setting_Up_NovAtel_Hardware:

Setting Up NovAtel Hardware
===========================

The novatel_oem7_driver by Hexagon | NovAtel works with OEM7 type NovAtel GNSS or SPAN type receivers.
For example, this could include a `PwrPak7 <https://novatel.com/products/receivers/enclosures>`__,
an `OEM7720 <https://novatel.com/products/receivers/gnss-gps-receiver-boards/oem7720>`__,
or some other `SPAN <https://novatel.com/products/gnss-inertial-navigation-systems>`__ hardware.

To configure a NovAtel receiver, you must:

-  Provide power to the receiver
-  Have a way to communicate with the receiver (hardware and software)

--------------

.. _Power_Supply:

Power Supply
------------

For the power supply requirements for your particular receiver, please
refer to the Power Supply Requirements documentation on the `NovAtel Documentation Portal <https://docs.novatel.com/OEM7/>`__.

--------------

.. _OEM7_Hardware_Communications:

OEM7 Communication Options
--------------------------

There are a few different ways you can communicate with your OEM7
receiver using novatel_oem7_driver:

-  RS-232 (a logic level shifter may be required)

-  USB Virtual Serial Ports

-  Ethernet (on supported models)

-  Wi-Fi (on supported models)

OEM7 receivers can also be communicated with via CAN bus, however there is no direct support for this in novatel_oem7_driver.
Therefore, the integrator/user would have to devise a means to provide a virtual TCP/IP, UDP/IP or serial interface to their CAN bus, in order to possibly use novatel_oem7_driver with a CAN-connected OEM7 receiver.

--------------

.. _USB_Communications:

USB Communication
-----------------

Newer Linux Kernels will show your OEM7 receiver when connected over USB (visible via the 'lsusb' command).
Older systems need to be configured to know to connect your OEM7 receiver to the common 'usbserial' kernel module.

.. Note::
  On Ubuntu Linux systems, for a user account to use a serial device, the account must be a member of the 'dialout' user group.

If you do not see your NovAtel receiver listed by 'lsusb' when USB connected and powered-on, then you must configure your Linux system to attach the USB device to the 'usbserial' kernel module.

.. _Optional:_Persistent_USB_driver_configuration_with_usbserial_via_udev:

Optional: Persistent USB driver configuration with usbserial via udev
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a new udev rules file, such as:
::

   /etc/udev/rules.d/z90_novatel.rules

Paste the following contents in to your new udev rules file:

::

   SUBSYSTEM=="usb", SYSFS{idProduct}=="0100", SYSFS{idVendor}=="09d7",
   PROGRAM="/sbin/modprobe usbserial vendor=0x09d7 product=0x0100"

   BUS=="usb", SYSFS{idProduct}=="0100", SYSFS{idVendor}=="09d7",
   SYSFS{product}=="NovAtel GPS Receiver", SYSFS{manufacturer}=="NovAtel Inc.", SYMLINK+="gps%n"

Reboot your system (or restart the necessary services for the changes to take full effect).

.. _Optional:_Temporary_USB_driver_configuration_with_usbserial:

Optional: Temporary USB driver configuration with usbserial
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can use the following line at the command line to temporarily instruct the Linux USB serial kernel module to drive your OEM7 device:

::

   echo '09d7 0100' > /sys/bus/usb-serial/drivers/generic/new_id

.. _Optional:_Legacy_USB_Driver:

Optional: Legacy USB Driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~
Legacy systems may try the NovAtel Linux USB driver. This should not be necessary due to Linux's usbserial kernel module.
You can download the installer for this driver here:

https://novatel.com/support/support-materials/usb-drivers

--------------

.. _Network_Communications:

Network Communication
---------------------

You can use IP networking such as ethernet or Wi-Fi with the novatel_oem7_driver, provided your OEM7 receiver supports IP networking.
This setup process merely requires that you configure your receiver's network configuration so that it's reachable by the machine running novatel_oem7_driver.

The OEM7 Documentation Portal has extensive documentation regarding configuring networking on your receiver. Key pages of this documentation are:

-  `Ethernet Configuration <https://docs.novatel.com/OEM7/Content/Ethernet_Configuration/Ethernet_Configuration.htm>`__
-  `Static IP Configuration <https://docs.novatel.com/OEM7/Content/Ethernet_Configuration/Static_IP_Address_Config.htm>`__
-  `Dynamic IP Configuration <https://docs.novatel.com/OEM7/Content/Ethernet_Configuration/Dynamic_IP_Address_Confi.htm>`__
-  `Wi-Fi Configuration <https://docs.novatel.com/OEM7/Content/Configuration/Wi-FiConfigurationOverview.htm>`__

The application note `Network Configuration on OEM7 <https://hexagondownloads.blob.core.windows.net/public/Novatel/assets/Documents/Papers/D102173-Network-Configuration-on-OEM7/D102173-Network-Configuration-on-OEM7.pdf>`__ further explains networking considerations.

Once you have configured your networking on your receiver you must make sure that the computer running novatel_oem7_driver can communicate with your receiver.
You could use test the network connection to your receiver with NovAtel Application Suite or a command-line utility such as netcat.
Note that NovAtel Application Suite installation instructions are provided further below.

For example, to connect to the ICOM1 port on a typical receiver that is configured with an IP address of 192.168.19.1 with netcat:

::

   nc 192.168.19.1 3001

Once connected, you can send a basic log command to verify that you are indeed connected. For instance, you could request the `VERSION <https://docs.novatel.com/OEM7/Content/Logs/VERSION.htm>`__ log:

::

   LOG VERSION ONCE

If you are connected to your receiver, then the above command will return a log that resembles:

::

   <VERSION USB1 0 72.0 FINESTEERING 2025 247123.828 02000020 3681 14970
   <   11
   <      GPSCARD "FFNRNNCBES1" "BMHR17090005E" "OEM7700-1.00" "OM7CR0500RN0000" "OM7BR0001RB0000" "2018/Jul/10" "14:37:01"
   <      OEM7FPGA "" "" "" "OMV070001RN0000" "" "" ""
   <      WHEELSENSOR "" "" "" "SWS000101RN0000" "" "2018/Jul/10" "14:37:28"
   <      WIFI "RS9113" "" "" "1.6.8" "" "2018/Jul/10" "14:37:32"
   <      APPLICATION "" "" "" "EP7AR0500RN0000" "" "2018/Jul/10" "14:37:13"
   <      DEFAULT_CONFIG "" "" "" "EP7CR0500RN0000" "" "2018/Jul/10" "14:37:23"
   <      PACKAGE "" "" "" "EP7PR0500RN0000" "" "2018/Jul/10" "14:37:18"
   <      DB_WWWISO "WWWISO" "0" "" "WMC010201AN0004" "" "2017/Sep/20" "21:00:04"
   <      ENCLOSURE "" "NMNE17200009B" "" "" "" "" ""
   <      REGULATORY "US" "" "" "" "" "" ""
   <      IMUCARD "Epson G320N" "" "" "" "" "" ""

Once your network is configured and confirmed to be working, then you're ready to configure your novatel_oem7_driver to work with your receiver over your network.

--------------

.. _Tools_for_configuring_your_OEM7_receiver:

Tools for configuring your OEM7 receiver
----------------------------------------

You may interface with your OEM7 receiver using the NovAtel Application Suite for Windows or Ubuntu Linux.
Alternatively, you can directly connect to your receiver via a UDP/TCP, USB or serial connection and configure it manually.

Once you have connected to your receiver, you can configure it for use with the novatel_oem7_driver.
Depending on your use case, you may not need to pre-configure your receiver at all.
Some users will want to pre-configure their receiver to use a specific IP address (for models that have ethernet/Wi-Fi network support).

NovAtel Application Suite
~~~~~~~~~~~~~~~~~~~~~~~~~~~

NovAtel Application Suite is a free software provided by NovAtel to connect to and monitor the receiver.
NovAtel Application Suite for Windows or Ubuntu may be downloaded from the NovAtel Support website `here <https://novatel.com/support/support-materials/software-downloads>`__.

.. _Manual_Configuration:

Manual Configuration
~~~~~~~~~~~~~~~~~~~~

You can directly connect to your OEM7 receiver and manually configure it using a terminal program.
This approach requires that you become familiar with the OEM7 Firmware Commands.
If you would rather use a desktop application, then you should use NovAtel Application Suite.

Refer to :ref:`Notes_on_Manually_Connecting_to_an_OEM7_receiver` for information on connecting via a terminal application.

.. _OEM7_Firmware_Command_Documentation:

OEM7 Firmware Command Documentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The OEM7 Firmware command documentation is available `here <https://docs.novatel.com/OEM7/Content/Commands/OEM7_Core_Commands.htm>`__.

.. _Applications_for_Manual_Receiver_Communication:

Applications for Manual Receiver Communication
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To manually configure your receiver, you will have to use a terminal program that can interact with your receiver.
What program to use depends on how your receiver is connected to your computer.
For RS-232 or USB connections, you can use minicom.
For ethernet or Wi-Fi connections, you can use netcat.
Windows users can use PuTTY to connect directly to their receiver both for IP networking and for serial connections.

.. _Further_Configuration_Assistance:

Further Configuration Assistance
--------------------------------

If you require further assistance with the basic configuration of your OEM7 product, please contact NovAtel Customer Support `here <https://shop.novatel.com/s/contactsupport>`__.
