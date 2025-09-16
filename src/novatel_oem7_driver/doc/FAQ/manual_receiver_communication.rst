.. _Notes_on_Manually_Connecting_to_an_OEM7_receiver:

FAQ: How to Manually Connect to the Receiver?
=============================================

These page explains some methods that can be used to directly connect to an OEM7 receiver.
This is useful for troubleshooting or observing real-time operations with your OEM7 receiver.
If you are struggling to launch novatel_oem7_driver it is important to check that you are able to establish a manual connection.


.. _NAS_Connections:

NovAtel Application Suite Connections
-------------------------------------

NovAtel Application Suite is a free software provided by NovAtel to connect to and monitor the receiver.

`https://novatel.com/support/support-materials/software-downloads <https://novatel.com/support/support-materials/software-downloads>`__


.. _Manual_Ethernet_Connections:

Manual Ethernet/Wi-Fi Connections
---------------------------------

OEM7 receivers that support Wi-Fi and/or Ethernet have internal "ICOM" ports you can reach through a TCP or UDP socket.
These ports can be reconfigured to act as either TCP or UDP, client or server sockets.
For more on ICOM configuration, please refer to the `ICOMCONFIG <https://docs.novatel.com/OEM7/Content/Commands/ICOMCONFIG.htm>`__ command documentation.

Most Linux systems have the 'netcat' program preinstalled, or easily added through their package managers.
Netcat is invoked by the 'nc' command.

Suppose you wanted to connect to your receiver's ICOM2 port, by default this is assigned TCP port number 3002.
If your receiver's IP was 192.168.19.1, then the netcat command would be:

::

   nc 192.168.19.1 3002

Once connected, you can send OEM7 commands, for example:

::

   LOG BESTPOSA ONTIME 2

The above command would cause ASCII BESTPOS logs to be written to the port you connected on every 2 seconds.

For troubleshooting networking issues, the `Network Configuration on OEM7 Application Note <https://hexagondownloads.blob.core.windows.net/public/Novatel/assets/Documents/Papers/D102173-Network-Configuration-on-OEM7/D102173-Network-Configuration-on-OEM7.pdf>`__ is a good starting point.

.. _Manual_USB_Connections:

Manual USB/Serial Connections
-----------------------------

OEM7 receivers will have 3 virtual USB ports for each physical USB port.
Therefore, you can simultaneously maintain a manual connection while also running the driver by using one of the other two virtual ports.

Each serial port has one port you can connect to.
It is important for serial connections to have matching baud rates and other settings on both the computer and receiver side.
Most serial port settings can be configured on the receiver side through the `SERIALCONFIG <https://docs.novatel.com/OEM7/Content/Commands/SERIALCONFIG.htm>`__ command:

To connect to the receiver manually with USB/Serial the 'minicom' program can be used.

minicom can be installed with:

::

   apt install minicom

You can then connect to minicom with USB using:
::

   minicom -D /dev/ttyUSBx

where x represents the USB connection within your system.

.. Note::

	The 'lsusb' command can be used to find the specific port number of the device (i.e. ttyUSB0).

You can similarly connect to minicom with serial using:
::

   minicom -b 460800 -D /dev/ttyUSBx

where 460800 represents the same baud rate that was set in the SERIALCONFIG of the receiver.
