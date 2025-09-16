.. _Position_Configuration:

FAQ: How to Configure the Receiver for Different Positioning Types?
===================================================================

The receiver is highly configurable for a variety of position types. The goal of this page is to point you in the direction of different configuration options.

Refer to this page for more details on position configuration `here <https://docs.novatel.com/OEM7/Content/ReceiverOperation/PositioningOverview.htm>`__.

This page only covers GNSS configuration, SPAN users must also consider the SPAN configuration.


.. _RTK_Correction_Services:

Configuring RTK Corrections
---------------------------

NovAtel RTK enabled receivers can use RTK corrections to improve real-time position accuracy to as high as 1cm + 1ppm RMS.

Depending on your use case you can either connect to a single base setup, multi base setup or to a network RTK service.

For information for getting started on a single base setup refer `here <https://docs.novatel.com/OEM7/Content/Operation/Transmit_Receive_Corrections.htm>`__.

For information on getting started with a network RTK service refer `here <https://hexagondownloads.blob.core.windows.net/public/Novatel/assets/Documents/Bulletins/apn041/apn041.pdf>`__.

.. _PPP_Correction_Services:

Configuring PPP Correction Services
-----------------------------------

.. _TerraStar.2C_Oceanix_and_VERIPOS_Correction_Services_Overview:

TerraStar, Oceanix and VERIPOS Correction Services Overview
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

NovAtel OEM7 receivers can use NovAtel Correction Services to receive PPP corrections.
This can improve your real-time position accuracy to as high as 2.5cm RMS (via TerraStar-X™).
NovAtel offers correction services for different applications:

.. container::

   +-----------------------------------+-----------------------------------+
   | **Correction Service**            | **Application Types**             |
   +-----------------------------------+-----------------------------------+
   | TerraStar™                        | Various applications including    |
   |                                   | precision agriculture, survey,    |
   |                                   | automotive and aerial.            |
   +-----------------------------------+-----------------------------------+
   | Oceanix                           | Marine applications including     |
   |                                   | dredging, hydrographic survey and |
   |                                   | mapping.                          |
   +-----------------------------------+-----------------------------------+
   | VERIPOS                           | Offshore Oil and Gas              |
   +-----------------------------------+-----------------------------------+


.. _Correction_Service_Activation:

Correction Service Activation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To use NovAtel Correction Services with the novatel_oem7_driver, you must:

-  Have a receiver model that supports corrections
-  Have an active subscription for your receiver's serial number (PSN)
-  Have configured your receiver appropriately, for example: `https://docs.novatel.com/OEM7/Content/Operation/TerraStarCorrectionServices.htm <https://docs.novatel.com/OEM7/Content/Operation/TerraStarCorrectionServices.htm>`__


To purchase a subscription, please contact your NovAtel Sales representative.
To request a temporary or trial subscription, please contact `NovAtel Customer Support <https://shop.novatel.com/s/contactsupport>`__.


