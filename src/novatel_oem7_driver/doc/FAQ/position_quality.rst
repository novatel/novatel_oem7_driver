.. _position_quality:

FAQ: How to Assess Position Quality?
====================================


Assessing the position quality is a challenge unique to your setup, risk tolerance, and based on the ROS topics that you use.
The below covers an introduction of ways to check the quality of your position:

.. _BESTPOS_Check:

BESTPOS Indicators
------------------

The ``/novatel/oem7/bestpos`` topic may be observed at runtime to observe some key indicators of position quality.

Very similar concepts apply for checking the position quality of ``/novatel/oem7/inpsva`` and ``/novatel/oem7/inspvax``.

Below is an example ``/novatel/oem7/bestpos`` record that will be referred to further below:

::

   header:
     seq: 2603
     stamp:
       secs: 1598887658
       nsecs: 580854223
     frame_id: "gps"
   nov_header:
     message_name: "BESTPOS"
     message_id: 42
     message_type: 0
     sequence_number: 0
     time_status: 180
     gps_week_number: 2121
     gps_week_milliseconds: 142076500
   sol_status:
     status: 0
   pos_type:
     type: 69
   lat: 51.1503897563
   lon: -114.030699187
   hgt: 1097.37025877
   undulation: -17.0
   datum_id: 61
   lat_stdev: 0.017622647807
   lon_stdev: 0.0169686395675
   hgt_stdev: 0.0348801650107
   stn_id: "TSTR"
   diff_age: 16.5
   sol_age: 0.0
   num_svs: 39
   num_sol_svs: 31
   num_sol_l1_svs: 31
   num_sol_multi_svs: 25
   reserved: 0
   ext_sol_stat:
     status: 128
   galileo_beidou_sig_mask: 53
   gps_glonass_sig_mask: 51

Note that these records are populated using the `BESTPOS <https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm>`__ log data from your receiver.

.. _Latitude_Longitude_Height_Standard_Deviations:

Latitude, Longitude, Height Standard Deviations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

   ...
   lat_stdev: 0.017622647807
   lon_stdev: 0.0169686395675
   hgt_stdev: 0.0348801650107
   ...

The BESTPOS standard deviations (*lat_stdev*, *lon_stdev*, *hgt_stdev*) give a good sense of the quality of the position solution.
These are computed separately, in meters, for latitude, longitude and height.
In the above example, they are: *0.017622647807m*, *0.0169686395675m*, *0.0348801650107m*.

.. _Number_of_Satellites_Tracked:

Number of Satellites Tracked
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

   ...
   num_sol_svs: 31
   ...

The *num_sol_svs* gives the number of GNSS satellites tracked that are used in the position solution.
In the above example, this was *31* SVs.

.. _Position_Type:

Position Type
~~~~~~~~~~~~~

::

   ...
   pos_type:
     type: 69
   ...

The Position Type (*pos_type*) reported indicates the method of positioning being reported through the BESTPOS log.
BESTPOS selects the position type with the best real-time performance for your receiver model and environment.
In the above example, this was type *69*. There is a table of position types that lists the type IDs and their corresponding names, available here:

`https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm?Highlight=bestpos#Position_VelocityType <https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm?Highlight=bestpos#Position_VelocityType>`__

In this example, the position type is *PPP*.

Position Type for GPSFix
~~~~~~~~~~~~~~~~~~~~~~~~
``/novatel/oem7/gps`` reports a measurement status.
This measurement status is derived based on the position type the receiver is reporting for the position solution being used.

This status is simplified and it can be used to quickly check if the receiver is in RTK for example.

.. _Solution_Status:

Solution Status
~~~~~~~~~~~~~~~

::

   ...
   sol_status:
     status: 0
   ...

Use the Solution Status (*sol_status*) field to check the BESTPOS log's solution status.
In this example, the status was *0*, which corresponds to the status *SOL_COMPUTED*.
Here's a direct link to the table of solution status ID's and their names:

`https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm?Highlight=bestpos#SolutionStatus <https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm?Highlight=bestpos#SolutionStatus>`__


.. _Time_Status:

Time Status
~~~~~~~~~~~

::

   ...
     time_status: 180
   ...

The *time_status* field gives the GPS solution's time status which is also a key status indicator.
In the above example, the status was *180*, this corresponds to a healthy status of *FINESTEERING*.
Here's the table of status IDs and their names:

`https://docs.novatel.com/OEM7/Content/Messages/GPS_Reference_Time_Statu.htm <https://docs.novatel.com/OEM7/Content/Messages/GPS_Reference_Time_Statu.htm>`__


.. _Diff_Age:

Differential Age
~~~~~~~~~~~~~~~~

::

   ...
     diff_age: 16.5
   ...

The *diff_age* field gives the age of corrections.
This is a way to monitor if RTK corrections are lost.
If the age significantly exceeds the rate of the RTK corrections it can be a sign that RTK corrections are lost.
Additionally, an age of 0 indicates that no differential correction was used.

.. _RXSTATUS_Check:

RXSTATUS Indicators
-------------------

The ``/novatel/oem7/rxstatus`` topic reports various status parameters of the GNSS receiver system.
This includes a variety of warnings and errors.

You can examine an RXSTATUS log to determine the source of the warning or error.
Reviewing this can help with troubleshooting and identifying the source of an issue.

More information about RXSTATUS can be found `here <https://docs.novatel.com/OEM7/Content/Logs/RXSTATUS.htm>`__.
