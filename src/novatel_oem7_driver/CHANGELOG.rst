^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package novatel_oem7_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.0 (2022-03-04)
------------------
* Adding new messages

* Support BESTGNSSPOS log topic publish

3.0.0 (2022-02-23)
------------------
* Adding new messages

* Support PPPPOS, TERRASTARSTATUS, TERRASTARINFO log topic publish

2.2.0 (2021-02-03)
------------------
* No feature changes

* More robust support for compillation for 32 vs. 64 bit targets


2.1.0 (2021-01-28)
------------------

* Support for ARM builds: arm32v7, arm64v8 (`#1 <https://github.com/novatel/novatel_oem7_driver/issues/1>`_)

* Initialization command mechanism robustness improvements


2.0.0 (2020-12-18)
------------------
* Support logging 'unknown' OEM7 messages under Oem7Raw topic
   
  
* Populate gps_common/GPSStatus.status with more detailed status info

* Source gps_common/GPSFix, gps_common/NavSatFix position data from the most recent and higher quality
  INSPVAS or BESTPOS/BESTVEL messsage.
  
  Previously, position data was always sourced from BESTPOS/BESTVEL, which is transmitted
  at lower rate than INSPVAS
  (`#13 <https://github.com/novatel/novatel_oem7_driver/issues/13>`_)   
* Support nmea_msgs/Sentence for all OEM7 NMEA0183 messages (`#4 <https://github.com/novatel/novatel_oem7_driver/issues/4>`_)

* Support nav_msgs/Odometry, with position and orientation populated based on BESTPOS/BESTVEL/INSPVAS
  (`#8 <https://github.com/novatel/novatel_oem7_driver/issues/8>`_)



1.1.0 (2020-05-02)
------------------------
* No feature changes

  Build fixes and documentation improvements.

1.0.0 (2020-04-20)
------------------------------
* Initial release


