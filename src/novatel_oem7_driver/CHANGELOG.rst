^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package novatel_oem7_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



20.0.0 (2023-04-18)
--------------------
Formal support for Humble; functionality updated to that of ROS1 v4.2.0

Features:

* BESTGNSSPOS, PPPPOS, TERRASTARINFO, TERRASTARSTATUS Oem7 Messages
     
* imu/data_raw output, source from RAWIMUSX and scaled
    
* HG4930_AN04, HG4930_AN04_400Hz IMUs
  
* Odometry Angular velocities

* Optionally, publish Odometry Transform

* Optionally, use first valid GPSFix as Odometry Pose origin


Fixes:

* Rotate Odometry Twist covariances into local frame



10.5.0 (2021-11-12)
--------------------
Misc ROS2 fixes


10.0.0 (2021-08-31)
--------------------
Initial Support for ROS2/Foxy


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


