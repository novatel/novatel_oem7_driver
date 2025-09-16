.. Position_Generation:

FAQ: What Reference Point are ROS Specific Topics Output At?
============================================================

GpsFix, NavSatFix and Odometry positions
are generated from position information reported by a combination of
the most recent OEM7 BESTPOS, BESTVEL and INSPVAS messages:

-  `BESTPOS <https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm>`__

-  `BESTVEL <https://docs.novatel.com/OEM7/Content/Logs/BESTVEL.htm>`__

-  `INSPVAS <https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSPVAS.htm>`__

3D position quality is used to determine whether BESTPOS/BESTVEL or INSPVA are used.
By default, INSPVAS is reported at a higher rate than BESTPOS/BESTVEL; so INSPVAS is used to generate GpsFix/NavSatFix.
GpsFix is then used to publish Odometry.

Under ideal conditions, the position quality between BESTPOS/BESTVEL and INSPVAS will be similar, but BESTPOS/BESTVEL can be a higher quality than INSPVAS if the GNSS only solution outperforms the GNSS+INS solution.

BESTPOS is published at the L1 phase centre of the primary antenna.
INSPVAS is published at the IMU centre of navigation, unless the command `SETINSTRANSLATION <https://docs.novatel.com/OEM7/Content/SPAN_Commands/SETINSTRANSLATION.htm>`__ USER is set.

Running the driver in it's default configuration may cause position jumps when switching between BESTPOS and INSPVAS.
There are two ways to resolve this:

.. _Force_the_Position_Source:

Manually Select which Position Source to Use
--------------------------------------------

It is possible to override the auto-selection behaviour by updating the launch file.

In the relevant ``.launch.py`` file, under parameters set:
::

   'oem7_position_source' : 'BESTPOS'

or

::

   'oem7_position_source' : 'INSPVAS'

Considerations for using each:

-  "BESTPOS": always use BESTPOS/BESTVEL as position source,
   regardless of availability of a more recent position.
-  "INSPVAS": always use INPSVAS as position source, regardless of 3D
   quality. INSPVAS can also be output at a custom user defined location using SETINSTRANSLATION USER

When 'position_source' is not specified, the most recent message with the highest quality is used. This is the default configuration.

When using INS (INSPVAS) as position source, the driver accounts for undulation automatically.
SPAN lever arm must be configured correctly by the user to obtain correct position/orientation.

.. _Set_INSPVAS_Output_Location:

Set a User Output Location for INSPVAS
--------------------------------------

INSPVAS can be configured to a custom output location.
The custom location can be defined as the L1 phase centre of the primary antenna.
Position jumps would still be present when switching position sources but those jumps would be with reference to the same frame.

The lever arm would have already been set from the IMU to the primary antenna via SETINSTRANSLATION ANT1.
The same parameters applied to SETINSTRANSLATION ANT1 should be applied to SETINSTRANSLATION USER to translate the IMU to the antenna phase centre or to any other desired point (vehicle rear axle, etc.).

.. note::
   It is important to keep in mind that as the USER offset is moved further from the IMU centre of navigation, the PVA solution will become noisier due to the projection of angular changes over a longer distance.
