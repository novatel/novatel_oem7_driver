.. _timing_synchronization:

FAQ: How to Synchronize Timing Between Sensors/Topics?
======================================================

For various sensor fusion application the need may come up to synchronize timing from novatel_oem7_driver ROS topics with other ROS topics.

The timestamp provided in the header of the ROS topics is the GPS measurement time reported by the receiver, when available.
This is a much closer reflection of the exact moment the data was valid, independent of latency introduced by the communication
protocol or ROS callback scheduling. Before the receiver has acquired GPS time (e.g. immediately after startup), the header
timestamp falls back to the time the topic is published.

.. _Synchronize_with_GPS_Time:

GPS Time Stamps in the Header (Preferred Option)
-------------------------------------------------

Most of the custom NovAtel topics have a gps_week_milliseconds field in Oem7Header with some having additional timing fields;
the header timestamp is derived from this same data, so reading the header directly is sufficient for synchronization in most
cases.

If you need the raw GPS week/week-milliseconds representation, or want to detect whether a given message was stamped before
GPS time was acquired, you can still read the fields directly:
::

    /novatel/oem7/inspvax/nov_header/gps_week_number
    /novatel/oem7/inspvax/nov_header/gps_week_milliseconds

For additional synchronization, consider aligning system time with GNSS time using an event-out pulse or a similar mechanism.
Many receiver models offer both a PPS (pulse per second) and PTP (precision timing protocol) options to use for system time synchronization.

Reduce Latency
--------------

Before GPS time has been acquired, or for messages that don't carry a GPS timestamp, minimizing latency is the next best option but it may produce non ideal results.

To minimize latency, using USB or a high serial baud rate is preferred compared to a TCP/UDP connection.

If you are communicating over TCP/UDP, a solution may exist to reduce latency (but not remove it).
However, this solution does not apply to all users and when used incorrectly, user may achieve adverse results. Therefore it is not posted online.
To see if this solution is applicable, please send a message to support.novatel@hexagon.com and reference `ROS GitHub Issue #74 <https://github.com/novatel/novatel_oem7_driver/issues/74>`__.
