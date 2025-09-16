.. _Collect_NovAtel_Logs:

FAQ: How to Collect NovAtel Logs?
=================================

There are various use cases where collecting NovAtel logs is required, such as for troubleshooting or post processing.

There are a couple of ways to collect logs and the goal of this page is to outline those options.

.. _oem7raw:

oem7raw (Highly recommended and easiest option)
-----------------------------------------------

The bytes of raw NovAtel logs are published into the topic ``/novatel/oem7/oem7raw``.

By collecting oem7raw data into a ROS bag, you can then extract the raw NovAtel logs by properly extracting the data from the ROS bag or providing this bag file to NovAtel support if it has been requested.

.. note::
    It is highly recommended to generally always be collecting oem7raw data for troubleshooting purposes. The exception to this is in high usage applications where CPU usage is being impacted.

Logs Collected by oem7raw
~~~~~~~~~~~~~~~~~~~~~~~~~

For a log to be collected into oem7raw there are three requirements:

- The log is requested - Such as in ``config/std_init_commands.yaml``.
- The log is listed in ``config/std_oem7_raw_msgs.yaml`` - As of Humble 20.6.0 all supported logs are listed there.
- The log is supported by the driver - Messages are decoded using novatel_oem7_decoder. This is part of another open source project, NovAtel EDIE. If the log is not supported by the version of EDIE used for novatel_oem7_driver then it will not be collected. All EDIE supported logs are listed in ``config/std_oem7_raw_msgs.yaml`` as of Humble 20.6.0.

For updating ``config/std_init_commands.yaml`` and ``config/std_oem7_raw_msgs.yaml`` refer to :ref:`Configuration_of_novatel_oem7_driver`

.. note::
    By default in the driver configuration the oem7raw topic is enabled with a base set of troubleshooting / post-processing logs.


.. _receiver_log_file:

Receiver Log File
-----------------

If a log is required that is not supported by the driver, then a receiver log file can be an alternative option.
The receiver log file captures the raw byte stream of any data bytes collected by novatel_oem7_driver. The data will then be output to a file.

There are two requirements for collecting data the receiver log file:

- The log is requested - Such as in ``config/std_init_commands.yaml``.
- The receiver log file is enabled

To enable the receiver log file, in the relevant launch file, such as ``launch/oem7_net.launch.py`` add or modify a parameter titled ``oem7_receiver_log_file``.
Set the argument to be a file name that will be created as the log file. For example:

::

    'oem7_receiver_log_file' : 'troubleshootingdata.gps',

Run the driver as normal, the receiver log file will then be created in the directory the driver is run in.
