.. _Changing_Topic_Rates:

FAQ: How to Change the Publishing Rates of Topics?
==================================================

.. _Oem7_Specific_Topics:

OEM7 Specific Topics
--------------------
Most OEM7 specific topics (such as ``/novatel/oem7/bestpos``) are generally driven by a single log as defined in ``config/std_init_commands.yaml``.

Most published logs are either asynchronous (published when available, indicated by a ONNEW or ONCHANGED trigger) or synchronous (published at a periodic rate, indicated by a ONTIME trigger). 
For more information about the LOG command, see here: https://docs.novatel.com/OEM7/Content/Commands/LOG.htm
The rate of the asynchronous logs can generally not be changed.
The rate of synchronous logs can often be changed. For example ``config/std_init_commands.yaml`` calls:

::

    - "LOG BESTPOSB ONTIME 0.1"

If you want ``/novatel/oem7/bestpos`` at a higher/lower rate you can modify the set frequency in ``config/std_init_commands.yaml``.
For example, change the command to below to have BESTPOS log at 20Hz instead:

::

    - "LOG BESTPOSB ONTIME 0.05"

.. warning::
    Each log will have a max rate that it can be logged at.
    Depending on the receiver model, most non-SPAN logs will generally be limited to either a maximum of 20Hz or 100Hz.

.. warning::
    Increasing the logging rate may negatively impact the CPU usage of the receiver.
    If increasing the logging rate, other logs may need to be reduced in rate or stopped.

.. _ROS_Standard_Topics:

ROS Standard Topics
--------------------
The ROS standard topics publishing is driven by various NovAtel logs with a single log often being the driving source.
To increase the rate of these topics you will need to identify the log that drives the topic rate. This can be identified in :ref:`Topic_Mapping_and_Sourcing`

Once the command the drives the topic is known, you can then modify ``config/std_init_commands.yaml`` to increase the rate of that log.

.. _CPT7_Rate:

CPT7 IMU Data Rate
------------------
By default, the CPT7 and CPT7700 are configured for a 100Hz raw IMU data rate.

This rate can generally be changed to 400Hz if required. Information about that can be found `here <https://docs.novatel.com/OEM7/Content/SPAN_Operation/CPT7_DataRateConfig.htm>`__.

.. warning::
    Increasing the logging rate may negatively impact the CPU usage of the receiver.
    Closely monitor the CPU usage if increasing the IMU rate.