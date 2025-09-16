.. _gnss_only_receiver_configuration:

FAQ: How to Configure the Driver for a GNSS Only Receiver?
==========================================================

By default novatel_oem7_driver is configured for a GNSS+INS (SPAN) receiver.
If you are using a GNSS only receiver, the configuration files will need to be modified.
In ``config/std_init_commands``, the following commands will need to be deleted or commented out:

::

    - "LOG INSCONFIGB ONCE"
    - "LOG INSCONFIGB ONTIME 300"
    - "LOG INSPVASB ONTIME 0.02"
    - "LOG INSPVAXB ONTIME 1"
    - "LOG CORRIMUSB ONTIME 0.01"
    - "LOG INSSTDEVB ONTIME 1"
    - "LOG RAWIMUSXB ONNEW"
    - "LOG INSUPDATESTATUSB ONNEW"

.. Warning::
    When using a GNSS only receiver not all ROS topics will be published, some of them may be missing information or publishing rates could be changed.

    Refer to :ref:`Topic_Mapping_and_Sourcing` to see what topics are published with non SPAN logs.
