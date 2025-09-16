.. _Advanced_Topics_for_novatel_oem7_driver:

Advanced Topics for novatel_oem7_driver
=======================================

.. _ROS_Driver_Parameter_Namespace_Layout:

ROS2 Driver Parameter Namespace Layout
-------------------------------------=

The novatel_oem7_driver uses standard ROS2 relative parameter addressing, with some parameters private to the node.
The namespace structure can be changed in the ``.launch.py`` file.
The default namespace is: ``/novatel/oem7`` Overall driver namespace; global driver parameters.

--------------

.. _Topic_Parameter_Configuration:

Topic Parameter Configuration
-----------------------------

The topic name and reference frame are configured in ``config/std_msg_topics.yaml`` This file can be modified to fit your needs.

.. _General_Form:

General Form
~~~~~~~~~~~~

The general form for topic parameter configuration is:
``$message_name {topic: $topic_name, frame_id:$id, queue_size: "$size"}``

.. _Example:_INSCONFIG_configuration:

Example: INSCONFIG configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

   INSCONFIG:          {topic: insconfig,        frame_id: gps,  queue_size: 10}

The default configuration, assuming the driver was installed from a released binary, is populated from:

::

   /opt/ros/${ROS_DISTRO}/share/novatel_oem7_driver/config/std_msg_topics.yaml

--------------

.. _Error_Detection_and_Recovery:

Error Detection and Recovery
----------------------------

The novatel_oem7_driver has no internal recovery mechanism.
If the driver goes into an error state, message publishing stops.
The user is responsible for detection of this condition.
Recovery is achieved by relaunching the driver.