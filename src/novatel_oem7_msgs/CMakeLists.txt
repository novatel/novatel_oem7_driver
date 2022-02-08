cmake_minimum_required(VERSION 3.0.2)
project(novatel_oem7_msgs)

set(MSG_DEPS 
  std_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${MSG_DEPS}
)

add_service_files(
  FILES
  Oem7AbasciiCmd.srv
)

add_message_files(DIRECTORY msg FILES
  Oem7RawMsg.msg
  Oem7Header.msg
  BESTGNSSPOS.msg
  BESTPOS.msg
  BESTUTM.msg
  PPPPOS.msg
  TERRASTARSTATUS.msg
  TERRASTARINFO.msg
  BESTVEL.msg
  HEADING2.msg
  INSCONFIG.msg
  INSPVA.msg
  INSPVAX.msg
  INSSTDEV.msg
  CORRIMU.msg
  IMURATECORRIMU.msg
  RXSTATUS.msg
  TIME.msg
  INSExtendedSolutionStatus.msg
  INSFrame.msg
  INSReceiverStatus.msg
  INSResetType.msg
  INSSourceStatus.msg
  INSUpdate.msg
  SubscriptionPermission.msg
  SubscriptionType.msg
  SyncState.msg
  RegionRestriction.msg
  LocalAreaStatus.msg
  AccessStatus.msg
  GeogatingStatus.msg
  BestExtendedSolutionStatus.msg
  InertialSolutionStatus.msg
  INSAlignmentMode.msg
  INSOutputFrame.msg
  PositionOrVelocityType.msg
  Rotation.msg
  RotationalOffset.msg
  SolutionStatus.msg
  SolutionSource.msg
  Translation.msg
  TranslationOffset.msg
)
generate_messages(DEPENDENCIES ${MSG_DEPS})
catkin_package(
  CATKIN_DEPENDS message_runtime ${MSG_DEPS}
)
