#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <oem7_ros_publisher.hpp>

#include <ros/ros.h>


#include <vector>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include "novatel_oem7_msgs/TERRASTARINFO.h"


namespace novatel_oem7_driver
{

  class TERRASTARINFOHandler: public Oem7MessageHandlerIf
  {
    Oem7RosPublisher TERRASTARINFO_pub_;

    std::string frame_id_;

  public:
    TERRASTARINFOHandler()
    {
    
    }

    ~TERRASTARINFOHandler()
    {
    }

    void initialize(ros::NodeHandle& nh)
    {
      TERRASTARINFO_pub_.setup<novatel_oem7_msgs::TERRASTARINFO>("TERRASTARINFO", nh);
    }

    const std::vector<int>& getMessageIds()
    {
      static const std::vector<int> MSG_IDS({TERRASTARINFO_OEM7_MSGID});
      return MSG_IDS;
    }

    void handleMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      boost::shared_ptr<novatel_oem7_msgs::TERRASTARINFO> terrastarinfo;
      MakeROSMessage(msg, terrastarinfo);

      TERRASTARINFO_pub_.publish(terrastarinfo);
    }
  };



}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::TERRASTARINFOHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
