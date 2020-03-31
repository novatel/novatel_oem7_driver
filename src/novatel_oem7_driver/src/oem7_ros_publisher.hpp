////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef __OEM7_ROS_PUBLISHER_HPP__
#define __OEM7_ROS_PUBLISHER_HPP__


#include <novatel_oem7_driver/ros_messages.hpp>


namespace novatel_oem7_driver
{

/**
 * Encapsulates ROS message publisher, configured and enabled based on ROS parameters.
 */
class Oem7RosPublisher
{
  ros::Publisher  ros_pub_; ///< ROS publisher

  std::string frame_id_; ///< Configurable frame ID.

public:

  template<typename M>
  void setup(const std::string& name, ros::NodeHandle& nh)
  {
    typedef std::map<std::string, std::string> message_config_map_t;

    message_config_map_t message_config_map;
    nh.getParam(name, message_config_map);

    message_config_map_t::iterator topic_itr = message_config_map.find("topic");
    if(topic_itr == message_config_map.end())
    {
      ROS_WARN_STREAM("Message '" << name << "' will not be published.");
      return;
    }

    int queue_size = 100; // default size

    message_config_map_t::iterator q_size_itr  = message_config_map.find("queue_size");
    if(q_size_itr != message_config_map.end())
    {
      std::stringstream ss(q_size_itr->second);
      ss >> queue_size;
    }

    message_config_map_t::iterator frame_id_itr  = message_config_map.find("frame_id");
    if(frame_id_itr != message_config_map.end())
    {
      frame_id_ = frame_id_itr->second;
    }

    ROS_INFO_STREAM("topic [" << topic_itr->second << "]: frame_id: '" << frame_id_ << "'; q size: " << queue_size);
    ros_pub_ = nh.advertise<M>(topic_itr->second, queue_size);
  }

  /**
   * @return true if the publisher is enabled
   */
  bool isEnabled()
  {
    return !ros_pub_.getTopic().empty();
  }

  /**
   * Publish a message on this publisher. The message is ignored when the publisher is disabled.
   */
  template <typename M>
  void publish(boost::shared_ptr<M>& msg)
  {
    if(!isEnabled())
    {
      return;
    }

    SetROSHeader(frame_id_, msg);
    ros_pub_.publish(msg);
  }

};

}
#endif
