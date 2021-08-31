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


#include <rclcpp/rclcpp.hpp>

#include <driver_parameter.hpp>


namespace novatel_oem7_driver
{

/**
 * Encapsulates ROS message publisher, configured and enabled based on ROS parameters.
 */
template <typename M>
class Oem7RosPublisher
{
  rclcpp::Node& node_; ///< publishing Node.

  std::shared_ptr<rclcpp::Publisher<M>>  ros_pub_; ///< ROS publisher

  std::string frame_id_; ///< Configurable frame ID.

  std::string topic_; ///< ROS topic name

public:

  Oem7RosPublisher(const std::string& name, rclcpp::Node& node):
    node_(node)
  {
    DriverParameter<std::string> topic_p(   name + ".topic",      "",    node_);
    DriverParameter<std::string> frame_id_p(name + ".frame_id",   "gps", node_);
    DriverParameter<int>         qsize_p(   name + ".queue_size", 100,   node_);

    topic_    = topic_p.value();
    frame_id_ = frame_id_p.value();

    if(!isEnabled())
    {
      RCLCPP_WARN_STREAM(node.get_logger(), "Message '" << name << "' will not be published.");
      return;
    }

    RCLCPP_INFO_STREAM(node.get_logger(), name << ":  topic ["
                                               << topic_
                                               << "]: frame_id: '"
                                               << frame_id_
                                               << "'; q size: "
                                               << qsize_p.value());
    ros_pub_ = node_.create_publisher<M>(topic_, qsize_p.value());
  }

  /**
   * @return true if the publisher is enabled
   */
  bool isEnabled()
  {
    return !topic_.empty();
  }

  /**
   * Publish a message on this publisher. The message is ignored when the publisher is disabled.
   */
  void publish(std::shared_ptr<M> msg)
  {
    if(!isEnabled())
    {
      return;
    }

    msg->header.frame_id = frame_id_;
    msg->header.stamp    = node_.now();

    ros_pub_->publish(*msg);
  }

};

}
#endif
