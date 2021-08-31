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

#include <rclcpp/rclcpp.hpp>

#include <message_handler.hpp>

#include "novatel_oem7_msgs/msg/oem7_raw_msg.hpp"
#include "novatel_oem7_driver/oem7_messages.h"



namespace novatel_oem7_driver
{

  /*
   * Adapter: novatel_oem7_msgs::Oem7RawMsg and Oem7RawMsgIf
   */
  class RawMsgAdapter: public Oem7RawMessageIf
  {
  public:
    const novatel_oem7_msgs::msg::Oem7RawMsg::ConstSharedPtr msg_;

    RawMsgAdapter(const novatel_oem7_msgs::msg::Oem7RawMsg::ConstSharedPtr& msg):
      msg_(msg)
    {
    }

    Oem7MessageType getMessageType() const
    {
      assert(false);
      return Oem7RawMessageIf::OEM7MSGTYPE_UNKNOWN;
    }

    Oem7MessageFormat getMessageFormat() const
    {
      assert(false);
      return Oem7RawMessageIf::OEM7MSGFMT_UNKNOWN;
    }

    int  getMessageId() const
    {
      const Oem7MessageCommonHeaderMem* mem =
          reinterpret_cast<const Oem7MessageCommonHeaderMem*>(getMessageData(0));
      return mem->message_id;
    }

    const uint8_t* getMessageData(size_t offset) const
    {
      return const_cast<uint8_t*>(msg_->message_data.data()); // FIXME
    }

    size_t getMessageDataLength() const
    {
      return msg_->message_data.size();
    }
  };
  /*
   * Node responsible for decoding raw Oem7 messages and generating specific ROS and novatel_oem7_msg messages.
   * Subscribes to "oem7_raw_msg", and loads plugins which advertise specific messages.
   * Raw oem7 messages are dispatched to plugins for decoding.
   */
  class Oem7LogNode : public rclcpp::Node
  {
    std::unique_ptr<MessageHandler> msg_handler_;

    rclcpp::Subscription<novatel_oem7_msgs::msg::Oem7RawMsg>::SharedPtr oem7_raw_msg_sub_;
   
  public:
    Oem7LogNode(const rclcpp::NodeOptions& options): 
       rclcpp::Node("Oem7Log", options)
    {
       msg_handler_ = std::make_unique<MessageHandler>(*this);
       oem7_raw_msg_sub_ = create_subscription<novatel_oem7_msgs::msg::Oem7RawMsg>(
		       "oem7_raw_msg", 
		       100, 
		       std::bind(&Oem7LogNode::oem7RawMsgCb, this, std::placeholders::_1));
    }

    /**
     * Dispatches raw messages for handling
     */
    void oem7RawMsgCb(const novatel_oem7_msgs::msg::Oem7RawMsg::ConstSharedPtr msg)
    {
      std::shared_ptr<RawMsgAdapter> raw_msg = std::make_shared<RawMsgAdapter>(msg);
      msg_handler_->handleMessage(raw_msg);
    }
  };
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(novatel_oem7_driver::Oem7LogNode)
