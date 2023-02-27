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

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <message_handler.hpp>

#include "novatel_oem7_msgs/Oem7RawMsg.h"
#include "novatel_oem7_driver/oem7_messages.h"

#include <boost/scoped_ptr.hpp>

namespace novatel_oem7_driver
{

  /*
   * Adapter: novatel_oem7_msgs::Oem7RawMsg and Oem7RawMsgIf
   */
  class RawMsgAdapter: public Oem7RawMessageIf
  {
  public:
    const novatel_oem7_msgs::Oem7RawMsg::ConstPtr msg_;

    RawMsgAdapter(const novatel_oem7_msgs::Oem7RawMsg::ConstPtr& msg):
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
   * Nodelet responsible for decoding raw Oem7 messages and generating specific ROS and novatel_oem7_msg messages.
   * Subscribes to "oem7_raw_msg", and loads plugins which advertise specific messages.
   * Raw oem7 messages are dispatched to plugins for decoding.
   */
  class Oem7LogNodelet : public nodelet::Nodelet
  {
    boost::scoped_ptr<MessageHandler> msg_handler_;

    ros::Subscriber oem7_raw_msg_sub_;


    public:
    Oem7LogNodelet()
    {
    }

    void onInit()
    {
      ros::NodeHandle nh = getNodeHandle();
      ros::NodeHandle priv_nh = getPrivateNodeHandle();
      msg_handler_.reset(new MessageHandler(priv_nh));

      oem7_raw_msg_sub_ = nh.subscribe("oem7_raw_msg", 100, &Oem7LogNodelet::oem7RawMsgCb, this);
    }

    /**
     * Dispatches raw messages for handling
     */
    void oem7RawMsgCb(const novatel_oem7_msgs::Oem7RawMsg::ConstPtr& msg)
    {
      boost::shared_ptr<RawMsgAdapter> raw_msg = boost::make_shared<RawMsgAdapter>(msg);
      msg_handler_->handleMessage(raw_msg);
    }
  };
}


PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7LogNodelet, nodelet::Nodelet);
