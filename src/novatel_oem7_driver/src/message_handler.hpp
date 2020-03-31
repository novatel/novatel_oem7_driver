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

#ifndef __MESSAGE_HANDLER_HPP__
#define __MESSAGE_HANDLER_HPP__

#include <ros/ros.h>

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

#include "oem7_raw_message_if.hpp"
using novatel_oem7::Oem7RawMessageIf;

#include "novatel_oem7_driver/oem7_message_decoder_if.hpp"
#include "novatel_oem7_driver/oem7_message_handler_if.hpp"

#include <boost/scoped_ptr.hpp>

namespace novatel_oem7_driver
{
  /**
   * Encapsulates a collection of message handling plugins, where a message
   * a messages is handled by 0 or more plugins, matching the message on ID.
   */
  class MessageHandler
  {
    pluginlib::ClassLoader<novatel_oem7_driver::Oem7MessageHandlerIf> msg_handler_loader_; ///< Plugin loader

    typedef boost::shared_ptr<novatel_oem7_driver::Oem7MessageHandlerIf> MessageHandlerShPtr;
    typedef std::list<MessageHandlerShPtr> MsgHandlerList;
    typedef boost::scoped_ptr<MsgHandlerList> MessageHandlerListPtr;
    typedef std::map<int, MessageHandlerListPtr> MessageHandlerMap;
    MessageHandlerMap msg_handler_map_; ///< Dispatch map for raw messages.

  public:
    MessageHandler(ros::NodeHandle& nh);

    void handleMessage(Oem7RawMessageIf::ConstPtr raw_msg);
  };
}

#endif
