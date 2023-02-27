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

#include "message_handler.hpp"

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>



namespace novatel_oem7_driver
{
 /**
 * Constructs the handler by loading the plugins based on user-defined parameters
 */
  MessageHandler::MessageHandler(ros::NodeHandle& nh):
    msg_handler_loader_("novatel_oem7_driver", "novatel_oem7_driver::Oem7MessageHandlerIf")
  {
    // Load the plugins and create the dispatch table.
    std::vector<std::string> msg_handler_names;
    nh.getParam("oem7_msg_handlers", msg_handler_names);
    for(const auto& name : msg_handler_names)
    {
      MessageHandlerShPtr msg_handler = msg_handler_loader_.createInstance(name);

      msg_handler->initialize(nh);

      for(int msg_id: msg_handler->getMessageIds())
      {
        MessageHandlerMap::iterator itr = msg_handler_map_.find(msg_id);
        if(itr == msg_handler_map_.end())
        {
          msg_handler_map_[msg_id].reset(new MsgHandlerList);
        }

        msg_handler_map_[msg_id]->push_back(msg_handler);
      }
    }
  }

  /**
   * Dispatches raw messages to plugins for decoding.
   */
  void MessageHandler::handleMessage(Oem7RawMessageIf::ConstPtr raw_msg)
  {
    MessageHandlerMap::iterator itr = msg_handler_map_.find(raw_msg->getMessageId());
    if(itr == msg_handler_map_.end())
    {
      ROS_DEBUG_STREAM("No handler for message ID= " <<  raw_msg->getMessageId());
      return;
    }

    MessageHandlerListPtr& msg_handler_list = itr->second;
    for(auto& h: *msg_handler_list)
    {
      h->handleMsg(raw_msg);
    }
  }
}

