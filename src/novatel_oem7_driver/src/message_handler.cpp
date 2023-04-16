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


namespace novatel_oem7_driver
{
 /**
 * Constructs the handler by loading the plugins based on user-defined parameters
 */
  MessageHandler::MessageHandler(rclcpp::Node& node):
    node_(node),
    msg_handler_loader_("novatel_oem7_driver", "novatel_oem7_driver::Oem7MessageHandlerIf"),
    msg_filter_(MSGFLAG_NONE)
  {
    // Load the plugins and create the dispatch table.
    std::vector<std::string> init_array = {"", ""};
    node.declare_parameter("oem7_msg_handlers", init_array);
    rclcpp::Parameter msg_handlers_param = node.get_parameter("oem7_msg_handlers");
    std::vector<std::string> msg_handler_names = msg_handlers_param.as_string_array();
    for(const auto& name : msg_handler_names)
    {
      MessageHandlerIf msg_handler = msg_handler_loader_.createSharedInstance(name);

      msg_handler->initialize(node);
      msg_handler_list_.push_back(msg_handler);

      for(auto& msg_rec: msg_handler->getMessageIds())
      {
        MessageHandlerMap::iterator itr = msg_handler_map_.find(msg_rec.first);
        if(itr == msg_handler_map_.end())
        {
          msg_handler_map_[msg_rec.first].reset(new MsgHandlerRecordList);
        }

        msg_handler_map_[msg_rec.first]->push_back(MessageHandlerRecord(msg_handler, msg_rec.second));
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
      RCLCPP_DEBUG_STREAM(node_.get_logger(), "No handler for message ID= " << raw_msg->getMessageId());
      return;
    }

    auto& msg_handler_list = itr->second;
    for(auto& msg_handler_rec: *msg_handler_list)
    {
      if(msg_filter_ == MSGFLAG_ALL || msg_filter_ & msg_handler_rec.second)
      {
        msg_handler_rec.first->handleMsg(raw_msg);
      }
    }
  }

  /**
   * Sets a mask to filter out messages
   */
  void MessageHandler::setMessageFilter(unsigned int filter)
  {
    msg_filter_ = filter;    
  }
  
}

