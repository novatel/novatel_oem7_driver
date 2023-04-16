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

#ifndef __OEM7_MESSAGE_HANDLER_IF_HPP__
#define __OEM7_MESSAGE_HANDLER_IF_HPP__


#include <vector>

#include "oem7_raw_message_if.hpp"
using novatel_oem7::Oem7RawMessageIf;

#include <rclcpp/rclcpp.hpp>

namespace novatel_oem7_driver
{

  // Flags describing message semantics
  const unsigned int MSGFLAG_NONE             = 0;            ///< None; can be used for initialiation.
  const unsigned int MSGFLAG_STATUS_OR_CONFIG = 1 << 0;       ///< Message reports receiver status or configuration. 
  const unsigned int MSGFLAG_ALL              = 0xFFFFFFFF;   ///< All flags set

  /**
   * Interface implemented by modules handling Oem7RawMessageIf messages
   */
  class Oem7MessageHandlerIf
  {
  public:
    typedef std::pair<int, unsigned int> MessageIdRecord;
    typedef std::vector<MessageIdRecord> MessageIdRecords;

    virtual ~Oem7MessageHandlerIf(){};

    /**
     * Initializes the handler
     */
    virtual void initialize(rclcpp::Node&) = 0;

    /**
     * @return a vector of Oem7 message IDs to be handled by this Handler.
     */
    virtual const MessageIdRecords& getMessageIds() = 0;

    /**
     * Handle a message
     */
    virtual void handleMsg(Oem7RawMessageIf::ConstPtr msg) = 0;
  };
}


#endif
