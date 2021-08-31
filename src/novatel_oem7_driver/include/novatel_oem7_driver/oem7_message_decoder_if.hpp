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

#ifndef __OEM7_MESSAGE_PARSER_IF_HPP__
#define __OEM7_MESSAGE_PARSER_IF_HPP__


#include <cstddef>
#include <boost/asio/buffer.hpp>
#include <boost/shared_ptr.hpp>

#include <oem7_raw_message_if.hpp>
#include <novatel_oem7_driver/oem7_receiver_if.hpp>

#include "novatel_oem7_msgs/msg/oem7_raw_msg.hpp"

#include <rclcpp/rclcpp.hpp>

namespace novatel_oem7_driver
{
  /**
   * Interface implemented by Oem7RawMessageParserIf users, in order to receiver message callbacks.
   */
  class Oem7MessageDecoderUserIf
  {
  public:
    virtual ~Oem7MessageDecoderUserIf(){};

    /**
     * Called when new message is available.
     */
    virtual void onNewMessage(std::shared_ptr<const novatel_oem7::Oem7RawMessageIf>) = 0;
  };


  /**
   * Interface for accessing Oem7 message decoder.
   * The user is responsible for initializing it, and calling service from its preferred context.
   * Any callbacks on Oem7RawMessageParserUserIf are made from the service context.
   */
  class Oem7MessageDecoderIf
  {
  public:
    virtual ~Oem7MessageDecoderIf(){}

    /**
     * Initializes the parser.
     * @return true on success
     */
    virtual bool initialize(
        rclcpp::Node&            nh,    /**< [in] handle of the owner node. Parser uses it to access ROS environment. */
        Oem7ReceiverIf*             recvr, /**< [in] Receiver interface used for data input */
        Oem7MessageDecoderUserIf* user   /**< [in] Interface to receiver message callbacks */
        ) = 0;

    /**
     * Message decoder service loop; blocks as long as input is available.
     * Returns when no more input is available; or when ros::ok() returns false.
     */
    virtual void service() = 0;
  };
}


#endif
