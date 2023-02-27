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

#ifndef __OEM7_ROS_MESSAGES_HPP__
#define __OEM7_ROS_MESSAGES_HPP__

#include <ros/ros.h>

#include "oem7_raw_message_if.hpp"
using novatel_oem7::Oem7RawMessageIf;

#include "novatel_oem7_driver/ros_messages.hpp"
#include "novatel_oem7_driver/oem7_message_ids.h"

namespace novatel_oem7_driver
{

  template <class T>
  void
  MakeROSMessage(const Oem7RawMessageIf::ConstPtr& msg, boost::shared_ptr<T>& rosmsg);

  void
  GetDOPFromPSRDOP2(
      const Oem7RawMessageIf::ConstPtr& msg,
      uint32_t system_to_use,
      double&      gdop,
      double&      pdop,
      double&      hdop,
      double&      vdop,
      double&      tdop);
}
#endif
