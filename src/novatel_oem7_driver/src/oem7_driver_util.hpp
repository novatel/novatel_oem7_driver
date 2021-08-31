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

#include <stdint.h>
#include "novatel_oem7_msgs/msg/oem7_header.hpp"


namespace novatel_oem7_driver
{
  /*** Converts GSP time to Milliseconds
   *  @return milliseconds
   */
  static inline int64_t GPSTimeToMsec(uint32_t gps_week_no, uint32_t week_msec)
  {
    static const int64_t GPS_MSEC_IN_WEEK = 7 * 24 * 60 * 60 * 1000;
    return GPS_MSEC_IN_WEEK * gps_week_no + week_msec;
  }

  /***
   * Covers GPSTime obtained from header to milliseconds.
   * @return milliseconds
   */
  static inline int64_t GPSTimeToMsec(const novatel_oem7_msgs::msg::Oem7Header& hdr)
  {
    return GPSTimeToMsec(hdr.gps_week_number, hdr.gps_week_milliseconds);
  }
}
