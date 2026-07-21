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

#ifndef __OEM7_MESSAGE_UTIL_HPP__
#define __OEM7_MESSAGE_UTIL_HPP__

#include "novatel_oem7_msgs/msg/oem7_header.hpp"
#include "novatel_oem7_msgs/msg/oem7_raw_msg.hpp"

#include "novatel_oem7_driver/oem7_message_ids.h"

#include "oem7_raw_message_if.hpp"
using novatel_oem7::Oem7RawMessageIf;

#include "oem7_messages.h"

#include "builtin_interfaces/msg/time.hpp"

#include <cmath>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

namespace novatel_oem7_driver
{
  // GPS time has no leap seconds; convert GPS time to Unix/UTC time via the
  // fixed GPS epoch offset (1980-01-06 -> 1970-01-01) plus the current
  // GPS-UTC leap-second offset. Update kGpsUtcLeapSeconds if IERS announces
  // a new leap second (last one: 2016-12-31, offset 18s).
  inline constexpr int64_t kGpsEpochUnixOffsetSec = 315964800; // 1980-01-06 - 1970-01-01
  inline constexpr int32_t kGpsUtcLeapSeconds = 18;

  inline bool HasValidGpsTime(const novatel_oem7_msgs::msg::Oem7Header& hdr)
  {
    return hdr.gps_week_number != 0;
  }

  inline builtin_interfaces::msg::Time GpsWeekMsToRosTime(uint16_t week, uint32_t week_ms)
  {
    const int64_t gps_ms = static_cast<int64_t>(week) * 604800000LL + week_ms;
    const int64_t unix_ms = gps_ms + kGpsEpochUnixOffsetSec * 1000LL - kGpsUtcLeapSeconds * 1000LL;
    builtin_interfaces::msg::Time out;
    out.sec = static_cast<int32_t>(unix_ms / 1000);
    out.nanosec = static_cast<uint32_t>((unix_ms % 1000) * 1000000LL);
    return out;
  }

  /**
   * Converts GPS time expressed as seconds-since-GPS-epoch (e.g. gps_msgs::msg::GPSFix::time)
   * to Unix/UTC ROS time.
   */
  inline builtin_interfaces::msg::Time GpsSecondsToRosTime(double gps_seconds)
  {
    const double unix_sec = gps_seconds + kGpsEpochUnixOffsetSec - kGpsUtcLeapSeconds;
    builtin_interfaces::msg::Time out;
    out.sec = static_cast<int32_t>(std::floor(unix_sec));
    out.nanosec = static_cast<uint32_t>((unix_sec - std::floor(unix_sec)) * 1e9);
    return out;
  }

  static const std::vector<int> OEM7_NMEA_MSGIDS(
    {
     GLMLA_OEM7_MSGID,
     GPALM_OEM7_MSGID,
     GPGGA_OEM7_MSGID,
     GPGGALONG_OEM7_MSGID,
     GPGLL_OEM7_MSGID,
     GPGRS_OEM7_MSGID,
     GPGSA_OEM7_MSGID,
     GPGST_OEM7_MSGID,
     GPGSV_OEM7_MSGID,
     GPHDT_OEM7_MSGID,
     GPHDTDUALANTENNA_MSGID,
     GPRMB_OEM7_MSGID,
     GPRMC_OEM7_MSGID,
     GPVTG_OEM7_MSGID,
     GPZDA_OEM7_MSGID
    }
  );


  void initializeOem7MessageUtil(rclcpp::Node& nh);
  int getOem7MessageId(const std::string& msg_name);
  const std::string& getOem7MessageName(int msg_id);

  /**
   * Populates Oem7 Binary message header from raw message
   *
   */
  void getOem7Header(
      const Oem7RawMessageIf::ConstPtr& raw_msg, ///< [in] Raw binary message
      novatel_oem7_msgs::msg::Oem7Header& hdr   ///< [out] Oem7 Message Header
      );

  /**
   * Populates Oem7 Binary message header from 'short' raw message
   *
   */
  void getOem7ShortHeader(
      const Oem7RawMessageIf::ConstPtr& raw_msg, ///< [in] Raw binary message
      novatel_oem7_msgs::msg::Oem7Header& hdr   ///< [out] Oem7 Message Header
      );

  bool isNMEAMessage(const Oem7RawMessageIf::ConstPtr& raw_msg);

  size_t Get_INSCONFIG_NumTranslations(const INSCONFIG_FixedMem* insconfig);

  const INSCONFIG_TranslationMem* Get_INSCONFIG_Translation(const INSCONFIG_FixedMem* insconfig, size_t idx);

  size_t Get_INSCONFIG_NumRotations(const INSCONFIG_FixedMem* insconfig);

  const INSCONFIG_RotationMem* Get_INSCONFIG_Rotation(const INSCONFIG_FixedMem* insconfig, size_t idx);


  size_t Get_PSRDOP2_NumSystems(const PSRDOP2_FixedMem* psrdop2);
  const PSRDOP2_SystemMem* Get_PSRDOP2_System(const PSRDOP2_FixedMem* psrdop2, size_t idx);
}


#endif
