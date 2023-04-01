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

#include <novatel_oem7_driver/oem7_message_util.hpp>

#include "novatel_oem7_driver/oem7_messages.h"

namespace
{
  bool is_initialized = false;

  std::map<std::string, int> oem7_msg_id_map;   ///< Mapping of Oem7 Message Names to IDs
  std::map<int, std::string> oem7_msg_name_map; ///< Mapping of Oem7 Message IDs to Names.

}

namespace novatel_oem7_driver
{

  enum GPSReferenceTimeStatus
  {
    GPS_REFTIME_STATUS_UNKNOWN = 20 // Refer to Oem7 manual.
  };

  void initializeOem7MessageUtil(rclcpp::Node& nh)
  {
    if(is_initialized)
      return;

    is_initialized = true;
  }


  /**
   * Gets Oem7 Binary message header from raw message
   *
   */
  void getOem7Header(
      const Oem7RawMessageIf::ConstPtr& raw_msg,
      novatel_oem7_msgs::msg::Oem7Header& hdr
      )
  {
    const Oem7MessageHeaderMem* hdr_mem = reinterpret_cast<const Oem7MessageHeaderMem*>(raw_msg->getMessageData(0));

    hdr.message_id             = hdr_mem->message_id;
    hdr.message_type           = hdr_mem->message_type;
    hdr.sequence_number        = hdr_mem->sequence;
    hdr.time_status            = hdr_mem->time_status;
    hdr.gps_week_number        = hdr_mem->gps_week;
    hdr.gps_week_milliseconds  = hdr_mem->gps_milliseconds;
    hdr.receiver_status        = hdr_mem->receiver_status;
    hdr.idle_time              = hdr_mem->idle_time / 2.0; // Refer to Oem7 manual
  }

  void getOem7ShortHeader(
      const Oem7RawMessageIf::ConstPtr& raw_msg,  ///< [in] Raw binary message
      novatel_oem7_msgs::msg::Oem7Header& hdr     ///< [out] Oem7 Message Header
      )
  {
    const Oem7MessgeShortHeaderMem* hdr_mem = reinterpret_cast<const Oem7MessgeShortHeaderMem*>(raw_msg->getMessageData(0));

    hdr.message_id             = hdr_mem->message_id;
    hdr.message_type           = novatel_oem7_msgs::msg::Oem7Header::OEM7MSGTYPE_LOG; // Always log
    hdr.sequence_number        = 0; // Not available; assume it's a single log.
    hdr.time_status            = GPS_REFTIME_STATUS_UNKNOWN;
    hdr.gps_week_number        = hdr_mem->gps_week;
    hdr.gps_week_milliseconds  = hdr_mem->gps_milliseconds;
    hdr.idle_time              = 255; // max uchar, clearly invalid value.
  }


  /**
   * Determines if this is NMEA0183 Oem7 message
   */
  bool isNMEAMessage(const Oem7RawMessageIf::ConstPtr& raw_msg)
  {
    return std::find(OEM7_NMEA_MSGIDS.begin(),
                     OEM7_NMEA_MSGIDS.end(),
                     raw_msg->getMessageId()) != OEM7_NMEA_MSGIDS.end();
  }

  size_t Get_INSCONFIG_NumTranslations(const INSCONFIG_FixedMem* insconfig)
  {
    const uint8_t* mem = reinterpret_cast<const uint8_t*>(insconfig) + sizeof(INSCONFIG_FixedMem);
    return *reinterpret_cast<const uint32_t*>(mem); // Safe because all fields are guaranteed to be aligned.
  }

  const INSCONFIG_TranslationMem* Get_INSCONFIG_Translation(const INSCONFIG_FixedMem* insconfig, size_t idx)
  {
    const uint8_t* mem = reinterpret_cast<const uint8_t*>(insconfig) +
                    sizeof(INSCONFIG_FixedMem) +
                    sizeof(uint32_t) +
                    sizeof(INSCONFIG_TranslationMem) * idx;

    return reinterpret_cast<const INSCONFIG_TranslationMem*>(mem);
  }

  size_t Get_INSCONFIG_NumRotations(const INSCONFIG_FixedMem* insconfig)
  {
    const uint8_t* mem = reinterpret_cast<const uint8_t*>(insconfig) +
                   sizeof(INSCONFIG_FixedMem) +
                   sizeof(uint32_t) +
                   sizeof(INSCONFIG_TranslationMem) * Get_INSCONFIG_NumTranslations(insconfig);

    return *reinterpret_cast<const uint32_t*>(mem);
  }


  const INSCONFIG_RotationMem* Get_INSCONFIG_Rotation(const INSCONFIG_FixedMem* insconfig, size_t idx)
  {
    const uint8_t* mem = reinterpret_cast<const uint8_t*>(insconfig) +
                    sizeof(INSCONFIG_FixedMem) +
                    sizeof(uint32_t) +
                    sizeof(INSCONFIG_TranslationMem) * Get_INSCONFIG_NumTranslations(insconfig) +
                    sizeof(uint32_t) +
                    sizeof(INSCONFIG_RotationMem) * idx;

    return reinterpret_cast<const INSCONFIG_RotationMem*>(mem);
  }


  size_t Get_PSRDOP2_NumSystems(const PSRDOP2_FixedMem* psrdop2)
  {
    const uint8_t* mem = reinterpret_cast<const uint8_t*>(psrdop2) + sizeof(PSRDOP2_FixedMem);
    return *reinterpret_cast<const uint32_t*>(mem); // Safe because all fields are guaranteed to be aligned.
  }

  const PSRDOP2_SystemMem* Get_PSRDOP2_System(const PSRDOP2_FixedMem* psrdop2, size_t idx)
  {
    const uint8_t* mem = reinterpret_cast<const uint8_t*>(psrdop2) +
                    sizeof(PSRDOP2_FixedMem) +
                    sizeof(uint32_t) +
                    sizeof(PSRDOP2_SystemMem) * idx;

    return reinterpret_cast<const PSRDOP2_SystemMem*>(mem);
  }


}





