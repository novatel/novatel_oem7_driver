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
#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <oem7_ros_publisher.hpp>

#include <ros/ros.h>


#include <vector>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include "novatel_oem7_msgs/RXSTATUS.h"


namespace novatel_oem7_driver
{
  typedef std::vector<std::string> str_vector_t;


  /*** Oem7 Receiver errors strings - refer to Oem7 manual */
  const str_vector_t RECEIVER_ERROR_STRS
  {
      "DRAM",
      "Invalid FW",
      "ROM",
      "",
      "ESN Access",
      "AuthCode",
      "",
      "Supply Voltage",
      "",
      "Temperature",
      "MINOS",
      "PLL RF",
      "",
      "",
      "",
      "NVM",
      "Software resource limit exceeded",
      "Model invalid for this receiver",
      "",
      "",
      "Remote loading has begun",
      "Export restriction",
      "Safe Mode",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "Component hardware failure"
  };

  /** Oem7 receiver status strings  - refer to Oem7 manual */
  const str_vector_t RECEIVER_STATUS_STRS
  {
      "Receiver Error Flag",
      "Temperature",
      "Voltage Supply",
      "Primary antenna not powered",
      "LNA Failure",
      "Primary antenna open circuit",
      "Primary antenna short circuit",
      "CPU overload",
      "COM port tx buffer overrun",
      "",
      "",
      "Link overrun",
      "Input overrun",
      "Aux transmit overrun",
      "Antenna gain out of range",
      "Jammer detected",
      "INS reset",
      "IMU communication failure",
      "GPS almanac invalid",
      "Position solution invalid",
      "Position fixed",
      "Clock steering disabled",
      "Clock model invalid",
      "External oscillator locked",
      "Software resource warning",
      "",
      "Interpret Status/Error Bits as Oem7 format",
      "Tracking mode: HDR",
      "Digital filtering enabled",
      "Aux3 event",
      "Aux2 event",
      "Aux1 event"
  };

  /** Auxiliary 1 Status strings - refer to Oem7 manual. */
  const str_vector_t AUX1_STATUS_STRS
  {
    "Jammer detected on RF1",
    "Jammer detected on RF2",
    "Jammer detected on RF3",
    "Position averaging On",
    "Jammer detected on RF4",
    "Jammer detected on RF5",
    "Jammer detected on RF6",
    "USB not connected",
    "USB1 buffer overrun",
    "USB2 buffer overrun",
    "USB3 buffer overrun",
    "",
    "Profile Activation Error",
    "Throttled Ethernet Reception",
    "",
    "",
    "",
    "",
    "Ethernet not connected",
    "ICOM1 buffer overrun",
    "ICOM2 buffer overrun",
    "ICOM3 buffer overrun",
    "NCOM1 buffer overrun",
    "NCOM2 buffer overrun",
    "NCOM3 buffer overrun",
    "",
    "",
    "",
    "",
    "",
    "",
    "IMU measurement outlier detected"
  };

  /** Auxiliary 2 Status strings - refer to Oem7 manual. */
  const str_vector_t AUX2_STATUS_STRS
  {
    "SPI Communication Failure",
    "I2C Communication Failure",
    "COM4 buffer overrun",
    "COM5 buffer overrun",
    "",
    "",
    "",
    "",
    "",
    "COM1 buffer overrun",
    "COM2 buffer overrun",
    "COM3 buffer overrun",
    "PLL RF1 unlock",
    "PLL RF2 unlock",
    "PLL RF3 unlock",
    "PLL RF4 unlock",
    "PLL RF5 unlock",
    "PLL RF6 unlock",
    "CCOM1 buffer overrun",
    "CCOM2 buffer overrun",
    "CCOM3 buffer overrun",
    "CCOM4 buffer overrun",
    "CCOM5 buffer overrun",
    "CCOM6 buffer overrun",
    "ICOM4 buffer overrun",
    "ICOM5 buffer overrun",
    "ICOM6 buffer overrun",
    "ICOM7 buffer overrun",
    "Secondary antenna not powered",
    "Secondary antenna open circuit",
    "Secondary antenna short circuit",
    "Reset loop detected",
  };

  /** Auxiliary 3 Status strings - refer to Oem7 manual. */
  const str_vector_t AUX3_STATUS_STRS
  {
    "SCOM buffer overrun",
    "WCOM1 buffer overrun",
    "FILE buffer overrun",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "Web content is corrupt or does not exist",
    "RF Calibration Data is present and in error",
    "RF Calibration data exists and has no errors"
  };

  const str_vector_t AUX4_STATUS_STRS
  {
    "<60% of available satellites are tracked well",
    "<15% of available satellites are tracked well",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "Clock freewheeling due to bad position integrity",
    "",
    "Usable RTK Corrections: < 60%",
    "Usable RTK Corrections: < 15%",
    "Bad RTK Geometry",
    "",
    "",
    "Long RTK Baseline",
    "Poor RTK COM link",
    "Poor ALIGN COM link",
    "GLIDE Not Active",
    "Bad PDP Geometry",
    "No TerraStar Subscription",
    "",
    "",
    "",
    "Bad PPP Geometry",
    "",
    "No INS Alignment",
    "INS not converged"
  };


  void
  get_status_info(
      uint32_t bitmask,
      const str_vector_t&       str_map,
      str_vector_t&             str_list,
      std::vector<uint8_t>&     bit_list)
  {
    for(int bit = 0;
            bit < (sizeof(bitmask) * 8);
            bit++)
    {
      if(bitmask & (1 << bit))
      {
        bit_list.push_back(bit);
        if(str_map[bit].length() > 0)
        {
          str_list.push_back(str_map[bit]);
        }
      }
    }
  }


  /*** Handles RXSTATUS messages */
  class ReceiverStatusHandler: public Oem7MessageHandlerIf
  {
    Oem7RosPublisher RXSTATUS_pub_;

    std::string frame_id_;

  public:
    ReceiverStatusHandler()
    {
      static const size_t NUM_BITS = sizeof(uint32_t) * 8;
      assert(RECEIVER_ERROR_STRS.size() == NUM_BITS);
      assert(AUX1_STATUS_STRS.size()    == NUM_BITS);
      assert(AUX2_STATUS_STRS.size()    == NUM_BITS);
      assert(AUX3_STATUS_STRS.size()    == NUM_BITS);
      assert(AUX4_STATUS_STRS.size()    == NUM_BITS);
    }

    ~ReceiverStatusHandler()
    {
    }

    void initialize(ros::NodeHandle& nh)
    {
      RXSTATUS_pub_.setup<novatel_oem7_msgs::RXSTATUS>("RXSTATUS", nh);
    }

    const std::vector<int>& getMessageIds()
    {
      static const std::vector<int> MSG_IDS({RXSTATUS_OEM7_MSGID});
      return MSG_IDS;
    }

    void handleMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      boost::shared_ptr<novatel_oem7_msgs::RXSTATUS> rxstatus;
      MakeROSMessage(msg, rxstatus);

      // Populate status strings:
      get_status_info(rxstatus->error,     RECEIVER_ERROR_STRS,  rxstatus->error_strs,     rxstatus->error_bits);
      get_status_info(rxstatus->rxstat,    RECEIVER_STATUS_STRS, rxstatus->rxstat_strs,    rxstatus->rxstat_bits);
      get_status_info(rxstatus->aux1_stat, AUX1_STATUS_STRS,     rxstatus->aux1_stat_strs, rxstatus->aux1_stat_bits);
      get_status_info(rxstatus->aux2_stat, AUX2_STATUS_STRS,     rxstatus->aux2_stat_strs, rxstatus->aux2_stat_bits);
      get_status_info(rxstatus->aux3_stat, AUX3_STATUS_STRS,     rxstatus->aux3_stat_strs, rxstatus->aux3_stat_bits);
      get_status_info(rxstatus->aux4_stat, AUX4_STATUS_STRS,     rxstatus->aux4_stat_strs, rxstatus->aux4_stat_bits);

      RXSTATUS_pub_.publish(rxstatus);
    }
  };



}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::ReceiverStatusHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
