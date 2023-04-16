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


#include <cassert>
#include <vector>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include "novatel_oem7_msgs/msg/rxstatus.hpp"
#include "novatel_oem7_msgs/msg/terrastarinfo.hpp"
#include "novatel_oem7_msgs/msg/terrastarstatus.hpp"


namespace novatel_oem7_driver
{
  typedef std::vector<std::string> str_vector_t;


  /*** Oem7 Receiver errors strings - refer to Oem7 manual */
  const str_vector_t RECEIVER_ERROR_STRS =
  {
      "DRAM",
      "Invalid FW",
      "ROM",
      "",

      "ESN Access",
      "AuthCode",
      "",
      "Supply voltage",

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
      "Error",
      "Temperature",
      "Voltage supply",
      "Primary antenna power",

      "LNA failure",
      "Primary antenna open circuit",
      "Primary antenna short circuit",
      "CPU overload",
      
      "COM port tx buffer overrun",
      "Spoofing detection",
      "Reserved",
      "Link overrun",
      
      "Input overrun",
      "Aux transmit overrun",
      "Antenna gain state",
      "Jammer detected",
      
      "INS reset",
      "IMU communication failure",
      "GPS almanac / UTC known",
      "Position solution",
      
      "Position fixed",
      "Clock steering",
      "Clock model",
      "External oscillator locked",
      
      "Software resource",
      "Version bit 0",
      "Version bit 1",
      "Tracking mode",
      
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
    "Position averaging",
    
    "Jammer detected on RF4",
    "Jammer detected on RF5",
    "Jammer detected on RF6",
    "USB connection",
    
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
    "Status error reported by IMU",
    "IMU measurement outlier detected"
  };

  /** Auxiliary 2 Status strings - refer to Oem7 manual. */
  const str_vector_t AUX2_STATUS_STRS
  {
    "SPI communication failure",
    "I2C communication failure",
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
    
    "Secondary antenna power",
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

    "Anntenna 1 gain state bit 1 ",
    "Anntenna 1 gain state bit 2 ",
    "Anntenna 2 gain state bit 1 ",
    "Anntenna 2 gain state bit 2 ",

    "GPS reference time incorrect",
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

    "Spoofing calibration status",
    "Spoofing calibration required",
    "",
    "",

    "",
    "Web content is corrupt or does not exist",
    "RF calibration data is present and in error",
    "RF calibration data is present"
  };

  const str_vector_t AUX4_STATUS_STRS
  {
    "GNSS tracked status: <60% of available satellites are tracked well",
    "GNSS tracked status: <15% of available satellites are tracked well",
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
    "Usable RTK corrections: <60%",
    "Usable RTK corrections: <15%",

    "Bad RTK geometry",
    "",
    "",
    "Long RTK baseline",

    "Poor RTK COM link",
    "Poor ALIGN COM link",
    "GLIDE Not Active",
    "Bad PDP Geometry",

    "No TerraStar subscription",
    "",
    "",
    "",

    "Bad PPP Geometry",
    "",
    "No INS alignment",
    "INS not converged"
  };


  /**
   * Populate a list of bits and their corresponding strings
   * from a bitmask.
   */
  template <class T, class U>
  void
  get_status_info(
      uint32_t bitmask,                  /** Input bitmask */
      const str_vector_t&  bit2str_map, /** Input strings, one for each bit */
      T&                   str_list,/** Output: list of set bits */
      U&                   bit_list /** Output: list of set bit strings */
      )
  {
    for(int bit = 0;
            bit < (sizeof(bitmask) * 8);
            bit++)
    {
      if(bitmask & (1 << bit)) // Bit is set in bitmask
      {
        bit_list.push_back(bit); // Add bit to list
        str_list.push_back(bit2str_map[bit]); // Add the corresponding string, even it it's empty, to align with bits.
      }
    }
  }


  /*** Handles RXSTATUS messages */
  class ReceiverStatusHandler: public Oem7MessageHandlerIf
  {
    std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::RXSTATUS>>         RXSTATUS_pub_;
    std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::TERRASTARINFO>>    TSTInfo_pub_;
    std::unique_ptr<Oem7RosPublisher<novatel_oem7_msgs::msg::TERRASTARSTATUS>>  TSTStatus_pub_;

    std::string frame_id_;

  public:

    void initialize(rclcpp::Node& node)
    {
      static const size_t NUM_BITS = sizeof(uint32_t) * 8;

      assert(RECEIVER_ERROR_STRS.size() == NUM_BITS);
      assert(AUX1_STATUS_STRS.size()    == NUM_BITS);
      assert(AUX2_STATUS_STRS.size()    == NUM_BITS);
      assert(AUX3_STATUS_STRS.size()    == NUM_BITS);
      assert(AUX4_STATUS_STRS.size()    == NUM_BITS);


      RXSTATUS_pub_  = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::RXSTATUS>>(       "RXSTATUS",        node);
      TSTInfo_pub_   = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::TERRASTARINFO>>(  "TERRASTARINFO",   node);
      TSTStatus_pub_ = std::make_unique<Oem7RosPublisher<novatel_oem7_msgs::msg::TERRASTARSTATUS>>("TERRASTARSTATUS", node);
    }

    const MessageIdRecords& getMessageIds()
    {
      static const MessageIdRecords MSG_IDS(
                                      {
                                        {RXSTATUS_OEM7_MSGID,        MSGFLAG_STATUS_OR_CONFIG},
                                        {TERRASTARINFO_OEM7_MSGID,   MSGFLAG_STATUS_OR_CONFIG},
                                        {TERRASTARSTATUS_OEM7_MSGID, MSGFLAG_STATUS_OR_CONFIG},
                                      }
                                    );
      return MSG_IDS;
    }

    void publishRXSTATUS(Oem7RawMessageIf::ConstPtr msg)
    {
      std::shared_ptr<novatel_oem7_msgs::msg::RXSTATUS> rxstatus;
      MakeROSMessage(msg, rxstatus);

      // Populate status strings:
      get_status_info(rxstatus->error,     RECEIVER_ERROR_STRS,  rxstatus->error_strs,     rxstatus->error_bits);

      get_status_info(rxstatus->rxstat,    RECEIVER_STATUS_STRS, rxstatus->rxstat_strs,    rxstatus->rxstat_bits);
      get_status_info(rxstatus->aux1_stat, AUX1_STATUS_STRS,     rxstatus->aux1_stat_strs, rxstatus->aux1_stat_bits);
      get_status_info(rxstatus->aux2_stat, AUX2_STATUS_STRS,     rxstatus->aux2_stat_strs, rxstatus->aux2_stat_bits);
      get_status_info(rxstatus->aux3_stat, AUX3_STATUS_STRS,     rxstatus->aux3_stat_strs, rxstatus->aux3_stat_bits);
      get_status_info(rxstatus->aux4_stat, AUX4_STATUS_STRS,     rxstatus->aux4_stat_strs, rxstatus->aux4_stat_bits);

      RXSTATUS_pub_->publish(rxstatus);
    }

    void publishTERRASTARINFO(Oem7RawMessageIf::ConstPtr msg)
    {
        std::shared_ptr<novatel_oem7_msgs::msg::TERRASTARINFO> terrastarinfo;
        MakeROSMessage(msg, terrastarinfo);
        TSTInfo_pub_->publish(terrastarinfo);
    }

    void publishTERRASTARSTATUS(Oem7RawMessageIf::ConstPtr msg)
    {
        std::shared_ptr<novatel_oem7_msgs::msg::TERRASTARSTATUS> terrastarstatus;
        MakeROSMessage(msg, terrastarstatus);
        TSTStatus_pub_->publish(terrastarstatus);
    }

    void handleMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      if(msg->getMessageId() == RXSTATUS_OEM7_MSGID)
      {
        publishRXSTATUS(msg);
      }
      else if(msg->getMessageId() == TERRASTARINFO_OEM7_MSGID)
      {
        publishTERRASTARINFO(msg);
      }
      else if(msg->getMessageId() == TERRASTARSTATUS_OEM7_MSGID)
      {
        publishTERRASTARSTATUS(msg);
      }
    }
  };



}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::ReceiverStatusHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
