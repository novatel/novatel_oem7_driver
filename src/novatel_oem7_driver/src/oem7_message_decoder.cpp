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

#include <novatel_oem7_driver/oem7_message_decoder_if.hpp>
#include <novatel_oem7_driver/oem7_receiver_if.hpp>


#include "oem7_message_decoder_lib.hpp"

#include "oem7_debug_file.hpp"

#include <driver_parameter.hpp>


namespace novatel_oem7_driver
{

  /***
   * Parser for Oem7 messages. Obtains serial input from Oem7RawMessageParserIf; makes callbacks on Oem7RawMessageParserUserIf when Oem7 messages.
   */
  class Oem7MessageDecoder: public Oem7MessageDecoderIf, public novatel_oem7::Oem7MessageDecoderLibUserIf
  {
    rclcpp::Node* node_; // ROS Node Handle.

    std::unique_ptr<Oem7DebugFile> decoder_dbg_file_;
    std::unique_ptr<Oem7DebugFile> receiver_dbg_file_;
 

    Oem7MessageDecoderUserIf* user_; //< Parser user callback interface

    Oem7ReceiverIf* recvr_;

    std::shared_ptr<novatel_oem7::Oem7MessageDecoderLibIf> decoder_; //< NovAtel message decoder


  public:

    /**
     * Initializes the decoder
     */
    bool initialize(
        rclcpp::Node& node,
        Oem7ReceiverIf* recvr,
        Oem7MessageDecoderUserIf* user)
    {
      node_    = &node;
      user_  = user;
      recvr_ = recvr;

      novatel_oem7::version_element_t major, minor, special;
      novatel_oem7::GetOem7MessageDecoderLibVersion(major, minor, special);

      RCLCPP_INFO_STREAM(node_->get_logger(), "Oem7MessageDecoderLib version: " << major << "." << minor << "." << special);

      decoder_ = novatel_oem7::GetOem7MessageDecoder(this);

      DriverParameter<std::string> rcvr_log_file("oem7_receiver_log_file", "", node);
      DriverParameter<std::string> dcdr_log_file("oem7_decoder_log_file",  "", node);
      
      decoder_dbg_file_  = std::make_unique<Oem7DebugFile>(dcdr_log_file.value(), node_->get_logger());
      receiver_dbg_file_ = std::make_unique<Oem7DebugFile>(rcvr_log_file.value(), node_->get_logger());

      return true;
    }

    virtual bool read( boost::asio::mutable_buffer buf, size_t& s)
    {
      bool ok = recvr_->read(buf, s);
      if(ok)
      {
        receiver_dbg_file_->write(boost::asio::buffer_cast<unsigned char*>(buf), s);
      }

      return ok;
    }


    /*
     * Parser service loop.
     * Drive the parser forward to keep reading from its input stream and making message callbacks into user.
     * Blocks until:
     * The system is shut down / ros::ok() returns false
     * No more input available (as a permanent condition),
     */
    void service()
    {

    //  try
    //  {
        while(rclcpp::ok())
        {
          std::shared_ptr<novatel_oem7::Oem7RawMessageIf> msg;
          if(decoder_->readMessage(msg))
          {
            if(msg)
            {
              decoder_dbg_file_->write(msg->getMessageData(0), msg->getMessageDataLength());

              user_->onNewMessage(msg);
            }
            // else: No messages available now; keep retrying until we get one or decoder gives up.
          }
          else
          {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Decoder: no more messages available.");
            break;
          }
        }

     // }
     // catch(std::exception const& ex)
     // {
     //   RCLCPP_ERROR_STREAM(node_->get_logger(), "Decoder exception: " << ex.what());
     // }
    }
  };

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7MessageDecoder, novatel_oem7_driver::Oem7MessageDecoderIf)
