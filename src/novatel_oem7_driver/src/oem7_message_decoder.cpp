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

#include <ros/console.h>


#include <boost/scoped_ptr.hpp>
#include "oem7_message_decoder_lib.hpp"

#include "oem7_debug_file.hpp"



namespace novatel_oem7_driver
{

  /***
   * Parser for Oem7 messages. Obtains serial input from Oem7RawMessageParserIf; makes callbacks on Oem7RawMessageParserUserIf when Oem7 messages.
   */
  class Oem7MessageDecoder: public Oem7MessageDecoderIf, public novatel_oem7::Oem7MessageDecoderLibUserIf
  {
    ros::NodeHandle nh_; // ROS Node Handle.

    Oem7DebugFile decoder_dbg_file_;
    Oem7DebugFile receiver_dbg_file_;
 

    Oem7MessageDecoderUserIf* user_; //< Parser user callback interface

    Oem7ReceiverIf* recvr_;

    boost::shared_ptr<novatel_oem7::Oem7MessageDecoderLibIf> decoder_; //< NovAtel message decoder


  public:

    /**
     * Initializes the decoder
     */
    bool initialize(
        ros::NodeHandle& nh,
        Oem7ReceiverIf* recvr,
        Oem7MessageDecoderUserIf* user)
    {
      nh_    = nh;
      user_  = user;
      recvr_ = recvr;

      novatel_oem7::version_element_t major, minor, build;
      novatel_oem7::GetOem7MessageDecoderLibVersion(major, minor, build);

      ROS_INFO_STREAM("Oem7MessageDecoderLib version: " << major << "." << minor << "." << build);

      decoder_ = novatel_oem7::GetOem7MessageDecoder(this);

      std::string decoder_dbg_file_name;
      std::string receiver_dbg_file_name;
      nh_.getParam("oem7_receiver_log_file", receiver_dbg_file_name);
      nh_.getParam("oem7_decoder_log_file",  decoder_dbg_file_name);
      
      decoder_dbg_file_.initialize( decoder_dbg_file_name);
      receiver_dbg_file_.initialize(receiver_dbg_file_name);
 

      return true;
    }

    virtual bool read( boost::asio::mutable_buffer buf, size_t& s)
    {
      bool ok = recvr_->read(buf, s);
      if(ok)
      {
        receiver_dbg_file_.write(boost::asio::buffer_cast<unsigned char*>(buf), s);
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
      try
      {
        while(!ros::isShuttingDown())
        {
          boost::shared_ptr<novatel_oem7::Oem7RawMessageIf> msg;
          if(decoder_->readMessage(msg))
          {
            if(msg)
            {
              decoder_dbg_file_.write(msg->getMessageData(0), msg->getMessageDataLength());

              user_->onNewMessage(msg);
            }
            // else: No messages available now; keep retrying until we get one or decoder gives up.
          }
          else
          {
            ROS_WARN("Decoder: no more messages available.");
            break;
          }
        }
      }
      catch(std::exception const& ex)
      {
        ROS_ERROR_STREAM("Decoder exception: " << ex.what());
      }
    }
  };

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7MessageDecoder, novatel_oem7_driver::Oem7MessageDecoderIf)
