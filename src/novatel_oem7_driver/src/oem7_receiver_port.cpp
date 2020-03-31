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

#include <novatel_oem7_driver/oem7_receiver_if.hpp>
#include <oem7_receiver.hpp>

#include <ros/ros.h>

#include <boost/asio.hpp>


namespace novatel_oem7_driver
{
  /**
   * Serial port (tty) implementation.
   */
  class Oem7ReceiverPort: public Oem7Receiver<boost::asio::serial_port>
  {
    void endpoint_try_open()
    {
      if(endpoint_.is_open())
      {
        return;
      }

      std::string recvr_tty_name;
      nh_.getParam("oem7_tty_name", recvr_tty_name);

      int baud_rate = 0; // Optional parameter
      nh_.getParam("oem7_tty_baud", baud_rate);
      ROS_INFO_STREAM("Oem7SerialPort['" << recvr_tty_name << "' : " << baud_rate << "]");


      boost::system::error_code err;
      // We proceed regardless of the error code.
      // Successful connection does not guarantee subsequent operations will succeed.
      // Attempting operations on closed ports is harmless.

      endpoint_.close(err);
      endpoint_.open(recvr_tty_name, err);
      ROS_INFO_STREAM("Oem7SerialPort open: '" << endpoint_.is_open() << "; err: " << err);

      if(baud_rate > 0)
      {
        boost::asio::serial_port_base::baud_rate baud_option(baud_rate);
        endpoint_.set_option(baud_option, err);
        ROS_INFO_STREAM("Oem7SerialPort set_option baud_rate: '" << baud_option.value() << " : " << err);
      }
    }

    virtual size_t endpoint_read(boost::asio::mutable_buffer buf, boost::system::error_code& err)
    {
      boost::array<boost::asio::mutable_buffer, 1> bufs = {buf};
      return endpoint_.read_some(bufs, err);
    }

    virtual size_t endpoint_write(boost::asio::const_buffer buf, boost::system::error_code& err)
    {
      boost::array<boost::asio::const_buffer, 1> bufs = {buf};
      return endpoint_.write_some(bufs, err);
    }
};

}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ReceiverPort,     novatel_oem7_driver::Oem7ReceiverIf)

