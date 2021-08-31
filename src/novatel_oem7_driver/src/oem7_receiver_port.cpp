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
#include <driver_parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/asio.hpp>
#include <boost/array.hpp>

namespace novatel_oem7_driver
{
  /**
   * Serial port (tty) implementation.
   */
  class Oem7ReceiverPort: public Oem7Receiver<boost::asio::serial_port>
  {
    using Oem7Receiver<boost::asio::serial_port>::node_;

    void endpoint_try_open()
    {
      if(endpoint_.is_open())
      {
        return;
      }

      static DriverParameter<std::string> port_name("oem7_port_name",  "", *node_);
      static DriverParameter<int>         port_baud("oem7_port_baud",  0,  *node_);
      RCLCPP_INFO_STREAM(node_->get_logger(), "Oem7SerialPort['" << port_name.value() << "' : " << port_baud.value() << "]");


      boost::system::error_code err;
      // We proceed regardless of the error code.
      // Successful connection does not guarantee subsequent operations will succeed.
      // Attempting operations on closed ports is harmless.

      endpoint_.close(err);
      endpoint_.open(port_name.value(), err);
      RCLCPP_INFO_STREAM(node_->get_logger(),"Oem7SerialPort open: '" << endpoint_.is_open() << "; err: " << err);

      if(port_baud.value() > 0)
      {
        boost::asio::serial_port_base::baud_rate baud_option(port_baud.value());
        endpoint_.set_option(baud_option, err);
        RCLCPP_INFO_STREAM(node_->get_logger(), "Oem7SerialPort set_option baud_rate: '" << baud_option.value() << " : " << err);
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


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ReceiverPort,     novatel_oem7_driver::Oem7ReceiverIf)

