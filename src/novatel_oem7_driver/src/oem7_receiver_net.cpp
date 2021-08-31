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


#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <driver_parameter.hpp>

namespace novatel_oem7_driver
{


  template <class T>
  class Oem7ReceiverNet: public Oem7Receiver<typename T::socket>
  {
    using Oem7Receiver<typename T::socket>::node_;

    void endpoint_try_open()
    {
      if(this->endpoint_.is_open())
      {
        return;
      }


      static DriverParameter<std::string> recvr_ip_addr("oem7_ip_addr", "", *node_);
      static DriverParameter<int>         recvr_port(   "oem7_port",    0,  *node_);

      RCLCPP_INFO_STREAM(node_->get_logger(),
                    "Oem7Net " << (T::v4().protocol() == IPPROTO_TCP ? "TCP" : "UDP") <<
                      "['" << recvr_ip_addr.value() << "' : " << recvr_port.value() << "]");

      boost::system::error_code err;

      this->endpoint_.close(err); // Doesn't matter if we fail.
      this->endpoint_.connect(typename T::endpoint(boost::asio::ip::address::from_string(recvr_ip_addr.value()), recvr_port.value()), err);
      // Proceed regardless; successful connection does not guarantee subsequent operations will succeed.

      RCLCPP_INFO_STREAM(node_->get_logger(),
                         "Oem7Net socket open: '" << this->endpoint_.is_open() << "; OS error= " << err.value());

      static const std::string CONN_PRIMER("\r\n");
      endpoint_write(boost::asio::buffer(CONN_PRIMER), err);
    }

    virtual size_t endpoint_read(boost::asio::mutable_buffer buf, boost::system::error_code& err)
    {
      boost::array<boost::asio::mutable_buffer, 1> bufs = {buf};
      return this->endpoint_.receive(bufs, 0, err);
    }

    virtual size_t endpoint_write(boost::asio::const_buffer buf, boost::system::error_code& err)
    {
      const boost::array<boost::asio::const_buffer, 1> bufs = {buf};
      return this->endpoint_.send(bufs, 0, err);
    }
  };

  class Oem7ReceiverTcp: public Oem7ReceiverNet<boost::asio::ip::tcp>{};
  class Oem7ReceiverUdp: public Oem7ReceiverNet<boost::asio::ip::udp>{};
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ReceiverTcp,     novatel_oem7_driver::Oem7ReceiverIf)
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ReceiverUdp,     novatel_oem7_driver::Oem7ReceiverIf)
