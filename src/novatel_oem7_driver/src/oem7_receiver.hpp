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


#include <boost/asio.hpp>

#include <rclcpp/rclcpp.hpp>


namespace novatel_oem7_driver
{
  /**
   * Common functionality related to boost::asio
   */
  template <typename T>
  class Oem7Receiver: public Oem7ReceiverIf
  {
    boost::asio::io_service io_;

    enum
    {
      DEFAULT_MAX_NUM_IO_ERRORS = 7
    };



  protected:

    rclcpp::Node* node_;

    T endpoint_; ///<  boost::asio communication endoint; socket, serial port, etc.

    int max_num_io_errors_; ///< Number of consecutive io errors before declaring failure and quitting.
    int num_io_errors_; ///< Number of consecuitive io errors.

    bool in_error_state()
    {
      if(num_io_errors_ >= max_num_io_errors_)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Oem7Receiver: Max Num IO errors exceeded: " << max_num_io_errors_);
        return true;
      }
      else
      {
        return false;
      }
    }


    /**
     * Attempt to opena the endpoint.
     */
    virtual void   endpoint_try_open() = 0;

    /**
     * Read some data from the endpoint.
     */
    virtual size_t endpoint_read( boost::asio::mutable_buffer buf, boost::system::error_code& err) = 0;


    /**
     * Write some data to the endpoint
     */
    virtual size_t endpoint_write(boost::asio::const_buffer buf,    boost::system::error_code& err) = 0;

    /**
     * Close the endpoint; delay to avoid tight re-open loop
     */
    void endpoint_close()
    {
      boost::system::error_code err;
      endpoint_.close(err);
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Oem7Receiver: close error: " <<  err.value());
      sleep(1.0);
    }

  public:
    Oem7Receiver():
      node_(NULL),
      io_(),
      endpoint_(io_),
      max_num_io_errors_(DEFAULT_MAX_NUM_IO_ERRORS),
      num_io_errors_(0)
    {
    }

    virtual ~Oem7Receiver()
    {
    }

    virtual bool initialize(rclcpp::Node& node)
    {
      node_ = &node;

      node_->declare_parameter("oem7_max_io_errors", 0);
      max_num_io_errors_ = node_->get_parameter("oem7_max_io_errors").as_int();

      return true;
    }

    virtual bool read( boost::asio::mutable_buffer buf, size_t& rlen)
    {
      while(rclcpp::ok() && !in_error_state())
      {
        endpoint_try_open();

        boost::system::error_code err;
        size_t len = endpoint_read(buf, err);
        if(err.value() == boost::system::errc::success)
        {
          num_io_errors_ = 0; // Reset error counter

          rlen = len;
          return true;
        }
        // else: error condition


        num_io_errors_++;

        RCLCPP_ERROR_STREAM(node_->get_logger(),
              "Oem7Receiver: read error: " <<  err.value()
                                           <<"; endpoint open: " << endpoint_.is_open()
                                           <<" errors/max: " << num_io_errors_
                                           <<"/"             << max_num_io_errors_);
        endpoint_close();
      }

      return false;
    }

    virtual bool write(boost::asio::const_buffer buf)
    {
      if(in_error_state() || !rclcpp::ok())
        return false;

      endpoint_try_open();

      boost::system::error_code err;
      endpoint_write(buf, err);
      if(err.value() != boost::system::errc::success)
      {
         num_io_errors_++;

         RCLCPP_ERROR_STREAM(node_->get_logger(),
                             "Oem7Receiver: write error: " << err.value() << "; endpoint open: " << endpoint_.is_open());
         endpoint_close();
         return false;
      }
      
      return true;
    }
};

}


