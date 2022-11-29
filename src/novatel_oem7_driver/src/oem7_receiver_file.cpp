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
#include <fstream>


namespace novatel_oem7_driver
{
  /**
   * 'Virtual' Oem7 interface, where input is read from a file. The contents of the file is any relevant receiver output.
   */
  class Oem7ReceiverFile: public Oem7ReceiverIf
  {
    rclcpp::Node* node_;

    std::ifstream oem7_file_; ///< Input

    size_t num_byte_read_; ///< Total number of bytes read from file.


  public:
    Oem7ReceiverFile():
      num_byte_read_(0)
    {
    }

    ~Oem7ReceiverFile()
    {
    }

    /**
     * Prepares the input file for reading. The file is opened as binary/read only.
     */
    virtual bool initialize(rclcpp::Node& nh)
    {
      node_ = &nh;

      node_->declare_parameter("oem7_file_name", "");
      std::string oem7_file_name = node_->get_parameter("oem7_file_name").as_string();

      RCLCPP_INFO_STREAM(node_->get_logger(), "Oem7File['" << oem7_file_name << "']");

      oem7_file_.open(oem7_file_name, std::ios::in | std::ios::binary);
      int errno_value = errno; // Cache errno locally, in case any ROS calls /macros affect it.
      if(!oem7_file_)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not open '" << oem7_file_name << "'; error= " << errno_value << " '"
                                            << strerror(errno_value) << "'");
        return false;
      }

      return true;
    }

    /**
     * Reads input from file.
     *
     */
    virtual bool read( boost::asio::mutable_buffer buf, size_t& rlen)
    {
      if(!oem7_file_)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error accessing file." );
        return false;
      }

      // Workaround for automated testing:
      // delay reporting logs so that 'rosbag record' has a chance to subscribe to the topic after they are published.
      // Otherwise it is likely to miss messages.
      if(num_byte_read_ == 0)
      {
        sleep(3); // Use absolute sleep, as this is not related to ROS internal timing.
      }

      oem7_file_.read(boost::asio::buffer_cast<char*>(buf), boost::asio::buffer_size(buf));
      int errno_value = errno; // Cache errno locally, in case any ROS calls /macros affect it.

      rlen = oem7_file_.gcount();
      num_byte_read_ += rlen;

      if(oem7_file_.eof())
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "No more input available. Read " << num_byte_read_ << " bytes." );

        return false;
      }

      if(!oem7_file_)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error " << errno_value << " reading input: '"  << strerror(errno_value) << "'" );

        return false;
      }

      return true;
    }

    /**
     * Takes no action.
     *
     * @return false always.
     */
    virtual bool write(boost::asio::const_buffer buf)
    {
      return false;
    }
  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::Oem7ReceiverFile, novatel_oem7_driver::Oem7ReceiverIf)
