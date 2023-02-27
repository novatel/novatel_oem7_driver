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


#include <ros/ros.h>

#include "oem7_debug_file.hpp"

namespace novatel_oem7_driver
{
    Oem7DebugFile::Oem7DebugFile()
    {
    }

    bool Oem7DebugFile::initialize(std::string& file_name)
    {
      file_name_ = file_name;

      if(file_name_.size() == 0)
      {
        return true; // Null initialization
      }

      oem7_file_.open(file_name_, std::ios::out | std::ios::binary | std::ios::trunc);
      int errno_value = errno; // Cache errno locally, in case any ROS calls /macros affect it.
      if(!oem7_file_)
      {
        ROS_ERROR_STREAM("Oem7DebugFile['" << file_name_ << "']: could not open; error= " << errno_value << " '"
                                            << strerror(errno_value) << "'");
        return false;
      }

      ROS_INFO_STREAM("Oem7DebugFile['" << file_name_ << "'] opened.");
          
      return true;
    }

    /**
     * Reads input from file.
     *
     */
    bool Oem7DebugFile::write(const unsigned char* buf, size_t len)
    {
      if(file_name_.size() == 0)
        return true;

      if(ros::isShuttingDown())
        return false;

      oem7_file_.write(reinterpret_cast<const char*>(buf), len);
      int errno_value = errno; // Cache errno locally, in case any ROS calls /macros affect it.
     
      if(!oem7_file_)
      {
        ROS_ERROR_STREAM("Oem7DebugFile[" << file_name_ << "]: errno= " << errno_value << " '" << strerror(errno_value) << "'");
        return false;
      }

      return true;
    }
}

