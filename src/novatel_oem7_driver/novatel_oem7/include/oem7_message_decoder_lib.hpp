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

#ifndef __OEM7_MESSAGE_DECODER__
#define __OEM7_MESSAGE_DECODER__

#include <oem7_raw_message_if.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/asio/buffer.hpp>



namespace novatel_oem7
{
  typedef short version_element_t;

  class Oem7MessageDecoderLibIf
  {

  public:
      virtual ~Oem7MessageDecoderLibIf(){}
      virtual bool readMessage(boost::shared_ptr<Oem7RawMessageIf>&) = 0;

  };

  class Oem7MessageDecoderLibUserIf
  {
  public:
    virtual ~Oem7MessageDecoderLibUserIf(){}
    virtual bool read( boost::asio::mutable_buffer, size_t&) = 0;
  };

  
  boost::shared_ptr<Oem7MessageDecoderLibIf>
  GetOem7MessageDecoder(Oem7MessageDecoderLibUserIf*);

  void
  GetOem7MessageDecoderLibVersion(version_element_t& major, version_element_t& minor, version_element_t& build);
}




#endif
