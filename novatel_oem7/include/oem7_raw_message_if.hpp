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

#ifndef __OEM7_RAW_MESSAGE_IF_
#define __OEM7_RAW_MESSAGE_IF_

#include <stdint.h>
#include <stddef.h>

#include <boost/shared_ptr.hpp>

namespace novatel_oem7
{
  class Oem7RawMessageIf
  {
    public:

      typedef boost::shared_ptr<const Oem7RawMessageIf> ConstPtr;

      enum Oem7MessageType
      {
        OEM7MSGTYPE_LOG,
        OEM7MSGTYPE_RSP,
        OOEM7MSGTYPE_CMD,
        OEM7MSGTYPE_UNKNOWN = 1000
      };

      enum Oem7MessageFormat
      {
        OEM7MSGFMT_BINARY,
        OEM7MSGFMT_ASCII,
        OEM7MSGFMT_ABASCII,
        OEM7MSGFMT_UNKNOWN = 1000
      };

      virtual ~Oem7RawMessageIf(){}

      virtual Oem7MessageType            getMessageType()              const = 0;
      virtual Oem7MessageFormat          getMessageFormat()            const = 0;
      virtual int                        getMessageId()                const = 0;
      virtual const uint8_t*             getMessageData(size_t offset) const = 0;
      virtual size_t                     getMessageDataLength()        const = 0;
  };
}

#endif
