#ifndef __OEM7_RAW_MESSAGE_IF_
#define __OEM7_RAW_MESSAGE_IF_

#include <stdint.h>
#include <stddef.h>

#include <memory>

namespace novatel_oem7
{
  /** Interface to 'raw' Oem7 message; a message where only the generic Oem7 header is avialble
   * and the payload is opaque.
   *
   * Refer to Oem7 manual for explanation of the header fields.
   */
  class Oem7RawMessageIf
  {
    public:

      typedef std::shared_ptr<const Oem7RawMessageIf> ConstPtr;

      enum Oem7MessageType
      {
        OEM7MSGTYPE_UNKNOWN,
        OEM7MSGTYPE_LOG,
        OEM7MSGTYPE_RSP,
        OOEM7MSGTYPE_CMD
      };

      enum Oem7MessageFormat
      {
        OEM7MSGFMT_UNKNOWN,
        OEM7MSGFMT_BINARY,
        OEM7MSGFMT_SHORTBINARY,
        OEM7MSGFMT_ASCII,
        OEM7MSGFMT_ABASCII
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
