#ifndef __OEM7_MESSAGE_DECODER__
#define __OEM7_MESSAGE_DECODER__

#include <oem7_raw_message_if.hpp>

#include <boost/asio/buffer.hpp>



namespace novatel_oem7
{
  /** Software component version elements
   */
  typedef short version_element_t;

  /**
   * Interface into Oem7 message decoder
   */
  class Oem7MessageDecoderLibIf
  {
  public:
      virtual ~Oem7MessageDecoderLibIf(){}

      /**
       * Obtain the next message
       * This is a blocking call
       *
       * @return false when no more message can be read, or an error has occured.
       */
      virtual bool readMessage(std::shared_ptr<Oem7RawMessageIf>&) = 0;

  };

  /**
   * Interface implemented by Oem7 message decoder users
   */
  class Oem7MessageDecoderLibUserIf
  {
  public:
    virtual ~Oem7MessageDecoderLibUserIf(){}

    /**
     * Called by Oem7 message decoder implementation to obtain a sequence of bytes, to be parsed into Oem7 message.
     */
    virtual bool read(
        boost::asio::mutable_buffer, ///< Buffer to be populated
        size_t& ///< [out]: number of valid bytes in buffer
        ) = 0;
  };

  /***
   * Obtains an instance of Oem7 message decoder
   *
   * @return Decoder
   */
  std::shared_ptr<Oem7MessageDecoderLibIf>
  GetOem7MessageDecoder(
      Oem7MessageDecoderLibUserIf* ///< decoder user
      );

  /**
   * Obtains Oem7 decoder library version
   */
  void
  GetOem7MessageDecoderLibVersion(version_element_t& major, version_element_t& minor, version_element_t& build);
}




#endif
