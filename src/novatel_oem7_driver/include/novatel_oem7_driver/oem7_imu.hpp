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

namespace novatel_oem7_driver
{

/// IMUs supported on OEM7 products; refer to INSCONFIG in the OEM7 manual.
enum oem7_imu_type_t
{
  IMU_TYPE_UNKNOWN           =  0,
  IMU_TYPE_HG1700_AG11       =  1,
  IMU_TYPE_HG1700_AG17       =  4,
  IMU_TYPE_HG1900_CA29       =  5,
  IMU_TYPE_LN200             =  8,
  IMU_TYPE_HG1700_AG58       = 11,
  IMU_TYPE_HG1700_AG62       = 12,
  IMU_TYPE_IMAR_FSAS         = 13,
  IMU_TYPE_KVH_COTS          = 16,
  IMU_TYPE_HG1930_AA99       = 20,
  IMU_TYPE_ISA100C           = 26,
  IMU_TYPE_HG1900_CA50       = 27,
  IMU_TYPE_HG1930_CA50       = 28,
  IMU_TYPE_ADIS16488         = 31,
  IMU_TYPE_STIM300           = 32,
  IMU_TYPE_KVH_1750          = 33,
  IMU_TYPE_EPSON_G320        = 41,
  IMU_TYPE_LITEF_MICROIMU    = 52,
  IMU_TYPE_STIM_300D         = 56,
  IMU_TYPE_HG4930_AN01       = 58,
  IMU_TYPE_EPSON_G370        = 61,
  IMU_TYPE_EPSON_G320_200HZ  = 62,
  IMU_TYPE_HG4930_AN04       = 68,
  IMU_TYPE_HG4930_AN04_400HZ = 69

};


typedef int imu_rate_t; ///< IMU message output rate. Refer to INSCONFIG in the OEM7 manual.s

///
/// Obtain scaling factors for raw IMU output (as reported by RAWIMUS etc).
/// Refer to RAWIMUSX documentation.
///
/// @return false if the IMU is not supported. This means that the code was not updated
///               to reflect new OEM7 product release.
///
bool
getImuRawScaleFactors(
    oem7_imu_type_t imu_type, ///< IMU type.
    imu_rate_t      imu_rate, ///< IMU rate; needed because some IMUs report instantaneous rate.
    double& gyro_scale,       ///< Gyroscope scale factor
    double& acc_scale         ///< Accelerometer scale factor
    );

}
