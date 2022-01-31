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

#include <novatel_oem7_driver/oem7_imu.hpp>


#include <cmath> // PI

namespace novatel_oem7_driver
{

const double ONE_G = 9.80665;

///
/// @return arcseconds
///
inline double arcsecondsToRadians(double arcsecs)
{
  return arcsecs * M_PI / (180.0 * 3600.0);
}

///
/// @return degrees
///
inline double degreesToRadians(double degrees)
{
  return degrees * M_PI / 180.0;
}

///
/// @return meters
///
inline double feetToMeters(double feet)
{
  return feet / 3.2808;
}


bool
getImuRawScaleFactors(
    oem7_imu_type_t imu_type,
    imu_rate_t      imu_rate,
    double& gyro_scale,
    double& acc_scale)
{
  // Refer to RAWIMUSX in OEM7 documentation.

  switch(imu_type)
  {
    case IMU_TYPE_LN200:
      gyro_scale = pow(2.0, -19);
      acc_scale  = pow(2.0, -14);
      break;

    case IMU_TYPE_HG1900_CA29:
    case IMU_TYPE_HG1900_CA50:
    case IMU_TYPE_HG1930_CA50:
    case IMU_TYPE_HG1930_AA99:
    case IMU_TYPE_HG1700_AG11:
    case IMU_TYPE_HG1700_AG58:
      gyro_scale = pow(2.0, -33);
      acc_scale  = feetToMeters(pow(2.0, -27));
      break;

    case IMU_TYPE_HG1700_AG17:
    case IMU_TYPE_HG1700_AG62:
      gyro_scale = pow(2.0, -33);
      acc_scale  = feetToMeters(pow(2.0, -26));
      break;


    case IMU_TYPE_IMAR_FSAS:
      gyro_scale = arcsecondsToRadians(0.1 / pow(2, 8));
      acc_scale  = 0.05 / pow(2.0, 15);
      break;

    case IMU_TYPE_ISA100C:
    case IMU_TYPE_LITEF_MICROIMU:
      gyro_scale = 1.0E-9;
      acc_scale  = 2.0E-8;
      break;

    case IMU_TYPE_ADIS16488:
      gyro_scale = degreesToRadians(720.0/pow(2,31));
      acc_scale  = 200.0 / pow(2, 31);
      break;

    case IMU_TYPE_STIM300:
    case IMU_TYPE_STIM_300D:
      gyro_scale = degreesToRadians(pow(2,-21));
      acc_scale  = pow(2, -22);
      break;

    case IMU_TYPE_KVH_1750:
      gyro_scale = 0.1 / (3600.0 * 256.0);
      acc_scale  = 0.05 * pow(2.0, -15);
      break;

    case IMU_TYPE_EPSON_G320:
    case IMU_TYPE_EPSON_G320_200HZ:
      gyro_scale = degreesToRadians(0.008 / pow(2.0, 16)) / imu_rate;
      acc_scale  = ( (0.2 / pow(2.0, 16)) * (ONE_G/1000.0) ) / imu_rate;
      break;

    case IMU_TYPE_HG4930_AN01:
    case IMU_TYPE_HG4930_AN04:
    case IMU_TYPE_HG4930_AN04_400HZ:
      gyro_scale = pow(2.0, -33);
      acc_scale  = pow(2.0, -29);
      break;


    case IMU_TYPE_EPSON_G370:
      gyro_scale = degreesToRadians(0.0151515/ pow(2.0, 16)) / imu_rate;
      acc_scale  = ((0.4 / pow(2.0, 16)) * (ONE_G/1000.0)) / imu_rate;
      break;

    case IMU_TYPE_KVH_COTS:
      gyro_scale = 0.1 / (3600.0 * 256.0);
      acc_scale  = 0.05 / pow(2.0, 15);
      break;

    case IMU_TYPE_UNKNOWN: // Unsupported
    default:
      return false;
  };

  return true;

}


}
