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

#ifndef __OEM7_MESSAGE_IDS_HPP__
#define __OEM7_MESSAGE_IDS_HPP__

/*! \file
 * Oem7 Message ID definitions.
 * Refer to Oem7 manual.
 */

namespace novatel_oem7_driver
{
  const int BESTPOS_OEM7_MSGID            =   42;
  const int BESTUTM_OEM7_MSGID            =  726;
  const int BESTVEL_OEM7_MSGID            =   99;
  const int BESTGNSSPOS_OEM7_MSGID        = 1429;
  const int PPPPOS_OEM7_MSGID             = 1538;
  const int CORRIMUS_OEM7_MSGID           = 2264;
  const int HEADING2_OEM7_MSGID           = 1335;
  const int IMURATECORRIMUS_OEM7_MSGID    = 1362;
  const int INSCONFIG_OEM7_MSGID          = 1945;
  const int INSPVAS_OEM7_MSGID            =  508;
  const int INSPVAX_OEM7_MSGID            = 1465;
  const int INSSTDEV_OEM7_MSGID           = 2051;
  const int PSRDOP2_OEM7_MSGID            = 1163;
  const int RXSTATUS_OEM7_MSGID           =   93;
  const int TERRASTARINFO_OEM7_MSGID      = 1719;
  const int TERRASTARSTATUS_OEM7_MSGID    = 1729; 
  const int TIME_OEM7_MSGID               =  101;
  const int RAWIMUSX_OEM7_MSGID           =  1462;

  // NMEA0183
  const int GLMLA_OEM7_MSGID              =   859;
  const int GPALM_OEM7_MSGID              =   217;
  const int GPGGA_OEM7_MSGID              =   218;
  const int GPGGALONG_OEM7_MSGID          =   521;
  const int GPGLL_OEM7_MSGID              =   219;
  const int GPGRS_OEM7_MSGID              =   220;
  const int GPGSA_OEM7_MSGID              =   221;
  const int GPGST_OEM7_MSGID              =   222;
  const int GPGSV_OEM7_MSGID              =   223;
  const int GPHDT_OEM7_MSGID              =  1045;
  const int GPHDTDUALANTENNA_MSGID        =  2045;
  const int GPRMB_OEM7_MSGID              =   224;
  const int GPRMC_OEM7_MSGID              =   225;
  const int GPVTG_OEM7_MSGID              =   226;
  const int GPZDA_OEM7_MSGID              =   227;




}

#endif
