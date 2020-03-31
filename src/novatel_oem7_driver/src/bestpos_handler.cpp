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

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>
#include <oem7_driver_util.hpp>

#include <ros/ros.h>


#include <novatel_oem7_driver/oem7_ros_messages.hpp>
#include <oem7_ros_publisher.hpp>

#include "novatel_oem7_msgs/SolutionStatus.h"
#include "novatel_oem7_msgs/PositionOrVelocityType.h"
#include "novatel_oem7_msgs/BESTPOS.h"
#include "novatel_oem7_msgs/BESTUTM.h"
#include "novatel_oem7_msgs/BESTVEL.h"
#include "novatel_oem7_msgs/INSPVA.h"

#include "gps_common/GPSFix.h"
#include "sensor_msgs/NavSatFix.h"

#include <cmath>
#include <stdint.h>


namespace novatel_oem7_driver
{


  /***
   * Derive ROS GPS Status from Oem7 BESTPOS
   *
   * @return ROS status.
   */
  int16_t ToROSGPSStatus(const novatel_oem7_msgs::BESTPOS::Ptr bestpos)
  {
    if(bestpos->pos_type.type == novatel_oem7_msgs::PositionOrVelocityType::NONE)
      return gps_common::GPSStatus::STATUS_NO_FIX;

    return gps_common::GPSStatus::STATUS_FIX;
  }

  /***
   * Convert GPS time to seconds
   *
   * @return seconds
   */
  double MakeGpsTime_Seconds(uint16_t gps_week, uint32_t gps_milliseconds)
  {
    static const double SECONDS_IN_GPS_WEEK = 604800.0;
    static const double MILLISECONDS_IN_SECOND = 1000.0;

    return gps_week * SECONDS_IN_GPS_WEEK +
           gps_milliseconds / MILLISECONDS_IN_SECOND;
  }

  /***
   * Converts covariance form GPSFix to NavSatFix
   * @return NavSatFix covariance
   */
  uint8_t GpsFixCovTypeToNavSatFixCovType(uint8_t covariance_type)
  {
    switch(covariance_type)
    {
      case gps_common::GPSFix::COVARIANCE_TYPE_APPROXIMATED:
        return sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

      case gps_common::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN:
        return sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

      case gps_common::GPSFix::COVARIANCE_TYPE_KNOWN:
        return sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;

      case gps_common::GPSFix::COVARIANCE_TYPE_UNKNOWN:
        return sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

      default:
        ROS_ERROR_STREAM("Unknown GPSFix covariance type: " << covariance_type);
        return sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
  }


  /*** Generates NavSatFix object from GpsFix
   */
  void GpsFixToNavSatFix(const gps_common::GPSFix::Ptr gpsfix, sensor_msgs::NavSatFix::Ptr navsatfix)
  {
    navsatfix->latitude    = gpsfix->latitude;
    navsatfix->longitude   = gpsfix->longitude;
    navsatfix->altitude    = gpsfix->altitude;

    navsatfix->position_covariance[0]   = gpsfix->position_covariance[0];
    navsatfix->position_covariance[4]   = gpsfix->position_covariance[4];
    navsatfix->position_covariance[8]   = gpsfix->position_covariance[8];

    navsatfix->position_covariance_type = GpsFixCovTypeToNavSatFixCovType(gpsfix->position_covariance_type);
  }

  /***
   * Handler of position-related messages. Synthesizes ROS messagse GPSFix and NavSatFix from native Oem7 Messages.
   */
  class BESTPOSHandler: public Oem7MessageHandlerIf
  {
    Oem7RosPublisher BESTPOS_pub_;
    Oem7RosPublisher BESTUTM_pub_;
    Oem7RosPublisher BESTVEL_pub_;
    Oem7RosPublisher INSPVA_pub_;

    Oem7RosPublisher GPSFix_pub_;
    Oem7RosPublisher NavSatFix_pub_;

    boost::shared_ptr<novatel_oem7_msgs::BESTPOS> bestpos_;
    boost::shared_ptr<novatel_oem7_msgs::BESTVEL> bestvel_;
    boost::shared_ptr<novatel_oem7_msgs::INSPVA>  inspva_;

    boost::shared_ptr<gps_common::GPSFix> gpsfix_;

    Oem7RawMessageIf::ConstPtr psrdop2_;

    int64_t last_bestpos_;
    int64_t last_bestvel_;
    int64_t last_inspva_;

    int32_t bestpos_period_;
    int32_t bestvel_period_;
    int32_t inspva_period_;


    /***
     * @return true if the specified period is the shortest in all messages.
     */
    bool isShortestPeriod(int32_t period)
    {
      return period <= bestpos_period_ &&
             period <= bestvel_period_ &&
             period <= inspva_period_;
    }

    /***
     * Updates message period
     */
    template<typename T>
    void updatePeriod(
        const T& msg,
        int64_t& last_msg_msec,
        int32_t& msg_period)
    {
      int64_t cur_msg_msec = GPSTimeToMsec(msg->nov_header);
      if(last_msg_msec > 0)
      {
        int32_t period = cur_msg_msec - last_msg_msec;
        if(period >= 0)
        {
          msg_period = period;
        }
        else // Could be input corruption; do not update anything.
        {
          ROS_ERROR_STREAM("updatePeriod: msg= " << msg->nov_header.message_id << "; per= " << period << "; ignored.");
        }
      }

      last_msg_msec = cur_msg_msec;
    }

    void publishBESTPOS(Oem7RawMessageIf::ConstPtr msg)
    {
      MakeROSMessage(msg, bestpos_);
      updatePeriod(bestpos_, last_bestpos_, bestpos_period_);


      BESTPOS_pub_.publish(bestpos_);
    }

    void publishBESTVEL(Oem7RawMessageIf::ConstPtr msg)
    {
      MakeROSMessage(msg, bestvel_);
      updatePeriod(bestvel_, last_bestvel_, bestvel_period_);
      BESTVEL_pub_.publish(bestvel_);
    }

    void publishBESTUTM(Oem7RawMessageIf::ConstPtr msg)
    {
        boost::shared_ptr<novatel_oem7_msgs::BESTUTM> bestutm;
        MakeROSMessage(msg, bestutm);
        BESTUTM_pub_.publish(bestutm);
    }

    void publishINSVPA(Oem7RawMessageIf::ConstPtr msg)
    {
      MakeROSMessage(msg, inspva_);
      updatePeriod(inspva_, last_inspva_, inspva_period_);

      INSPVA_pub_.publish(inspva_);
    }

    void publishGPSFix()
    {
      gpsfix_.reset(new gps_common::GPSFix);

      gpsfix_->status.position_source     = gps_common::GPSStatus::SOURCE_NONE;
      gpsfix_->status.orientation_source  = gps_common::GPSStatus::SOURCE_NONE;
      gpsfix_->status.motion_source       = gps_common::GPSStatus::SOURCE_NONE;

      // When INS is available, BESTPOS is the best of GNSS + INS, so it is always preferred.
      if(bestpos_)
      {
        gpsfix_->latitude              = bestpos_->lat;
        gpsfix_->longitude             = bestpos_->lon;
        gpsfix_->altitude              = bestpos_->hgt;

        // Convert stdev to diagonal covariance
        gpsfix_->position_covariance[0] = std::pow(bestpos_->lon_stdev, 2);
        gpsfix_->position_covariance[4] = std::pow(bestpos_->lat_stdev, 2);
        gpsfix_->position_covariance[8] = std::pow(bestpos_->hgt_stdev, 2);
        gpsfix_->position_covariance_type = gps_common::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        gpsfix_->time = MakeGpsTime_Seconds(
                          bestpos_->nov_header.gps_week_number,
                          bestpos_->nov_header.gps_week_milliseconds);


        gpsfix_->status.satellites_used = bestpos_->num_sol_svs;
        gpsfix_->status.status = ToROSGPSStatus(bestpos_);

        gpsfix_->status.position_source = gps_common::GPSStatus::SOURCE_GPS;
      }

      if(bestvel_)
      {
        gpsfix_->track = bestvel_->trk_gnd;
        gpsfix_->speed = bestvel_->hor_speed;
        gpsfix_->climb = bestvel_->ver_speed;

        if(gpsfix_->time == 0.0) // Not populated yet
        {
          gpsfix_->time = MakeGpsTime_Seconds(
                            bestvel_->nov_header.gps_week_number,
                            bestvel_->nov_header.gps_week_milliseconds);
        }

        if(bestvel_->vel_type.type == novatel_oem7_msgs::PositionOrVelocityType::DOPPLER_VELOCITY)
        {
          gpsfix_->status.motion_source != gps_common::GPSStatus::SOURCE_DOPPLER;
        }
        else
        {
          gpsfix_->status.motion_source |= gps_common::GPSStatus::SOURCE_POINTS;
        }
      }


      if(inspva_ )
      {
        if(!bestpos_) // BESTPOS is not available
        {
          gpsfix_->latitude              = inspva_->latitude;
          gpsfix_->longitude             = inspva_->longitude;
          gpsfix_->altitude              = inspva_->height;

          // TODO: compute variance form INSSTDEV instead?
          gpsfix_->position_covariance_type = gps_common::GPSFix::COVARIANCE_TYPE_UNKNOWN;
        }

        gpsfix_->pitch                 = inspva_->pitch;
        gpsfix_->roll                  = inspva_->roll;
        //gpsfix->dip: not populated.

        gpsfix_->status.position_source   |= (gps_common::GPSStatus::SOURCE_GYRO | gps_common::GPSStatus::SOURCE_ACCEL);
        gpsfix_->status.orientation_source = (gps_common::GPSStatus::SOURCE_GYRO | gps_common::GPSStatus::SOURCE_ACCEL);
        gpsfix_->status.motion_source      = (gps_common::GPSStatus::SOURCE_GYRO | gps_common::GPSStatus::SOURCE_ACCEL);

        if(gpsfix_->time == 0.0) // Not populated yet
        {
          gpsfix_->time = MakeGpsTime_Seconds(
                            inspva_->nov_header.gps_week_number,
                            inspva_->nov_header.gps_week_milliseconds);
        }
      }

      if(psrdop2_)
      {
        GetDOPFromPSRDOP2(
            psrdop2_,
            0, // GPS
            gpsfix_->gdop,
            gpsfix_->pdop,
            gpsfix_->hdop,
            gpsfix_->vdop,
            gpsfix_->tdop);
      }

      GPSFix_pub_.publish(gpsfix_);
    }

    void publishNavSatFix()
    {
      if(!gpsfix_)
      {
        return;
      }

      boost::shared_ptr<sensor_msgs::NavSatFix> navsatfix(new sensor_msgs::NavSatFix);

      // Derive from GPSFix.
      GpsFixToNavSatFix(gpsfix_, navsatfix);

      NavSatFix_pub_.publish(navsatfix);
    }

    void publishROSMessages()
    {
      publishGPSFix(); // Must be published first, since other message may be derived from it.
      publishNavSatFix();
    }



  public:
    BESTPOSHandler():
      last_bestpos_(0),
      last_bestvel_(0),
      last_inspva_(0),
      bestpos_period_(INT_MAX),
      bestvel_period_(INT_MAX),
      inspva_period_( INT_MAX)
    {
    }

    ~BESTPOSHandler()
    {
    }

    void initialize(ros::NodeHandle& nh)
    {
      BESTPOS_pub_.setup<novatel_oem7_msgs::BESTPOS>("BESTPOS",   nh);
      BESTVEL_pub_.setup<novatel_oem7_msgs::BESTVEL>("BESTVEL",   nh);
      BESTUTM_pub_.setup<novatel_oem7_msgs::BESTUTM>("BESTUTM",   nh);
      INSPVA_pub_.setup<novatel_oem7_msgs::INSPVA>(  "INSPVA",    nh);
      GPSFix_pub_.setup<gps_common::GPSFix>(         "GPSFix",    nh);
      NavSatFix_pub_.setup<sensor_msgs::NavSatFix>(  "NavSatFix", nh);
    }

    const std::vector<int>& getMessageIds()
    {
      static const std::vector<int> MSG_IDS({BESTPOS_OEM7_MSGID, BESTVEL_OEM7_MSGID, BESTUTM_OEM7_MSGID, INSPVAS_OEM7_MSGID, PSRDOP2_OEM7_MSGID});
      return MSG_IDS;
    }

    void handleMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      ROS_DEBUG_STREAM("BESTPOS < [id=" << msg->getMessageId() << "] periods (BP BV PVA):" <<
                        bestpos_period_ << " " <<
                        bestvel_period_ << " " <<
                        inspva_period_);

      // It is assumed all the messages are logged at reasonable rates.
      // BESTPOS and BESTVEL are always logged together.
      // On units with IMU, INSPVA would trigger publishing of ROS messages.
      // On non-IMU units, BESTVEL be.

      if(msg->getMessageId() == BESTPOS_OEM7_MSGID)
      {
        publishBESTPOS(msg);

        if(isShortestPeriod(bestpos_period_))
        {
          publishROSMessages();
        }
      }

      if(msg->getMessageId() == BESTVEL_OEM7_MSGID)
      {
        publishBESTVEL(msg);

        if(isShortestPeriod(bestvel_period_))
        {
          publishROSMessages();
        }
      }

      if(msg->getMessageId() == BESTUTM_OEM7_MSGID)
      {
        publishBESTUTM(msg);
      }

      if(msg->getMessageId() == INSPVAS_OEM7_MSGID)
      {
        publishINSVPA(msg);

        if(isShortestPeriod(inspva_period_))
        {
          publishROSMessages();
        }
      }

      if(msg->getMessageId() == PSRDOP2_OEM7_MSGID)
      {
        psrdop2_ = msg;
      }
    }
  };

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::BESTPOSHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
