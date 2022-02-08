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
#include "novatel_oem7_msgs/BESTGNSSPOS.h"
#include "novatel_oem7_msgs/BESTPOS.h"
#include "novatel_oem7_msgs/BESTUTM.h"
#include "novatel_oem7_msgs/BESTVEL.h"
#include "novatel_oem7_msgs/PPPPOS.h"
#include "novatel_oem7_msgs/AccessStatus.h"
#include "novatel_oem7_msgs/GeogatingStatus.h"
#include "novatel_oem7_msgs/LocalAreaStatus.h"
#include "novatel_oem7_msgs/RegionRestriction.h"
#include "novatel_oem7_msgs/SubscriptionPermission.h"
#include "novatel_oem7_msgs/SubscriptionType.h"
#include "novatel_oem7_msgs/SyncState.h"
#include "novatel_oem7_msgs/TERRASTARINFO.h"
#include "novatel_oem7_msgs/TERRASTARSTATUS.h"
#include "novatel_oem7_msgs/INSPVA.h"
#include "novatel_oem7_msgs/INSPVAX.h"

#include "nav_msgs/Odometry.h"
#include "gps_common/GPSFix.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gps_common/conversions.h>

#include <cmath>
#include <stdint.h>


namespace novatel_oem7_driver
{
  /***
   * Converts radians to degrees
   *
   * @return degrees
   */
  inline double radiansToDegrees(double radians)
  {
    return radians * 180.0 / M_PI;
  }

  /***
   * Converts degrees to Radians
   *
   * @return radians
   */
  inline double degreesToRadians(double degrees)
  {
    return degrees * M_PI / 180.0;
  }


  /**
   * Compute a single 3D standard deviation from individual deviations.
   * This can be used as a '3D error'.
   */
  double Get3DPositionError(double lat_stdev, double lon_stdev, double hgt_stdev)
  {
    // Sum stdevs:take square root of sum of variances
    return std::sqrt(
        std::pow(lat_stdev, 2) +
        std::pow(lon_stdev, 2) +
        std::pow(hgt_stdev, 2)
    );
  }


  /*
   * Refer to NovAtel APN029
   */
  double computeHorizontalError(double lat_stdev, double lon_stdev)
  {
    //95%:  2 * DRMS:

    return 2.0 * std::sqrt(
        std::pow(lat_stdev, 2) +
        std::pow(lon_stdev, 2));
  }

  /*
   * Refer to NovAtel APN029
   */
  double computeVerticalError(double hgt_stdev)
  {
    //95%
    return 2.0 * hgt_stdev;
  }

  /*
   * Refer to NovAtel APN029
   */
  double computeSphericalError(double lat_stdev, double lon_stdev, double hgt_stdev)
  {
    // 90% spherical accuracy
    return 0.833 * (lat_stdev + lon_stdev + hgt_stdev);
  };
  /***
   * Derive ROS GPS Status from Oem7 BESTPOS
   *
   * @return ROS status.
   */
  int16_t ToROSGPSStatus(const novatel_oem7_msgs::BESTPOS::Ptr bestpos)
  {
    // ROS does not support all necessary solution types to map Oem7 solution types correctly.
    // For consistency, OEM7 WAAS is reported as SBAS.


    switch(bestpos->pos_type.type)
    {
      case novatel_oem7_msgs::PositionOrVelocityType::PSRDIFF:
      case novatel_oem7_msgs::PositionOrVelocityType::INS_PSRDIFF:
      case novatel_oem7_msgs::PositionOrVelocityType::L1_FLOAT:
      case novatel_oem7_msgs::PositionOrVelocityType::NARROW_FLOAT:
      case novatel_oem7_msgs::PositionOrVelocityType::L1_INT:
      case novatel_oem7_msgs::PositionOrVelocityType::WIDE_INT:
      case novatel_oem7_msgs::PositionOrVelocityType::NARROW_INT:
      case novatel_oem7_msgs::PositionOrVelocityType::RTK_DIRECT_INS:
      case novatel_oem7_msgs::PositionOrVelocityType::INS_RTKFLOAT:
      case novatel_oem7_msgs::PositionOrVelocityType::INS_RTKFIXED:
        return gps_common::GPSStatus::STATUS_DGPS_FIX;

      case novatel_oem7_msgs::PositionOrVelocityType::FIXEDPOS:
      case novatel_oem7_msgs::PositionOrVelocityType::FIXEDHEIGHT:
      case novatel_oem7_msgs::PositionOrVelocityType::DOPPLER_VELOCITY:
      case novatel_oem7_msgs::PositionOrVelocityType::SINGLE:
      case novatel_oem7_msgs::PositionOrVelocityType::INS_PSRSP:
      case novatel_oem7_msgs::PositionOrVelocityType::PROPAGATED:
      case novatel_oem7_msgs::PositionOrVelocityType::OPERATIONAL:
      case novatel_oem7_msgs::PositionOrVelocityType::WARNING:
      case novatel_oem7_msgs::PositionOrVelocityType::OUT_OF_BOUNDS:
        return gps_common::GPSStatus::STATUS_FIX;

      case novatel_oem7_msgs::PositionOrVelocityType::WAAS:
      case novatel_oem7_msgs::PositionOrVelocityType::INS_SBAS:
      case novatel_oem7_msgs::PositionOrVelocityType::PPP_CONVERGING:
      case novatel_oem7_msgs::PositionOrVelocityType::PPP:
      case novatel_oem7_msgs::PositionOrVelocityType::INS_PPP_CONVERGING:
      case novatel_oem7_msgs::PositionOrVelocityType::INS_PPP:
      case novatel_oem7_msgs::PositionOrVelocityType::PPP_BASIC_CONVERGING:
      case novatel_oem7_msgs::PositionOrVelocityType::PPP_BASIC:
      case novatel_oem7_msgs::PositionOrVelocityType::INS_PPP_BASIC_CONVERGING:
      case novatel_oem7_msgs::PositionOrVelocityType::INS_PPP_BASIC:
        return gps_common::GPSStatus::STATUS_SBAS_FIX;

      case novatel_oem7_msgs::PositionOrVelocityType::NONE:
        return gps_common::GPSStatus::STATUS_NO_FIX;

      default:
        ROS_ERROR_STREAM("Unknown OEM7 PositionOrVelocityType: " << bestpos->pos_type.type);
        return gps_common::GPSStatus::STATUS_NO_FIX;
    };
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


  /*
   * Relation: greater than, less than, equal
   */
  enum ValueRelation
  {
    REL_GT,
    REL_LT,
    REL_EQ
  };

  /*
   * Determine the time relation between two message headers
   * @return REL_GT when msg_hdr_1 was generated by Oem7 later than msg_hdr_2
   */
  ValueRelation
  GetOem7MessageTimeRelation(
      novatel_oem7_msgs::Oem7Header msg_hdr_1,
      novatel_oem7_msgs::Oem7Header msg_hdr_2)
  {
    if(msg_hdr_1.gps_week_number > msg_hdr_2.gps_week_number)
      return REL_GT;

    if(msg_hdr_1.gps_week_number == msg_hdr_2.gps_week_number)
    {
      if(msg_hdr_1.gps_week_milliseconds > msg_hdr_2.gps_week_milliseconds)
        return REL_GT;

      if(msg_hdr_1.gps_week_milliseconds == msg_hdr_1.gps_week_milliseconds)
        return REL_EQ;
    }

    return REL_LT;
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

  uint8_t GpsStatusToNavSatStatus(int16_t gps_status)
  {
    // Keep this in sync with the return values of ToROSGPSStatus
    switch(gps_status)
    {
      case gps_common::GPSStatus::STATUS_NO_FIX:
        return sensor_msgs::NavSatStatus::STATUS_NO_FIX;

      case gps_common::GPSStatus::STATUS_FIX:
        return sensor_msgs::NavSatStatus::STATUS_FIX;

      case gps_common::GPSStatus::STATUS_SBAS_FIX:
      case gps_common::GPSStatus::STATUS_WAAS_FIX:
        return sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;

      case gps_common::GPSStatus::STATUS_DGPS_FIX:
      case gps_common::GPSStatus::STATUS_GBAS_FIX:
        return sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;

      default:
        ROS_ERROR_STREAM("Unknown gps status: " << gps_status);
        return gps_common::GPSStatus::STATUS_NO_FIX;
    }
  }


  uint16_t
  NavSatStatusService(novatel_oem7_msgs::BESTPOS::Ptr bestpos)
  {
    uint16_t service = 0;

    if(bestpos->gps_glonass_sig_mask & 0x07)
    {
      service |= sensor_msgs::NavSatStatus::SERVICE_GPS;
    }

    if(bestpos->gps_glonass_sig_mask & 0x70)
    {
      service |= sensor_msgs::NavSatStatus::SERVICE_GLONASS;
    }

    if(bestpos->galileo_beidou_sig_mask & 0x0F)
    {
      service |= sensor_msgs::NavSatStatus::SERVICE_GALILEO;
    }

    if(bestpos->galileo_beidou_sig_mask & 0x70)
    {
      service |= sensor_msgs::NavSatStatus::SERVICE_COMPASS;
    }

    return service;
  }

  /**
   * Get Geometry (UTM) point from GNSS position, assuming zero origin.
   */
  void UTMPointFromGnss(
          geometry_msgs::Point& pt,
          double lat,
          double lon,
          double hgt)
  {
    pt.z = hgt;

    std::string zone; //unused
    gps_common::LLtoUTM(lat, lon, pt.y, pt.x, zone);
  }

  /**
   * Returns true if INS Solution is available
   */
  bool IsINSSolutionAvailable(const novatel_oem7_msgs::InertialSolutionStatus& status)
  {
    switch(status.status)
    {
      case novatel_oem7_msgs::InertialSolutionStatus::INS_HIGH_VARIANCE:
      case novatel_oem7_msgs::InertialSolutionStatus::INS_SOLUTION_GOOD:
      case novatel_oem7_msgs::InertialSolutionStatus::INS_SOLUTION_FREE:
      case novatel_oem7_msgs::InertialSolutionStatus::INS_ALIGNMENT_COMPLETE:
            return true;

      default:
        return false;
    }
  }

  /***
   * Handler of position-related messages. Synthesizes ROS messages GPSFix and NavSatFix from native Oem7 Messages.
   */
  class BESTPOSHandler: public Oem7MessageHandlerIf
  {
    Oem7RosPublisher BESTGNSSPOS_pub_;
    Oem7RosPublisher BESTPOS_pub_;
    Oem7RosPublisher BESTUTM_pub_;
    Oem7RosPublisher BESTVEL_pub_;
    Oem7RosPublisher PPPPOS_pub_;
    Oem7RosPublisher TERRASTARINFO_pub_;
    Oem7RosPublisher TERRASTARSTATUS_pub_;
    Oem7RosPublisher INSPVA_pub_;

    Oem7RosPublisher GPSFix_pub_;
    Oem7RosPublisher NavSatFix_pub_;
    Oem7RosPublisher Odometry_pub_;

    boost::shared_ptr<novatel_oem7_msgs::BESTGNSSPOS> bestgnsspos_;
    boost::shared_ptr<novatel_oem7_msgs::BESTPOS> bestpos_;
    boost::shared_ptr<novatel_oem7_msgs::BESTVEL> bestvel_;
    boost::shared_ptr<novatel_oem7_msgs::INSPVA>  inspva_;
    boost::shared_ptr<novatel_oem7_msgs::INSPVAX> inspvax_;

    boost::shared_ptr<gps_common::GPSFix> gpsfix_;

    Oem7RawMessageIf::ConstPtr psrdop2_;

    int64_t last_bestpos_;
    int64_t last_bestvel_;
    int64_t last_inspva_;

    int32_t bestpos_period_;
    int32_t bestvel_period_;
    int32_t inspva_period_;

    std::string base_frame_; ///< Base frame for Odometry

    bool position_source_BESTPOS_; //< User override: always use BESTPOS
    bool position_source_INS_; ///< User override: always use INS

    tf2::Quaternion Z90_DEG_ROTATION; ///< Rotate ENU to ROS frames.


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

    void publishBESTGNSSPOS(Oem7RawMessageIf::ConstPtr msg)
    {
      MakeROSMessage(msg, bestgnsspos_);


      BESTGNSSPOS_pub_.publish(bestgnsspos_);
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

    void publishPPPPOS(Oem7RawMessageIf::ConstPtr msg)
    {
        boost::shared_ptr<novatel_oem7_msgs::PPPPOS> ppppos;
        MakeROSMessage(msg, ppppos);
        PPPPOS_pub_.publish(ppppos);
    }

    void publishTERRASTARINFO(Oem7RawMessageIf::ConstPtr msg)
    {
        boost::shared_ptr<novatel_oem7_msgs::TERRASTARINFO> terrastarinfo;
        MakeROSMessage(msg, terrastarinfo);
        TERRASTARINFO_pub_.publish(terrastarinfo);
    }

    void publishTERRASTARSTATUS(Oem7RawMessageIf::ConstPtr msg)
    {
        boost::shared_ptr<novatel_oem7_msgs::TERRASTARSTATUS> terrastarstatus;
        MakeROSMessage(msg, terrastarstatus);
        TERRASTARSTATUS_pub_.publish(terrastarstatus);
    }

    void publishINSVPA(Oem7RawMessageIf::ConstPtr msg)
    {
      MakeROSMessage(msg, inspva_);
      updatePeriod(inspva_, last_inspva_, inspva_period_);

      INSPVA_pub_.publish(inspva_);
    }

    void processPositionAndPublishGPSFix()
    {
      gpsfix_.reset(new gps_common::GPSFix);

      gpsfix_->status.position_source     = gps_common::GPSStatus::SOURCE_NONE;
      gpsfix_->status.orientation_source  = gps_common::GPSStatus::SOURCE_NONE;
      gpsfix_->status.motion_source       = gps_common::GPSStatus::SOURCE_NONE;
      gpsfix_->position_covariance_type   = gps_common::GPSFix::COVARIANCE_TYPE_UNKNOWN;

      // BESTPOS has the highest quality values, use them by default. They may be overriden later.
      // This is deliberately not optimized for clarity.

      if(bestpos_)
      {
        gpsfix_->latitude   = bestpos_->lat;
        gpsfix_->longitude  = bestpos_->lon;
        gpsfix_->altitude   = bestpos_->hgt;

        // Convert stdev to diagonal covariance
        gpsfix_->position_covariance[0] = std::pow(bestpos_->lon_stdev, 2);
        gpsfix_->position_covariance[4] = std::pow(bestpos_->lat_stdev, 2);
        gpsfix_->position_covariance[8] = std::pow(bestpos_->hgt_stdev, 2);
        gpsfix_->position_covariance_type = gps_common::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        gpsfix_->err_horz = computeHorizontalError(bestpos_->lon_stdev, bestpos_->lat_stdev);
        gpsfix_->err_vert = computeVerticalError(  bestpos_->hgt_stdev);
        gpsfix_->err      = computeSphericalError( bestpos_->lon_stdev, bestpos_->lat_stdev, bestpos_->hgt_stdev);

        gpsfix_->time = MakeGpsTime_Seconds(
                          bestpos_->nov_header.gps_week_number,
                          bestpos_->nov_header.gps_week_milliseconds);


        gpsfix_->status.satellites_visible = bestpos_->num_svs;
        gpsfix_->status.satellites_used    = bestpos_->num_sol_svs;
        gpsfix_->status.status             = ToROSGPSStatus(bestpos_);

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
          gpsfix_->status.motion_source |= gps_common::GPSStatus::SOURCE_DOPPLER;
        }
        else
        {
          gpsfix_->status.motion_source |= gps_common::GPSStatus::SOURCE_POINTS;
        }
      }

      if(inspva_ )
      {
        double undulation = 0;

        // Populate INS data
        gpsfix_->pitch  = -inspva_->pitch;
        gpsfix_->roll   =  inspva_->roll;
        //gpsfix->dip: not populated.

        // BESTPOS/BESTVEL take INS into account
        gpsfix_->status.position_source   |= (gps_common::GPSStatus::SOURCE_GYRO | gps_common::GPSStatus::SOURCE_ACCEL);
        gpsfix_->status.orientation_source = (gps_common::GPSStatus::SOURCE_GYRO | gps_common::GPSStatus::SOURCE_ACCEL);
        gpsfix_->status.motion_source =      (gps_common::GPSStatus::SOURCE_GYRO | gps_common::GPSStatus::SOURCE_ACCEL);

        // Use most recent timestamp
        gpsfix_->time = MakeGpsTime_Seconds(
                          inspva_->nov_header.gps_week_number,
                          inspva_->nov_header.gps_week_milliseconds);




        // For normal installations, INSPVA messages are sent at much higher rate than BESTPOS/BESTVEL.
        // More recent INSPVAS are preferred, unless they report inferior accuracy.
        // This takes effect, unless explicitly overriden:
        assert(position_source_BESTPOS_ != position_source_INS_ || !position_source_BESTPOS_); // Can't both be true, both can be false.
        bool prefer_INS = position_source_INS_; // Init to override value
        if(!position_source_INS_ && !position_source_BESTPOS_) // Not overriden: determine source on-the-fly based on quality
        {
          if( IsINSSolutionAvailable(inspva_->status) &&
              bestpos_                                &&
              inspvax_)
          {
            ValueRelation time_rel = GetOem7MessageTimeRelation(inspva_->nov_header, bestpos_->nov_header);
            if(time_rel == REL_GT || time_rel == REL_EQ)
            {
              static const float ACCURACY_MARGIN_FACTOR = 1.1; // Avoid shifting rapidly between data sources.
              prefer_INS = Get3DPositionError(
                              inspvax_->latitude_stdev,
                              inspvax_->longitude_stdev,
                              inspvax_->height_stdev) <
                           Get3DPositionError(
                              bestpos_->lat_stdev,
                              bestpos_->lon_stdev,
                              bestpos_->hgt_stdev) * ACCURACY_MARGIN_FACTOR;
            }
          }
        }
        //-------------------------------------------------------------------------------------------------------
        // Log INS vs BESTPOS preference
        // This logic is not necessary for correct operation.
        static bool prev_prefer_INS = false;
        if(prev_prefer_INS != prefer_INS)
        {
          ROS_INFO_STREAM("GPSFix position source= INSPVA: " << prev_prefer_INS
                                                               << " --> " << prefer_INS
                                                               << " at GPSTime["
                                                               << inspva_->nov_header.gps_week_number         << " "
                                                               << inspva_->nov_header.gps_week_milliseconds   << "]"
                                                               );
        }
        prev_prefer_INS = prefer_INS;
        //--------------------------------------------------------------------------------------------------------

        if(!bestpos_ || prefer_INS)
        {
          gpsfix_->latitude   = inspva_->latitude;
          gpsfix_->longitude  = inspva_->longitude;
          gpsfix_->altitude   = inspva_->height;

          gpsfix_->status.position_source |= (gps_common::GPSStatus::SOURCE_GYRO | gps_common::GPSStatus::SOURCE_ACCEL);

          if(bestpos_)
          {
            gpsfix_->altitude = inspva_->height - bestpos_->undulation;
          }

          if(inspvax_)
          {
            // Convert stdev to diagonal covariance
            gpsfix_->position_covariance[0] = std::pow(inspvax_->longitude_stdev, 2);
            gpsfix_->position_covariance[4] = std::pow(inspvax_->latitude_stdev,  2);
            gpsfix_->position_covariance[8] = std::pow(inspvax_->height_stdev,    2);
            gpsfix_->position_covariance_type = gps_common::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

            if(!bestpos_)
            {
              gpsfix_->altitude = inspva_->height - inspvax_->undulation;
            }
          }
        }

        if(!bestvel_ || prefer_INS)
        {
           // Compute track and horizontal speed from north and east velocities

           gpsfix_->track = radiansToDegrees(
                               atan2(inspva_->north_velocity, inspva_->east_velocity));
           if(gpsfix_->track < 0.0)
           {
             gpsfix_->track + 360.0;
           }

           gpsfix_->speed = std::sqrt(std::pow(inspva_->north_velocity, 2.0) +
                                      std::pow(inspva_->east_velocity,  2.0));

           gpsfix_->climb = inspva_->up_velocity;
        }

      } // if(inspva_)


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

      GetNavSatFix(navsatfix);

      NavSatFix_pub_.publish(navsatfix);
    }
    /*** Generates NavSatFix object from GpsFix and BESTPOS
     */
    void GetNavSatFix(sensor_msgs::NavSatFix::Ptr navsatfix)
    {
      navsatfix->latitude    = gpsfix_->latitude;
      navsatfix->longitude   = gpsfix_->longitude;
      navsatfix->altitude    = gpsfix_->altitude;

      navsatfix->position_covariance[0]   = gpsfix_->position_covariance[0];
      navsatfix->position_covariance[4]   = gpsfix_->position_covariance[4];
      navsatfix->position_covariance[8]   = gpsfix_->position_covariance[8];
      navsatfix->position_covariance_type = GpsFixCovTypeToNavSatFixCovType(gpsfix_->position_covariance_type);

      navsatfix->status.status  = GpsStatusToNavSatStatus(gpsfix_->status.status);

      if(bestpos_)
      {
        navsatfix->status.service = NavSatStatusService(bestpos_);
      }
      else
      {
        ROS_DEBUG_STREAM("No BESTPOS to produce NavSatFix 'service'. ");
      }
   }

    void publishOdometry()
    {
      boost::shared_ptr<nav_msgs::Odometry> odometry(new nav_msgs::Odometry);
      odometry->child_frame_id = base_frame_;

      if(gpsfix_)
      {
        UTMPointFromGnss(
            odometry->pose.pose.position,
            gpsfix_->latitude,
            gpsfix_->longitude,
            gpsfix_->altitude);

        odometry->pose.covariance[ 0] = gpsfix_->position_covariance[0];
        odometry->pose.covariance[ 7] = gpsfix_->position_covariance[4];
        odometry->pose.covariance[14] = gpsfix_->position_covariance[8];
      }

      if(inspva_)
      {
        // INSPVA uses 'y-forward' ENU orientation;
        // ROS uses x-forward orientation.

        tf2::Quaternion enu_orientation;
        enu_orientation.setRPY(
                          degreesToRadians(inspva_->roll),
                         -degreesToRadians(inspva_->pitch),
                         -degreesToRadians(inspva_->azimuth));

        tf2::Quaternion ros_orientation = Z90_DEG_ROTATION * enu_orientation;

        tf2::Transform velocity_transform(ros_orientation);
        tf2::Vector3 local_frame_velocity = velocity_transform.inverse()(tf2::Vector3(inspva_->east_velocity, inspva_->north_velocity, inspva_->up_velocity));

        odometry->pose.pose.orientation = tf2::toMsg(ros_orientation);
        tf2::convert(local_frame_velocity, odometry->twist.twist.linear);
      } // inspva_


      if(inspvax_)
      {
        odometry->pose.covariance[21] = std::pow(inspvax_->roll_stdev,      2);
        odometry->pose.covariance[28] = std::pow(inspvax_->pitch_stdev,     2);
        odometry->pose.covariance[35] = std::pow(inspvax_->azimuth_stdev,   2);

        odometry->twist.covariance[0]  = std::pow(inspvax_->north_velocity_stdev, 2);
        odometry->twist.covariance[7]  = std::pow(inspvax_->east_velocity_stdev,  2);
        odometry->twist.covariance[14] = std::pow(inspvax_->up_velocity_stdev,    2);
      }

      Odometry_pub_.publish(odometry);
    }

    void publishROSMessages()
    {
      processPositionAndPublishGPSFix(); // Must be published first, since other message may be derived from it.

      publishNavSatFix();

      publishOdometry();
    }



  public:
    BESTPOSHandler():
      last_bestpos_(0),
      last_bestvel_(0),
      last_inspva_(0),
      bestpos_period_(INT_MAX),
      bestvel_period_(INT_MAX),
      inspva_period_( INT_MAX),
      position_source_BESTPOS_(false),
      position_source_INS_(false)
    {
      Z90_DEG_ROTATION.setRPY(0, 0, degreesToRadians(90.0));
    }

    ~BESTPOSHandler()
    {
    }

    void initialize(ros::NodeHandle& nh)
    {
      BESTGNSSPOS_pub_.setup<novatel_oem7_msgs::BESTGNSSPOS>("BESTGNSSPOS",   nh);
      BESTPOS_pub_.setup<novatel_oem7_msgs::BESTPOS>("BESTPOS",   nh);
      BESTVEL_pub_.setup<novatel_oem7_msgs::BESTVEL>("BESTVEL",   nh);
      BESTUTM_pub_.setup<novatel_oem7_msgs::BESTUTM>("BESTUTM",   nh);
      PPPPOS_pub_.setup<novatel_oem7_msgs::PPPPOS>(   "PPPPOS",   nh);
      TERRASTARINFO_pub_.setup<novatel_oem7_msgs::TERRASTARINFO>(     "TERRASTARINFO",  nh);
      TERRASTARSTATUS_pub_.setup<novatel_oem7_msgs::TERRASTARSTATUS>("TERRASTARSTATUS", nh);
      INSPVA_pub_.setup<novatel_oem7_msgs::INSPVA>(  "INSPVA",    nh);
      GPSFix_pub_.setup<gps_common::GPSFix>(         "GPSFix",    nh);
      NavSatFix_pub_.setup<sensor_msgs::NavSatFix>(  "NavSatFix", nh);
      Odometry_pub_.setup<nav_msgs::Odometry>(       "Odometry",  nh);

      nh.param<std::string>("base_frame", base_frame_, "base_link");

      // Determine if position source is overriden by the user; otherwise it is determined dynamically.
      std::string position_source;
      nh.getParam("position_source", position_source);
      if(position_source == "BESTPOS")
      {
        position_source_BESTPOS_ = true;
      }
      else if(position_source == "INSPVAS")
      {
        position_source_INS_ = true;
      }
      else
      {
        position_source = "BESTPOS or INSPVAS based on quality";
      }
      ROS_INFO_STREAM("GPSFix position source: " << position_source);
    }

    const std::vector<int>& getMessageIds()
    {
      static const std::vector<int> MSG_IDS(
                                    {
                                      BESTGNSSPOS_OEM7_MSGID,
                                      BESTPOS_OEM7_MSGID,
                                      BESTVEL_OEM7_MSGID,
                                      BESTUTM_OEM7_MSGID,
                                      PPPPOS_OEM7_MSGID,
                                      TERRASTARINFO_OEM7_MSGID,
                                      TERRASTARSTATUS_OEM7_MSGID,
                                      INSPVAS_OEM7_MSGID,
                                      INSPVAX_OEM7_MSGID,
                                      PSRDOP2_OEM7_MSGID
                                    });
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

      if(msg->getMessageId() == PPPPOS_OEM7_MSGID)
      {
        publishPPPPOS(msg);
      }

      if(msg->getMessageId() == TERRASTARINFO_OEM7_MSGID)
      {
        publishTERRASTARINFO(msg);
      }

      if(msg->getMessageId() == TERRASTARSTATUS_OEM7_MSGID)
      {
        publishTERRASTARSTATUS(msg);
      }
      
      if(msg->getMessageId() == INSPVAS_OEM7_MSGID)
      {
        publishINSVPA(msg);

        if(isShortestPeriod(inspva_period_))
        {
          publishROSMessages();
        }
      }

      if(msg->getMessageId() == INSPVAX_OEM7_MSGID)
      {
        MakeROSMessage<novatel_oem7_msgs::INSPVAX>(msg, inspvax_);
      }

      if(msg->getMessageId() == PSRDOP2_OEM7_MSGID)
      {
        psrdop2_ = msg;
      }

      if(msg->getMessageId() == BESTGNSSPOS_OEM7_MSGID)
      {
        publishBESTGNSSPOS(msg);
      }
    }
  };

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::BESTPOSHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
