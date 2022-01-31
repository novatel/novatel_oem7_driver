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

#include <ros/ros.h>


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <novatel_oem7_driver/oem7_ros_messages.hpp>
#include <novatel_oem7_driver/oem7_messages.h>
#include <novatel_oem7_driver/oem7_imu.hpp>

#include "sensor_msgs/Imu.h"
#include "novatel_oem7_msgs/CORRIMU.h"
#include "novatel_oem7_msgs/IMURATECORRIMU.h"
#include "novatel_oem7_msgs/INSSTDEV.h"
#include "novatel_oem7_msgs/INSCONFIG.h"
#include "novatel_oem7_msgs/INSPVA.h"
#include "novatel_oem7_msgs/INSPVAX.h"

#include <boost/scoped_ptr.hpp>
#include <oem7_ros_publisher.hpp>

#include <math.h>
#include <map>



namespace novatel_oem7_driver
{
  /***
   * Converts degrees to Radians
   *
   * @return radians
   */
  inline double degreesToRadians(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  const double DATA_NOT_AVAILABLE = -1.0; ///< Used to initialized unpopulated fields.

  class INSHandler: public Oem7MessageHandlerIf
  {
    ros::NodeHandle nh_;

    Oem7RosPublisher       imu_pub_;
    Oem7RosPublisher       raw_imu_pub_;
    Oem7RosPublisher       corrimu_pub_;
    Oem7RosPublisher       insstdev_pub_;
    Oem7RosPublisher       inspvax_pub_;
    Oem7RosPublisher       insconfig_pub_;

    boost::shared_ptr<novatel_oem7_msgs::INSPVA>   inspva_;
    boost::shared_ptr<novatel_oem7_msgs::CORRIMU>  corrimu_;
    boost::shared_ptr<novatel_oem7_msgs::INSSTDEV> insstdev_;

    imu_rate_t imu_rate_; ///< IMU output rate
    double     imu_raw_gyro_scale_factor_;   ///< IMU-specific raw gyroscope scaling
    double     imu_raw_accel_scale_factor_;  ///< IMU-specific raw acceleration scaling.

    std::string frame_id_;

    typedef std::map<std::string, std::string> imu_config_map_t;
    imu_config_map_t imu_config_map;

    bool oem7_imu_reference_frame_; ///< Backwards compatibility: use OEM7 reference frame, not compliant with REP105.

    void getImuParam(oem7_imu_type_t imu_type, const std::string& name, std::string& param)
    {
      std::string ns = ros::this_node::getNamespace();
      std::string param_name = ns + "/supported_imus/" + std::to_string(imu_type) + "/" + name;
      if(!nh_.getParam(param_name, param))
      {
        ROS_FATAL_STREAM("INS: IMU type= " << imu_type << " is not supported.");
      }
    }

    int getImuRate(oem7_imu_type_t imu_type)
    {
      std::string rate;
      getImuParam(imu_type, "rate", rate);

      return std::stoi(rate);
    }

    void getImuDescription(oem7_imu_type_t imu_type, std::string& desc)
    {
      getImuParam(imu_type, "name", desc);
    }


    void processInsConfigMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      boost::shared_ptr<novatel_oem7_msgs::INSCONFIG> insconfig;
      MakeROSMessage(msg, insconfig);
      insconfig_pub_.publish(insconfig);

      oem7_imu_type_t imu_type = static_cast<oem7_imu_type_t>(insconfig->imu_type);

      std::string imu_desc;
      getImuDescription(imu_type, imu_desc);

      if(imu_rate_ == 0)
      {
        imu_rate_ = getImuRate(imu_type);
      }

      if(imu_raw_gyro_scale_factor_  == 0.0 &&
         imu_raw_accel_scale_factor_ == 0.0)
      {
          if(!getImuRawScaleFactors(
              imu_type,
              imu_rate_,
              imu_raw_gyro_scale_factor_,
              imu_raw_accel_scale_factor_))

        {
          ROS_ERROR_STREAM("Scale factors not supported for IMU '" << insconfig->imu_type << "'; raw IMU output disabled.");
        }
      }

      ROS_LOG_STREAM(imu_rate_ == 0 ? ::ros::console::levels::Error :
                                      ::ros::console::levels::Info,
                     ROSCONSOLE_DEFAULT_NAME,
                     "IMU: " << imu_type  << " '"  << imu_desc << "'"
                                          << " rate= "         << imu_rate_
                                          << " gyro scale= "   << imu_raw_gyro_scale_factor_
                                          << " accel scale= "  << imu_raw_accel_scale_factor_);
    }

    void publishInsPVAXMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      boost::shared_ptr<novatel_oem7_msgs::INSPVAX> inspvax;
      MakeROSMessage(msg, inspvax);

      inspvax_pub_.publish(inspvax);
    }

    void publishCorrImuMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      MakeROSMessage(msg, corrimu_);
      corrimu_pub_.publish(corrimu_);
    }


    void publishImuMsg()
    {
      if(!imu_pub_.isEnabled())
      {
        return;
      }

      boost::shared_ptr<sensor_msgs::Imu> imu(new sensor_msgs::Imu);

      if(oem7_imu_reference_frame_)
      {
        publishImuMsg_OEM7(imu);
      }
      else
      {
        publishImuMsg_ROS(imu);
      }

      if(insstdev_)
      {
        imu->orientation_covariance[0] = std::pow(insstdev_->roll_stdev,    2);
        imu->orientation_covariance[4] = std::pow(insstdev_->pitch_stdev,   2);
        imu->orientation_covariance[8] = std::pow(insstdev_->azimuth_stdev, 2);
      }

      imu->angular_velocity_covariance[0]    = DATA_NOT_AVAILABLE;
      imu->linear_acceleration_covariance[0] = DATA_NOT_AVAILABLE;

      imu_pub_.publish(imu);
    }

    void publishImuMsg_OEM7(boost::shared_ptr<sensor_msgs::Imu>& imu)
    {
      if(inspva_)
      {
        tf2::Quaternion tf_orientation;
        tf_orientation.setRPY(
                           degreesToRadians(inspva_->roll),
                          -degreesToRadians(inspva_->pitch),
                          -degreesToRadians(inspva_->azimuth));
        imu->orientation = tf2::toMsg(tf_orientation);
      }
      else
      {
        ROS_WARN_THROTTLE(10, "INSPVA not available; 'Imu' message not generated.");
        return;
      }

      if(corrimu_ && corrimu_->imu_data_count && imu_rate_ > 0)
      {
        double instantaneous_rate_factor = imu_rate_ / corrimu_->imu_data_count;

        imu->angular_velocity.x = corrimu_->pitch_rate * instantaneous_rate_factor;
        imu->angular_velocity.y = corrimu_->roll_rate  * instantaneous_rate_factor;
        imu->angular_velocity.z = corrimu_->yaw_rate   * instantaneous_rate_factor;

        imu->linear_acceleration.x = corrimu_->lateral_acc      * instantaneous_rate_factor;
        imu->linear_acceleration.y = corrimu_->longitudinal_acc * instantaneous_rate_factor;
        imu->linear_acceleration.z = corrimu_->vertical_acc     * instantaneous_rate_factor;
      }
    }

    void publishImuMsg_ROS(boost::shared_ptr<sensor_msgs::Imu>& imu)
    {
      if(inspva_)
      {	
        // Azimuth: Oem7 (North=0) to ROS (East=0), using Oem7 LH rule
        static const double ZERO_DEGREES_AZIMUTH_OFFSET = 90.0;
        double azimuth = inspva_->azimuth - ZERO_DEGREES_AZIMUTH_OFFSET;
        
        static const double AZIMUTH_ROLLOVER = 360 - ZERO_DEGREES_AZIMUTH_OFFSET;
        if(azimuth < -AZIMUTH_ROLLOVER) // Rollover
        {
          azimuth += AZIMUTH_ROLLOVER;
        }

        tf2::Quaternion tf_orientation;
        tf_orientation.setRPY(
                           degreesToRadians(inspva_->roll),
                          -degreesToRadians(inspva_->pitch),
                          -degreesToRadians(azimuth));

        imu->orientation = tf2::toMsg(tf_orientation);
      }
      else
      {
        ROS_WARN_THROTTLE(10, "INSPVA not available; 'Imu' message not generated.");
        return;
      }

      if(corrimu_ && corrimu_->imu_data_count > 0 && imu_rate_ > 0)
      {
        double instantaneous_rate_factor = imu_rate_ / corrimu_->imu_data_count;

        imu->angular_velocity.x =  corrimu_->roll_rate  * instantaneous_rate_factor;
        imu->angular_velocity.y = -corrimu_->pitch_rate * instantaneous_rate_factor;
        imu->angular_velocity.z =  corrimu_->yaw_rate   * instantaneous_rate_factor;

        imu->linear_acceleration.x =  corrimu_->longitudinal_acc * instantaneous_rate_factor;
        imu->linear_acceleration.y = -corrimu_->lateral_acc      * instantaneous_rate_factor;
        imu->linear_acceleration.z =  corrimu_->vertical_acc     * instantaneous_rate_factor;
      }
    }


    void publishInsStDevMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      MakeROSMessage(msg, insstdev_);
      insstdev_pub_.publish(insstdev_);
    }


    /**
     * @return angular velocity, rad / sec
     */
    inline double computeAngularVelocityFromRaw(double raw_gyro)
    {
      return raw_gyro * imu_raw_gyro_scale_factor_ * imu_rate_;
    }

    /**
     * @return linear acceleration, m / sec^2
     */
    inline double computeLinearAccelerationFromRaw(double raw_acc)
    {
      return raw_acc * imu_raw_accel_scale_factor_ * imu_rate_;
    }


    void processRawImuMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      if(!raw_imu_pub_.isEnabled())
      {
        return;
      }

      if(imu_rate_ == 0                     ||
         imu_raw_gyro_scale_factor_  == 0.0 ||
         imu_raw_accel_scale_factor_ == 0.0)
      {
        ROS_WARN_THROTTLE(10, "Unavailable or Invalid IMU rate and/or raw scale factors: %i %f %f",
                          imu_rate_, imu_raw_gyro_scale_factor_, imu_raw_accel_scale_factor_);

        return;
      }

      const RAWIMUSXMem* raw = reinterpret_cast<const RAWIMUSXMem*>(msg->getMessageData(OEM7_BINARY_MSG_SHORT_HDR_LEN));
      // All measurements are in sensor frame, uncorrected for gravity and Earth rotation. There is no up, forward, left;
      // x, y, z are nominal references to enclosure housing.

      boost::shared_ptr<sensor_msgs::Imu> imu = boost::make_shared<sensor_msgs::Imu>();
      imu->angular_velocity.x =  computeAngularVelocityFromRaw(raw->x_gyro);
      imu->angular_velocity.y = -computeAngularVelocityFromRaw(raw->y_gyro); // Refer to RAWIMUSX documentation
      imu->angular_velocity.z =  computeAngularVelocityFromRaw(raw->z_gyro);

      imu->linear_acceleration.x =  computeLinearAccelerationFromRaw(raw->x_acc);
      imu->linear_acceleration.y = -computeLinearAccelerationFromRaw(raw->y_acc);  // Refer to RASIMUSX documentation
      imu->linear_acceleration.z =  computeLinearAccelerationFromRaw(raw->z_acc);

      imu->angular_velocity_covariance[0]    = DATA_NOT_AVAILABLE;
      imu->linear_acceleration_covariance[0] = DATA_NOT_AVAILABLE;

      raw_imu_pub_.publish(imu);
    }


  public:
    INSHandler():
      imu_rate_(0),
      imu_raw_gyro_scale_factor_(0.0),
      imu_raw_accel_scale_factor_(0.0),
      oem7_imu_reference_frame_(false)
    {
    }

    ~INSHandler()
    {
    }

    void initialize(ros::NodeHandle& nh)
    {
      nh_ = nh;

      imu_pub_.setup<sensor_msgs::Imu>(                  "IMU",        nh);
      raw_imu_pub_.setup<sensor_msgs::Imu>(              "RAWIMU",        nh);
      corrimu_pub_.setup<  novatel_oem7_msgs::CORRIMU>(  "CORRIMU",    nh);
      insstdev_pub_.setup< novatel_oem7_msgs::INSSTDEV>( "INSSTDEV",   nh);
      inspvax_pub_.setup<  novatel_oem7_msgs::INSPVAX>(  "INSPVAX",    nh);
      insconfig_pub_.setup<novatel_oem7_msgs::INSCONFIG>("INSCONFIG",  nh);

      // User overrides for IMU
      nh.getParam("imu_rate",               imu_rate_);
      nh.getParam("imu_gyro_scale_factor",  imu_raw_gyro_scale_factor_);
      nh.getParam("imu_accel_scale_factor", imu_raw_accel_scale_factor_);
      if(imu_rate_ != 0                     ||
         imu_raw_gyro_scale_factor_  != 0.0 ||
         imu_raw_accel_scale_factor_ != 0.0)
      {
        ROS_INFO_STREAM("INS: IMU config overrides to rate= " << imu_rate_
                          << " gyro scale factor= "           << imu_raw_gyro_scale_factor_
                          << " accel scale factor= "          << imu_raw_accel_scale_factor_);
      }




      if(!nh_.getParam("oem7_imu_reference_frame", oem7_imu_reference_frame_))
      {
        if(oem7_imu_reference_frame_)
        {
          ROS_WARN_STREAM("INS Reference Frame: using OEM7 (X-forward) instead of ROS REP105.");
        }
      }
    }

    const std::vector<int>& getMessageIds()
    {
      static const std::vector<int> MSG_IDS(
                                      {
                                        RAWIMUSX_OEM7_MSGID,
                                        CORRIMUS_OEM7_MSGID,
                                        IMURATECORRIMUS_OEM7_MSGID,
                                        INSPVAS_OEM7_MSGID,
                                        INSPVAX_OEM7_MSGID,
                                        INSSTDEV_OEM7_MSGID,
                                        INSCONFIG_OEM7_MSGID
                                      }
                                    );
      return MSG_IDS;
    }

    void handleMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      ROS_DEBUG_STREAM("INS < [id= " <<  msg->getMessageId() << "]");

      if(msg->getMessageId()== INSPVAS_OEM7_MSGID)
      {
        MakeROSMessage(msg, inspva_); // Cache
      }
      else if(msg->getMessageId() == INSSTDEV_OEM7_MSGID)
      {
        publishInsStDevMsg(msg);
      }
      else if(msg->getMessageId() == CORRIMUS_OEM7_MSGID ||
              msg->getMessageId() == IMURATECORRIMUS_OEM7_MSGID)
      {
        publishCorrImuMsg(msg);

        publishImuMsg();
      }
      else if(msg->getMessageId() == INSCONFIG_OEM7_MSGID)
      {
        processInsConfigMsg(msg);
      }
      else if(msg->getMessageId() == INSPVAX_OEM7_MSGID)
      {
        publishInsPVAXMsg(msg);
      }
      else if(msg->getMessageId() == RAWIMUSX_OEM7_MSGID)
      {
        processRawImuMsg(msg);
      }
      else
      {
        assert(false);
      }
    }
  };

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::INSHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
