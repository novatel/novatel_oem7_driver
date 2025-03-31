////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2023 NovAtel Inc.
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


#include <memory>

#include <novatel_oem7_driver/oem7_message_handler_if.hpp>


#include <driver_parameter.hpp>
#include <novatel_oem7_driver/oem7_ros_messages.hpp>

#include <oem7_ros_publisher.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "novatel_oem7_msgs/msg/heading2.hpp"


#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <tf2_ros/transform_broadcaster.h>
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf2 {
inline geometry_msgs::msg::Vector3 toMsg(const Vector3& in) {
  geometry_msgs::msg::Vector3 out;
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

inline void fromMsg(const geometry_msgs::msg::Vector3& in, Vector3& out) {
  out = tf2::Vector3(in.x, in.y, in.z);
}
}  // namespace tf2
#endif

using sensor_msgs::msg::Imu;
using gps_msgs::msg::GPSFix;
using gps_msgs::msg::GPSStatus;
using geometry_msgs::msg::PointStamped;
using nav_msgs::msg::Odometry;
using novatel_oem7_msgs::msg::HEADING2;
using geometry_msgs::msg::PoseWithCovarianceStamped;

using tf2_ros::TransformBroadcaster;

namespace
{
  /***
   * @brief Converts degrees to Radians
   *
   * @return radians
   */
  inline double degreesToRadians(double degrees)
  {
    return degrees * M_PIf64 / 180.0;
  }

  /**
   * @brief Rotates a 3x3 covariance matrix
   * 
   * @return rotated cov matrix in tf2::Matrix3x3 form.
   */
  inline tf2::Matrix3x3 rotateCovMatrix(const tf2::Transform& tf,
                                                 const std::array<double, 9UL>& covIn)
  {
    tf2::Matrix3x3 covMat(covIn[0], covIn[1], covIn[2],
                            covIn[3], covIn[4], covIn[5],
                            covIn[6], covIn[7], covIn[8]);
    tf2::Quaternion q = tf.getRotation().normalized();
    if (!std::isnan(q.w())){
      return covMat;
    } else {
      tf2::Matrix3x3 tfRot = tf.getBasis();
      covMat = tfRot * covMat * (tfRot.transpose());
    }
    return covMat;
  }
}


namespace novatel_oem7_driver
{
  /***
   * Handles Odometry
   */
  class OdometryHandler: public Oem7MessageHandlerIf
  {
    rclcpp::Node* node_;

    std::unique_ptr<Oem7RosPublisher<Odometry>>       Odometry_pub_;
    std::unique_ptr<Oem7RosPublisher<PointStamped>>   Odometry_origin_pub_;
    std::unique_ptr<Oem7RosPublisher<Imu>>   Compass_pub_;

    std::unique_ptr<TransformBroadcaster> tf_bc_;

    Odometry odometry_;
    Imu compass_;

    rclcpp::Subscription<GPSFix>::SharedPtr    gpsfix_sub_;
    rclcpp::Subscription<Imu>::SharedPtr       imu_sub_;
    rclcpp::Subscription<HEADING2>::SharedPtr  heading2_sub_;

    GPSFix::SharedPtr   gpsfix_;
    HEADING2::SharedPtr heading2_;

    int utm_zone_; // UTM Zone we are operating in. Crossing zone boundary results in position jump.
    double meridian_convergence_; // meridian convergence at the current location.
    bool odom_zero_origin_;
    GeographicLib::LocalCartesian gps_local_cartesian_; // Local Cartesian projection around gps origin
    bool odom_zero_origin_set_;
    double odom_origin_x_;
    double odom_origin_y_;
    double odom_origin_z_;

    bool imu_present_; ///< Set to true when IMU output is detected

    std::string child_frame_;

    /**
    * Get Geometry (UTM) point from GNSS position, assuming zero origin.
    */
    bool UTMPointFromGnss(
            geometry_msgs::msg::Point& pt,
            double lat,
            double lon,
            double hgt)
    {
      pt.z = hgt;

      // unused:
      bool northhp = false;
      // scaling factor (unused)
      double k = 1.0;

      int zonespec = GeographicLib::UTMUPS::zonespec::MATCH;
      int new_utm_zone = 0;
      static unsigned int num_failed_conversions = 0;
      try
      {
        GeographicLib::UTMUPS::Forward(lat, lon, new_utm_zone, northhp, pt.x, pt.y, meridian_convergence_, k, zonespec);
      }
      catch(GeographicLib::GeographicErr& ex)
      {
        ++num_failed_conversions;
        auto& clk = *node_->get_clock();
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clk, 1000,
          "Failed Conversion (tot: " << num_failed_conversions << ") Lat: "  << lat      <<
                                                                   " Lon: "  << lon      <<
                                                                "; Zone: "   << zonespec <<
                                                                 std::endl   <<
                                                                 ex.what());
        return false;
      }
      meridian_convergence_ = degreesToRadians(meridian_convergence_);
      num_failed_conversions = 0;

      if(utm_zone_ != new_utm_zone)
      {
        RCLCPP_INFO_STREAM(node_->get_logger(),
          "UTM new Zone:  " << utm_zone_ << " --> " << new_utm_zone <<
          "; N: "            << northhp << " X: " << pt.x     << " Y: " << pt.y);
        utm_zone_ = new_utm_zone;
      }

      return true;
    }

    void LocalEnuFromGnss(
            geometry_msgs::msg::Point& pt,
            double lat,
            double lon,
            double hgt)
    {
      gps_local_cartesian_.Forward(lat, lon, hgt, pt.x, pt.y, pt.z);
      return;
    }

    void publishCompass(HEADING2 heading)
    {

      // Update the header
      compass_.header = heading.header; // Copy the header from HEADING2
      double yaw = M_PI_2 - heading.heading * M_PI / 180.0;
      double yaw_stdev = 100.0 * heading.heading_stdev * M_PI / 180.0;
      // Assign heading to the orientation in the pose
      // Assuming heading is in radians and corresponds to the yaw (z-axis rotation)
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);  // Roll and pitch are set to 0, yaw is set to heading
      compass_.orientation = tf2::toMsg(q);
      // Optionally set covariance values (for example, using heading_stdev)
      compass_.orientation_covariance[8] = yaw_stdev * yaw_stdev;
      // Publish the updated message
      Compass_pub_->publish(compass_);
    }


    void publishOdometry(sensor_msgs::msg::Imu* imu)
    {
      if(!gpsfix_)
      {
        // No data to derive basic Odometry values
        return;
      }

      odometry_.child_frame_id = child_frame_;

      if(odom_zero_origin_)
      {
        // set GPS origin
        // TODO: handle manual GPS origin
        if(!odom_zero_origin_set_)
        {
          if (gpsfix_->status.status == GPSStatus::STATUS_NO_FIX)
            return;
          std::unique_ptr<PointStamped> ptStamped = std::make_unique<PointStamped>();
          if(!UTMPointFromGnss(
              ptStamped->point,
              gpsfix_->latitude,
              gpsfix_->longitude,
              gpsfix_->altitude))
            return;
          gps_local_cartesian_.Reset(gpsfix_->latitude, gpsfix_->longitude, 0.0);
          odom_zero_origin_set_ = true;

          odom_origin_x_ = ptStamped->point.x;
          odom_origin_y_ = ptStamped->point.y;
          odom_origin_z_ = ptStamped->point.z;

          Odometry_origin_pub_->publish(std::move(ptStamped));

          RCLCPP_INFO_STREAM(node_->get_logger(),
                    "Odometry UTM Origin:  " << odom_origin_x_ << " " << odom_origin_y_);
        }
        LocalEnuFromGnss(
          odometry_.pose.pose.position,
          gpsfix_->latitude,
          gpsfix_->longitude,
          gpsfix_->altitude);
      }
      else if(!UTMPointFromGnss(
          odometry_.pose.pose.position,
          gpsfix_->latitude,
          gpsfix_->longitude,
          gpsfix_->altitude))
      {
        return;
      }

      odometry_.pose.covariance[ 0] = gpsfix_->position_covariance[0];
      odometry_.pose.covariance[ 7] = gpsfix_->position_covariance[4];
      odometry_.pose.covariance[14] = gpsfix_->position_covariance[8];
      tf2::Quaternion orientation;
      //tf2::fromMsg(imu->orientation, orientation);
      double track_enu = M_PI_2 - degreesToRadians(gpsfix_->track);
      orientation.setRPY(0., 0., track_enu);
      tf2::Transform local_tf(orientation); // Twist is rotated into local frame
      tf2::Transform local_tf_inv = local_tf.inverse();

      // orientation is in local ENU form track heading, but odom is in UTM, so we need to rotate it
      tf2::Quaternion meridian_rotation;
      meridian_rotation.setRPY(0., 0., meridian_convergence_); 
      orientation = meridian_rotation * orientation;
      odometry_.pose.pose.orientation = tf2::toMsg(orientation);

      odometry_.pose.covariance[21] = std::pow(std::atan2(0.3, gpsfix_->speed), 2); 
      odometry_.pose.covariance[28] = std::pow(std::atan2(0.3, gpsfix_->speed), 2);
      odometry_.pose.covariance[35] = std::pow(std::atan2(0.3, gpsfix_->speed), 2);

      if(gpsfix_->speed < 3.0) {
        odometry_.pose.covariance[21] = 1000.0;
        odometry_.pose.covariance[28] = 1000.0;
        odometry_.pose.covariance[35] = 1000.0;
      }

      if(imu) // Corrected is expected; no orientation in raw
      {
        // angular velocity in local ENU
        tf2::Vector3 angular_velocity;
        tf2::fromMsg(imu->angular_velocity, angular_velocity);
        angular_velocity = local_tf_inv(angular_velocity);
        odometry_.twist.twist.angular.x = angular_velocity.x();
        odometry_.twist.twist.angular.y = angular_velocity.y();
        odometry_.twist.twist.angular.z = angular_velocity.z();
        auto angular_vel_cov_rot = rotateCovMatrix(local_tf_inv, imu->angular_velocity_covariance);

        odometry_.twist.covariance[21] = angular_vel_cov_rot[0][0];
        //odometry_.twist.covariance[22] = angular_vel_cov_rot[0][1];
        //odometry_.twist.covariance[23] = angular_vel_cov_rot[0][2];
        //odometry_.twist.covariance[27] = angular_vel_cov_rot[1][0];
        odometry_.twist.covariance[28] = angular_vel_cov_rot[1][1];
        //odometry_.twist.covariance[29] = angular_vel_cov_rot[1][2];
        //odometry_.twist.covariance[33] = angular_vel_cov_rot[2][0];
        //odometry_.twist.covariance[34] = angular_vel_cov_rot[2][1];
        odometry_.twist.covariance[35] = angular_vel_cov_rot[2][2];

        // Linear velocity in local ENU
        // N.B. gpsfix_->track is in NED and in degrees
        double track_ground_rad = degreesToRadians(gpsfix_->track);
        // double sin_trk = std::sin(track_ground_rad);
        // double cos_trk = std::cos(track_ground_rad);
        // this is corrected yaw in UTM
        double roll_temp, pitch_temp, yaw_temp;
        tf2::Matrix3x3(orientation).getRPY(roll_temp, pitch_temp, yaw_temp); 
        double sin_yaw = std::sin(yaw_temp);
        double cos_yaw = std::cos(yaw_temp);
        // using corrected yaw in UTM to get local linear velocity
        tf2::Vector3 local_linear_velocity = local_tf_inv(tf2::Vector3(
                                                              gpsfix_->speed * cos_yaw,
                                                              gpsfix_->speed * sin_yaw,
                                                              gpsfix_->climb));
        // tf2::Vector3 local_linear_velocity = local_tf_inv(tf2::Vector3(
        //                                                           gpsfix_->speed * sin_trk,
        //                                                           gpsfix_->speed * cos_trk,
        //                                                           gpsfix_->climb));
        odometry_.twist.twist.linear.x = local_linear_velocity.x();
        odometry_.twist.twist.linear.y = local_linear_velocity.y();
        odometry_.twist.twist.linear.z = local_linear_velocity.z();

        // Use speed in gpsfix_. The speed is calculated from Doppler frequency shift.
        // A typical static stdev can be taken as 0.5m/s.
        // Ref: https://docs.novatel.com/OEM7/Content/Logs/BESTVEL.htm
        const auto linear_vel_cov_rot = rotateCovMatrix(local_tf_inv,
                                                      {0.25, 0., 0., 0., 0.25, 0., 0., 0., 0.25});
        odometry_.twist.covariance[0] = linear_vel_cov_rot[0][0];
        //odometry_.twist.covariance[1] = linear_vel_cov_rot[0][1];
        //odometry_.twist.covariance[2] = linear_vel_cov_rot[0][2];
        //odometry_.twist.covariance[6] = linear_vel_cov_rot[1][0];
        odometry_.twist.covariance[7] = linear_vel_cov_rot[1][1];
        //odometry_.twist.covariance[8] = linear_vel_cov_rot[1][2];
        //odometry_.twist.covariance[12] = linear_vel_cov_rot[2][0];
        //odometry_.twist.covariance[13] = linear_vel_cov_rot[2][1];
        odometry_.twist.covariance[14] = linear_vel_cov_rot[2][2];
      }

      Odometry_pub_->publish(odometry_);

      if(tf_bc_) // Publish Transform
      {
        geometry_msgs::msg::TransformStamped tf;
        // odometry_->header is automatically populated after publishing.
        tf.header.stamp    = odometry_.header.stamp;
        tf.header.frame_id = odometry_.header.frame_id;

        tf.child_frame_id = child_frame_;

        tf.transform.translation.x = odometry_.pose.pose.position.x;
        tf.transform.translation.y = odometry_.pose.pose.position.y;
        tf.transform.translation.z = odometry_.pose.pose.position.z;

        tf.transform.rotation = odometry_.pose.pose.orientation;

        tf_bc_->sendTransform(tf);
      }
    }

    std::string topic(const std::string& publisher)
    {
      std::string topic;
      node_->get_parameter(publisher + ".topic", topic);
      return std::string(node_->get_namespace()) + 
                        (node_->get_namespace() == std::string("/") ? topic : "/" + topic);
    }

  public:
    OdometryHandler():
      utm_zone_(-1),
      meridian_convergence_(0.),
      odom_zero_origin_(false),
      odom_zero_origin_set_(false),
      odom_origin_x_(0.0),
      odom_origin_y_(0.0),
      odom_origin_z_(0.0),
      imu_present_(false),
      child_frame_("base_link")
    {
    }

    ~OdometryHandler()
    {
    }

    void handleGPSFix(const GPSFix::SharedPtr gpsfix)
    {
      gpsfix_ = gpsfix;

      //GPSFix drives odometry, until IMU output is detected.
      if(!imu_present_)
      {
        publishOdometry(nullptr);
      }

    }

    void handleImu(const Imu::UniquePtr imu)
    {
      imu_present_ = true;

      publishOdometry(imu.get());
    }

    void handleHEADING2(const HEADING2::SharedPtr heading2)
    {
      publishCompass(*heading2);

    }

    void initialize(rclcpp::Node& node)
    {
      node_ = &node;

      Odometry_pub_         = std::make_unique<Oem7RosPublisher<Odometry>>( "Odometry",       node);
      Compass_pub_          = std::make_unique<Oem7RosPublisher<Imu>>("Compass",     node);
      Odometry_origin_pub_  = std::make_unique<Oem7RosPublisher<PointStamped>>( "OdometryOrigin", node);

      gpsfix_sub_  = node.create_subscription<GPSFix>( topic("GPSFix"),  10, std::bind(&OdometryHandler::handleGPSFix,  this, std::placeholders::_1));
      imu_sub_     = node.create_subscription<Imu>(    topic("IMU"),     10, std::bind(&OdometryHandler::handleImu,     this, std::placeholders::_1));
      heading2_sub_ = node.create_subscription<HEADING2>( topic("HEADING2"), 10, std::bind(&OdometryHandler::handleHEADING2,     this, std::placeholders::_1));
      DriverParameter<bool> odom_zero_origin_p("oem7_odometry_zero_origin", false, *node_);
      odom_zero_origin_ = odom_zero_origin_p.value();

      DriverParameter<bool> odom_transform_p("oem7_odometry_transform", false, *node_);
      if(odom_transform_p.value())
      {
        tf_bc_ = std::make_unique<TransformBroadcaster>(node);
      }

      DriverParameter<std::string> child_frame_p("oem7_odometry_child_frame", "base_link", *node_);
      child_frame_ = child_frame_p.value();
    }

    const MessageIdRecords& getMessageIds()
    {
      static const MessageIdRecords MSG_IDS;
      return MSG_IDS;
    }

    void handleMsg(const Oem7RawMessageIf::ConstPtr& msg)
    {
        assert(false);
    }
  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::OdometryHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
