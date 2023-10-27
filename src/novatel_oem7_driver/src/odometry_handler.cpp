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

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <oem7_ros_publisher.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "novatel_oem7_msgs/msg/inspvax.hpp"

#include <GeographicLib/UTMUPS.hpp>


using sensor_msgs::msg::Imu;
using gps_msgs::msg::GPSFix;
using gps_msgs::msg::GPSStatus;
using nav_msgs::msg::Odometry;

using novatel_oem7_msgs::msg::INSPVA;
using novatel_oem7_msgs::msg::INSPVAX;

using tf2_ros::TransformBroadcaster;

namespace
{
  const std::string CHILD_FRAME_ID = "base_link";
  /***
   * Converts degrees to Radians
   *
   * @return radians
   */
  inline double degreesToRadians(double degrees)
  {
    return degrees * M_PI / 180.0;
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
    std::unique_ptr<Oem7RosPublisher<Odometry>>       Odometry_origin_pub_;

    std::unique_ptr<TransformBroadcaster> tf_bc_;

    rclcpp::Subscription<GPSFix>::SharedPtr    gpsfix_sub_;
    rclcpp::Subscription<Imu>::SharedPtr       imu_sub_;
    rclcpp::Subscription<INSPVA>::SharedPtr    inspva_sub_;
    rclcpp::Subscription<INSPVAX>::SharedPtr   inspvax_sub_;

    GPSFix::SharedPtr   gpsfix_;
    Imu::SharedPtr      imu_;
    INSPVA::SharedPtr   inspva_;
    INSPVAX::SharedPtr  inspvax_;

    int utm_zone_; // UTM Zone we are operating in. Crossing zone boundary results in position jump.
    bool odom_zero_origin_;
    bool odom_zero_origin_set_;
    double odom_origin_x_;
    double odom_origin_y_;
    double odom_origin_z_;

    bool imu_present_; ///< Set to true when IMU output is detected 
  
    /**
    * Get Geometry (UTM) point from GNSS position, assuming zero origin.
    */
    bool UTMPointFromGnss(
            geometry_msgs::msg::Point& pt,
            double lat,
            double lon,
            double hgt)
    {
      pt.z = hgt + inspvax_->undulation;

      // unused:
      bool northhp = false; 
      double k = 0.0;
      double gamma = 0.0;

      int zonespec = utm_zone_ == -1 ? GeographicLib::UTMUPS::zonespec::MATCH : utm_zone_;
      int new_utm_zone = 0;
      static unsigned int num_failed_conversions = 0;
      try
      {
        GeographicLib::UTMUPS::Forward(lat, lon, new_utm_zone, northhp, pt.x, pt.y, gamma, k, zonespec);
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

    void publishOdometry()
    {
      if(!gpsfix_ || !inspvax_)
      {
        // No data to derive basic Odometry values
        return;
      }

      std::shared_ptr<Odometry> odometry = std::make_shared<Odometry>();
      odometry->child_frame_id = CHILD_FRAME_ID;
  
      if(!UTMPointFromGnss(
          odometry->pose.pose.position,
          gpsfix_->latitude,
          gpsfix_->longitude,
          gpsfix_->altitude))
      {
        return;
      }


      odometry->pose.covariance[ 0] = gpsfix_->position_covariance[0];
      odometry->pose.covariance[ 7] = gpsfix_->position_covariance[4];
      odometry->pose.covariance[14] = gpsfix_->position_covariance[8];
    
      if(imu_) // Corrected is expected; no orientation in raw
      {
        odometry->pose.pose.orientation = imu_->orientation;
      
        odometry->pose.covariance[21] = imu_->orientation_covariance[0];
        odometry->pose.covariance[28] = imu_->orientation_covariance[4];
        odometry->pose.covariance[35] = imu_->orientation_covariance[8];

        tf2::Quaternion orientation;
        tf2::fromMsg(imu_->orientation, orientation);
        tf2::Transform local_tf(orientation); // Twist is rotated into local frame

        tf2::Vector3 angular_velocity;
        tf2::fromMsg(imu_->angular_velocity, angular_velocity);
        tf2::convert(local_tf.inverse()(angular_velocity), odometry->twist.twist.angular); 
        
        tf2::Vector3 local_angular_vel_cov = local_tf.inverse()(tf2::Vector3(
                                                                imu_->angular_velocity_covariance[0], 
                                                                imu_->angular_velocity_covariance[4],
                                                                imu_->angular_velocity_covariance[8]
                                                              ));

        odometry->twist.covariance[21] = local_angular_vel_cov.getX();
        odometry->twist.covariance[28] = local_angular_vel_cov.getY();
        odometry->twist.covariance[35] = local_angular_vel_cov.getZ();

         
        // Linear velocity
        //
        if(inspva_)
        {   
          tf2::Vector3 local_linear_velocity = local_tf.inverse()(tf2::Vector3(
                                                                          inspva_->east_velocity, 
                                                                          inspva_->north_velocity, 
                                                                          inspva_->up_velocity));
          tf2::convert(local_linear_velocity, odometry->twist.twist.linear);        
        }
        if(inspvax_)
        {
          tf2::Vector3 local_linear_vel_cov = local_tf.inverse()(tf2::Vector3(
                                                                  std::pow(inspvax_->east_velocity_stdev,  2),
                                                                  std::pow(inspvax_->north_velocity_stdev, 2),
                                                                  std::pow(inspvax_->up_velocity_stdev,    2)
                                                                ));
      
          odometry->twist.covariance[ 0] = local_linear_vel_cov.getX();
          odometry->twist.covariance[ 7] = local_linear_vel_cov.getY();
          odometry->twist.covariance[14] = local_linear_vel_cov.getZ();
        }
      }


      if(odom_zero_origin_ && 
        !odom_zero_origin_set_ && 
         gpsfix_->status.status != GPSStatus::STATUS_NO_FIX)
      {
          odom_zero_origin_set_ = true; 

          odom_origin_x_ = odometry->pose.pose.position.x;
          odom_origin_y_ = odometry->pose.pose.position.y;
          odom_origin_z_ = odometry->pose.pose.position.z;

          Odometry_origin_pub_->publish(odometry);

          RCLCPP_INFO_STREAM(node_->get_logger(),
                    "Odometry UTM Origin:  " << odom_origin_x_ << " " << odom_origin_y_);
      }
   
      odometry->pose.pose.position.x -= odom_origin_x_;
      odometry->pose.pose.position.y -= odom_origin_y_;
      odometry->pose.pose.position.z -= odom_origin_z_;

      Odometry_pub_->publish(odometry);

      if(tf_bc_) // Publish Transform
      {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = odometry->header.stamp;
        tf.header.frame_id = odometry->header.frame_id;
        
        tf.child_frame_id = CHILD_FRAME_ID;
        
        tf.transform.translation.x = odometry->pose.pose.position.x;
        tf.transform.translation.y = odometry->pose.pose.position.y;
        tf.transform.translation.z = odometry->pose.pose.position.z;

        tf.transform.rotation = odometry->pose.pose.orientation;

        tf_bc_->sendTransform(tf);
      }
    }
  

  public:
    OdometryHandler():
      utm_zone_(-1),
      odom_zero_origin_(false),
      odom_zero_origin_set_(false),
      odom_origin_x_(0.0),
      odom_origin_y_(0.0),
      odom_origin_z_(0.0),
      imu_present_(false)
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
        publishOdometry();
      }

    }

    void handleINSPVA(const INSPVA::SharedPtr inspva)
    {
      inspva_ = inspva;
    }
    void handleINSPVAX(const INSPVAX::SharedPtr inspvax)
    {
      inspvax_ = inspvax;
    }

    void handleImu(const Imu::SharedPtr imu)
    {
      imu_ = imu;

      imu_present_ = true;

      publishOdometry();
    }


    std::string topic(std::string publisher)
    {
      std::string topic;
      node_->get_parameter(publisher + ".topic", topic);
      return std::string(node_->get_namespace()) + 
                        (node_->get_namespace() == std::string("/") ? topic : "/" + topic);
    }

    void initialize(rclcpp::Node& node)
    {
      node_ = &node;

      Odometry_pub_         = std::make_unique<Oem7RosPublisher<Odometry>>( "Odometry",       node);
      Odometry_origin_pub_  = std::make_unique<Oem7RosPublisher<Odometry>>( "OdometryOrigin", node);

      gpsfix_sub_  = node.create_subscription<GPSFix>( topic("GPSFix"),  10, std::bind(&OdometryHandler::handleGPSFix,  this, std::placeholders::_1));
      imu_sub_     = node.create_subscription<Imu>(    topic("IMU"),     10, std::bind(&OdometryHandler::handleImu,     this, std::placeholders::_1));
      inspva_sub_  = node.create_subscription<INSPVA>( topic("INSPVA"),  10, std::bind(&OdometryHandler::handleINSPVA,  this, std::placeholders::_1));
      inspvax_sub_ = node.create_subscription<INSPVAX>(topic("INSPVAX"), 10, std::bind(&OdometryHandler::handleINSPVAX, this, std::placeholders::_1));
   
      DriverParameter<bool> odom_zero_origin_p("oem7_odometry_zero_origin", false, *node_);
      odom_zero_origin_ = odom_zero_origin_p.value();

      DriverParameter<bool> odom_transform_p("oem7_odometry_transform", false, *node_);
      if(odom_transform_p.value())
      {
        tf_bc_ = std::make_unique<TransformBroadcaster>(node);
      }
    }

    const MessageIdRecords& getMessageIds()
    {
      static const MessageIdRecords MSG_IDS;
      return MSG_IDS;
    }

    void handleMsg(Oem7RawMessageIf::ConstPtr msg)
    {
        assert(false);
    }
  };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(novatel_oem7_driver::OdometryHandler, novatel_oem7_driver::Oem7MessageHandlerIf)
