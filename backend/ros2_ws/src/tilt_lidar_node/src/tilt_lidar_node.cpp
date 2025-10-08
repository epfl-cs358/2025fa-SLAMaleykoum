#include <memory>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "laser_geometry/laser_geometry.hpp"

using std::placeholders::_1;

class TiltLaserNode : public rclcpp::Node {
public:
  TiltLaserNode(const rclcpp::NodeOptions & options)
  : Node("tilt_laser_node", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Topics
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
    scan_sub_       = create_subscription<sensor_msgs::msg::LaserScan>(
                        "/scan", 10, std::bind(&TiltLaserNode::scanCallback, this, _1));
    servo_sub_      = create_subscription<std_msgs::msg::Int32>(
                        "/lidar_servo_angle", 10, std::bind(&TiltLaserNode::servoCallback, this, _1));

    // 1) Inject static mount: servo_frame → laser_frame
    {
      geometry_msgs::msg::TransformStamped mount_tf;
      mount_tf.header.frame_id    = "servo_frame";
      mount_tf.child_frame_id     = "laser_frame";
      mount_tf.header.stamp       = rclcpp::Time(0);  // static
      mount_tf.transform.translation.x = 0.0;
      mount_tf.transform.translation.y = 0.0;
      mount_tf.transform.translation.z = 0.04; // the lidar is 0.04m above the servo
      tf2::Quaternion q_mount;
      q_mount.setRPY(0,0,0);
      mount_tf.transform.rotation.x = q_mount.x();
      mount_tf.transform.rotation.y = q_mount.y();
      mount_tf.transform.rotation.z = q_mount.z();
      mount_tf.transform.rotation.w = q_mount.w();
      tf_buffer_.setTransform(mount_tf, "static_mount", true);
      RCLCPP_INFO(get_logger(),"Injected static mount servo_frame→laser_frame");
    }

    // 2) Inject initial servo tilt = 90° horizontal as base_link → servo_frame (static)
    //    (we treat 90° as “zero tilt” for horizontal)
    {
      current_servo_angle_deg_ = 90;
      geometry_msgs::msg::TransformStamped init_servo;
      init_servo.header.frame_id    = "base_link";
      init_servo.child_frame_id     = "servo_frame";
      init_servo.header.stamp       = rclcpp::Time(0);
      init_servo.transform.translation.x = 0.0;
      init_servo.transform.translation.y = 0.0;
      init_servo.transform.translation.z = 0.0;
      double a0 = (current_servo_angle_deg_ - 90.0) * M_PI/180.0;  // = 0
      tf2::Quaternion q0;
      q0.setRPY(0, a0, 0);
      init_servo.transform.rotation.x = q0.x();
      init_servo.transform.rotation.y = q0.y();
      init_servo.transform.rotation.z = q0.z();
      init_servo.transform.rotation.w = q0.w();
      tf_buffer_.setTransform(init_servo, "static_servo", true);
      RCLCPP_INFO(get_logger(),"Injected initial base_link→servo_frame @0 as horizontal");
    }

    RCLCPP_INFO(get_logger(),"TiltLaserNode started (servo=90°→horizontal)");
  }

private:
  void servoCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    current_servo_angle_deg_ = msg->data;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    // compute scan start/end in ROS time
    rclcpp::Time t_start(scan_msg->header.stamp);
    double dur = (scan_msg->time_increment > 0.0)
                 ? scan_msg->time_increment * scan_msg->ranges.size()
                 : scan_msg->scan_time;
    rclcpp::Time t_end = t_start + rclcpp::Duration::from_seconds(dur);

    RCLCPP_INFO(get_logger(),
                "scan@%.3f dur=%.3f → [%.3f]",
                t_start.seconds(), dur, t_end.seconds());

    // inject base_link→servo_frame at t_start and t_end with offset
    injectServoAt(t_start);
    injectServoAt(t_end);

    // check that base_link→laser_frame is now available at t_end
    if (!tf_buffer_.canTransform(
          "base_link", "laser_frame", t_end,
          rclcpp::Duration::from_seconds(0.0)))
    {
      RCLCPP_WARN(get_logger(),
                  "TF chain not ready at %.3f → skipping",
                  t_end.seconds());
      return;
    }

    // perform the high-fidelity projection
    sensor_msgs::msg::PointCloud2 cloud2;
    try {
      projector_.transformLaserScanToPointCloud(
        "base_link", *scan_msg, cloud2, tf_buffer_);
      RCLCPP_INFO(get_logger(),
                  "projected %zu points",
                  cloud2.width * cloud2.height);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(),
                   "projection failed: %s", ex.what());
      return;
    }

    // publish the 3D cloud with the original scan timestamp
    cloud2.header.frame_id = "base_link";
    cloud2.header.stamp    = scan_msg->header.stamp;
    pointcloud_pub_->publish(cloud2);
    RCLCPP_INFO(get_logger(),
                "published PointCloud2 @ %.3f",
                t_start.seconds());
  }

  // helper: inject base_link→servo_frame at an arbitrary stamp, offset by -90°
  void injectServoAt(const rclcpp::Time & stamp) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id    = "base_link";
    tf_msg.child_frame_id     = "servo_frame";
    tf_msg.header.stamp       = stamp;
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    // subtract 90° so that servo=90°→horizontal=0 tilt
    double a = - (current_servo_angle_deg_ - 90.0) * M_PI/180.0; // - 90 degrees to have horizontal plane at 0 in base_link. and use - because of the direction of rotation
    tf2::Quaternion q;
    q.setRPY(0, a, 0);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_buffer_.setTransform(tf_msg, "servo", false);
  }

  // Members
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  servo_sub_;

  tf2_ros::Buffer           tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  laser_geometry::LaserProjection projector_;
  int current_servo_angle_deg_{90};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<TiltLaserNode>(opts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
