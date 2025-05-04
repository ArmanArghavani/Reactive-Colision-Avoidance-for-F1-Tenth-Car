#pragma once

#include <rclcpp/rclcpp.hpp>                                 // Core ROS2 C++ API
#include <sensor_msgs/msg/laser_scan.hpp>                    // sensor_msgs::msg::LaserScan
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>    // Drive command message
#include <your_package/msg/trajectory.hpp>                   // Projected trajectory message
#include <vector>
#include <utility>
#include <cmath> 
#include <car.hpp>
#include <common.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>   // for sensor_msgs::msg::PointCloud


class reactiveColAvoid : public rclcpp::Node
{
public:
  reactiveColAvoid();

private:
  // LiDAR processing: fills 'gaps' and 'gapsMid'
  void process_LiDAR(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg,
    std::vector<std::pair<double, double>>& gaps,
    std::vector<std::pair<double, double>>& gapsMid);

  // Chooses best gap index based on subscribed trajectory and publishes drive command
  void chooseBestGap();

  // Callbacks
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void traj_callback(const your_package::msg::PointCloud::SharedPtr /traj_msg);

  // Subscriptions & publication

  sensor_msgs::msg::PointCloud::SharedPtr traj_msg; // might just be /traj_msg
  bool traj_received_{false};

// Subscriber handle
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr traj_sub_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr      scan_sub_; //Getting errors on these three lines for some reason
 
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

  // State
  your_package::msg::Trajectory         latest_traj_;
  bool                                  traj_received_{false};
  std::vector<std::pair<double,double>> last_gaps_;
};
