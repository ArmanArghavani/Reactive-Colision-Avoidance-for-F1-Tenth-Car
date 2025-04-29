#pragma once

#include <rclcpp/rclcpp.hpp>                                 // Core ROS2 C++ API
#include <sensor_msgs/msg/laser_scan.hpp>                    // sensor_msgs::msg::LaserScan
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>    // Drive command message
#include <your_package/msg/trajectory.hpp>                   // Projected trajectory message
#include <vector>
#include <utility>

class ReactiveColAvoid : public rclcpp::Node
{
public:
  ReactiveColAvoid();

private:
  // LiDAR processing: fills 'gaps' and 'gapsMid'
  void process_LiDAR(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg,
    std::vector<std::pair<double, double>>& gaps,
    std::vector<std::pair<double, double>>& gapsMid);

  // Chooses best gap index based on subscribed trajectory
  int32_t choosebestgap(std::vector<std::pair<double, double>>& gapsMid);

  // Callbacks
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void traj_callback(const your_package::msg::Trajectory::SharedPtr traj_msg);

  // Subscriptions & publication
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr     scan_sub_;
  rclcpp::Subscription<your_package::msg::Trajectory>::SharedPtr   traj_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

  // State
  your_package::msg::Trajectory         latest_traj_;
  bool                                  traj_received_{false};
  std::vector<std::pair<double,double>> last_gaps_;
};
