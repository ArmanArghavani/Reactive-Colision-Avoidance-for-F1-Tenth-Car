#pragma once

#include <rclcpp/rclcpp.hpp>                                 // Core ROS2 C++ API :contentReference[oaicite:0]{index=0}
#include <sensor_msgs/msg/laser_scan.hpp>                    // sensor_msgs::msg::LaserScan :contentReference[oaicite:1]{index=1}
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>    // Change this to whatever runs the drive commands
#include <your_package/msg/trajectory.hpp>                   //Change this to match the trajectory node we're subscribing to
#include <vector>
#include <utility>
#include <sensor_msgs/msg/point_cloud.hpp>   // for sensor_msgs::msg::PointCloud
#include <cmath>
#include <reactive_col_avoid.hpp> // Include the header file for the car class

/// Function that processes LiDAR data and populates 'gaps' vector with (angle, distance) pairs where distance > 3 meters
void process_LiDAR(const sensor_msgs::LaserScan::ConstPtr& scan_msg, std::vector<std::pair<double, double>>& gaps, std::vector<std::pair<double, double>>& gapsMid)
{
    gaps.clear();  // Clear the gaps vector to avoid accumulating old data
    gapsMid.clear();  // Clear the gapsMid vector to avoid accumulating old data
    // Extract the necessary parameters from the LiDAR scan message
    double angle_min = scan_msg->angle_min;       // Starting angle of the scan (in radians)
    double angle_max = scan_msg->angle_max;       // Ending angle of the scan (in radians)
    double angle_increment = scan_msg->angle_increment;  // Angular step between each reading (in radians)

    int gapsize = 0; // Initialize gapsize to 0

    // Set the angular range you want to process (in radians)
    // For example, let's process -45° to 45° (-0.7854 to 0.7854 radians)
    double min_angle = -0.7854;  // -45 degrees
    double max_angle = 0.7854;   // 45 degrees

    // Loop through all the LiDAR range readings and process based on the angle
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        // Calculate the angle of the current reading using the starting angle and increment
        double angle = angle_min + i * angle_increment;

        // Check if the current angle falls within the desired range for processing
        if (angle >= min_angle && angle <= max_angle) {
            // Get the distance measurement at this angle
            double distance = scan_msg->ranges[i];

            // Only add the reading if the distance is greater than 3 meters
            if (distance > 3.0) {
                // Populate the 'gaps' array with the angle and distance pair
                gaps.push_back({angle, distance});
            }
        }
    }

    for (size_t j = 0; j < gaps.size(); ++j) {
        
        gapsize = 0; // Reset gapsize for each new gap

        while (gaps[j+1, 0] == gaps[j, 0] + angle_increment){

            gapsize++;
            j++;
        }

        if (gapsize > 38 ) { // need the gap size to result in the perpendicular length to midpoint line to be at least 2.5 car width 
                             // assuming 20cm car width and angle increment of 0.00436 radians (0.25 degrees)
                             // equation for gapsize = 2 *  (arcsin(1.25*carwidth/shortest possible gap distance) / 0.00436 radians)
                             // this results in a gap size of 38 consecutive hits over 3m

            for (size_t z = 0; z < 2; z++) {
                // Populate the 'gapsMid' array with the mid-point angle and distance pair
                gapsMid [j - gapsize/2, z] = gaps[j - gapsize/2, z]
            }
        }
    }
}

double choosebestgap(std::vector<std::pair<double, double>>& gapsMid) { // function that chooses the gap who's angle is closes to the
                                                                          // angle of the projected trajectory that it's subscribed to          
    double best_gap_angle = 1000; // Initialize the best gap angle
    double gapAngleDiff = 0.0; // Initialize the gap angle difference

    for(int i = 0; i < gapsMid.size(); i++) {
        gapAngleDiff = std::abs(gapsMid[i].first - latest_traj_); //CHECK NAME Calculate the absolute difference between the gap angle and the trajectory angle

        if (gapAngleDiff < best_gap_angle) { // If this gap is better than the previous best
            best_gap_angle = gapsMid[i].first; // Update the best gap angle
        }
    }
    return best_gap_angle; // SHOULD PUBLUSH THIS TO THE DRIVE COMMAND NODE
}