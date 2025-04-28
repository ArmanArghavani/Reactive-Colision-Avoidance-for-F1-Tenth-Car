// Include custom utility functions for Follow-The-Gap algorithm
#include "f110_reactive_methods/utility.h"

// Include ROS message types for Ackermann drive messages (steering + throttle control)
#include <ackermann_msgs/AckermannDriveStamped.h>

// Include core ROS functionality
#include <ros/ros.h>

// Include standard sensor_msgs for LiDAR data
#include <sensor_msgs/LaserScan.h>

/// Class that contains all the logic for ROS connections and Follow-the-Gap algorithm
class follow_the_gap
{
public:
    follow_the_gap() :
            node_handle_(ros::NodeHandle()), // Create a NodeHandle to interact with ROS system
            // Subscribe to LiDAR data on "scan" topic (IMPORTANT: change "scan" here if your LiDAR publishes elsewhere)
            lidar_sub_(node_handle_.subscribe("scan", 100, &follow_the_gap::scan_callback, this)),
            // Publisher to send driving commands (IMPORTANT: change "nav" to your robot's driving topic if needed)
            drive_pub_(node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 100)),
            truncated_(false) // Initially, no truncation of LiDAR data has been done
    {
        // Read all necessary parameters from ROS parameter server (configured via launch file or command line)
        node_handle_.getParam("/bubble_radius", bubble_radius_);
        node_handle_.getParam("/smoothing_filter_size", smoothing_filter_size_);
        node_handle_.getParam("/truncated_coverage_angle", truncated_coverage_angle_);
        node_handle_.getParam("/velocity", velocity_);
        node_handle_.getParam("/max_accepted_distance", max_accepted_distance_);
        node_handle_.getParam("/error_based_velocities", error_based_velocities_);
        node_handle_.getParam("/steering_angle_reactivity", steering_angle_reactivity_);
    }

    /// Pre-process the incoming LiDAR scan
    /// - Replaces NaNs with 0
    /// - Limits max range values
    /// - Applies smoothing
    std::vector<double> preprocess_lidar_scan(const sensor_msgs::LaserScan::ConstPtr &scan_msg) const
    {
        std::vector<double> filtered_ranges;
        for(size_t i = truncated_start_index_; i < truncated_end_index_; ++i)
        {
            if (std::isnan(scan_msg->ranges[i]))
            {
                filtered_ranges.push_back(0.0); // If NaN, treat as 0 (unusable)
            }
            else if (scan_msg->ranges[i] > max_accepted_distance_ || std::isinf(scan_msg->ranges[i]))
            {
                filtered_ranges.push_back(max_accepted_distance_); // Limit too large distances
            }
            else
            {
                filtered_ranges.push_back(scan_msg->ranges[i]); // Valid measurement
            }
        }
        // Apply smoothing filter (moving average, etc.)
        return fgm::apply_smoothing_filter(filtered_ranges, smoothing_filter_size_);
    }

    /// Picks the best point in a gap — simply the midpoint between start and end
    size_t get_best_point(const std::vector<double>& filtered_ranges, int start_index, int end_index) const
    {
        return (start_index + end_index) / 2;
    }

    /// Calculate the steering angle needed based on the best LiDAR point
    /// - Also compensates based on how close an obstacle is
    double get_steering_angle_from_range_index(const sensor_msgs::LaserScan::ConstPtr &scan_msg,
                                               const size_t best_point_index,
                                               const double closest_value)
    {
        // Correct the index back to the full scan frame
        const size_t best_point_index_input_scan_frame = truncated_start_index_ + best_point_index;
        double best_point_steering_angle;

        // If point is to the left of center, angle is negative
        if(best_point_index_input_scan_frame < scan_msg->ranges.size() / 2)
        {
            best_point_steering_angle = - scan_msg->angle_increment *
                               static_cast<double>(scan_msg->ranges.size()/2.0 - best_point_index_input_scan_frame);
        }
        else
        {
            best_point_steering_angle = scan_msg->angle_increment *
                               static_cast<double>(best_point_index_input_scan_frame - scan_msg->ranges.size()/2.0);
        }

        // Clamp and scale the steering based on proximity (more reactive if closer to obstacles)
        const auto distance_compensated_steering_angle =
                std::clamp((best_point_steering_angle * steering_angle_reactivity_) /
                static_cast<double>(closest_value), -1.57, 1.57); // Clamp between -90 and 90 degrees (in radians)

        return distance_compensated_steering_angle;
    }

    /// Main callback when LiDAR publishes a new scan
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        if (!truncated_)
        {
            // Only on first scan: determine the indices corresponding to the desired truncated angle
            ROS_INFO("input scan start angle = %f", scan_msg->angle_min);
            ROS_INFO("input scan end angle = %f", scan_msg->angle_max);
            ROS_INFO("input scan start index = %u", 0);
            ROS_INFO("input scan end index = %u", static_cast<u_int>(scan_msg->ranges.size()));

            // Find start and end indices to crop the LiDAR field of view (e.g., ignore rear-facing beams)
            const auto truncated_indices =
                    fgm::truncated_start_and_end_indices(scan_msg, truncated_coverage_angle_);
            truncated_start_index_ = truncated_indices.first;
            truncated_end_index_ = truncated_indices.second;
            truncated_ = true;

            ROS_INFO("truncated scan start angle = %f", -truncated_coverage_angle_/2);
            ROS_INFO("truncated scan end angle = %f", truncated_coverage_angle_/2);
            ROS_INFO("truncated start index = %u", static_cast<u_int>(truncated_start_index_));
            ROS_INFO("truncated end index = %u", static_cast<u_int>(truncated_end_index_));
        }

        // Step 1: Pre-process the scan
        auto filtered_ranges = preprocess_lidar_scan(scan_msg);

        // Step 2: Find the closest obstacle
        const size_t closest_index = fgm::minimum_element_index(filtered_ranges);
        const auto closest_range = filtered_ranges[closest_index];

        // Step 3: Create a safety bubble around closest obstacle (zero out points around it)
        fgm::zero_out_safety_bubble(&filtered_ranges, closest_index, bubble_radius_);

        // Step 4: Find the largest gap (nonzero sequence after bubble masking)
        const auto [start_index, end_index] = fgm::find_largest_nonzero_sequence(filtered_ranges);

        // Step 5: Pick the best point inside the gap to drive towards
        const size_t best_point_index = get_best_point(filtered_ranges, start_index, end_index);

        // Step 6: Calculate the required steering angle
        const double steering_angle = get_steering_angle_from_range_index(scan_msg, best_point_index, closest_range);

        // Step 7: Publish the steering + speed command
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now(); // Set timestamp
        drive_msg.header.frame_id = "laser"; // Frame of reference (IMPORTANT: update if your frame_id is "base_link" or different!)

        drive_msg.drive.steering_angle = steering_angle;

        // Choose speed based on how sharp the turn is
        if(abs(steering_angle) > 0.349) // Sharp turn
        {
            drive_msg.drive.speed = error_based_velocities_["high"];
        }
        else if(abs(steering_angle) > 0.174) // Medium turn
        {
            drive_msg.drive.speed = error_based_velocities_["medium"];
        }
        else // Straight
        {
            drive_msg.drive.speed = error_based_velocities_["low"];
        }

        drive_pub_.publish(drive_msg);
    }

private:
    // --- ROS Variables ---
    ros::NodeHandle node_handle_;  // Main ROS node handle
    ros::Subscriber lidar_sub_;    // Subscriber for LiDAR scans
    ros::Publisher drive_pub_;     // Publisher for driving commands

    // --- Parameters ---
    double bubble_radius_;               // Radius of safety bubble
    double max_accepted_distance_;       // Max allowed range reading
    double smoothing_filter_size_;       // Size of smoothing window
    double steering_angle_reactivity_;   // How aggressively we react to obstacles
    bool truncated_;                     // Flag if truncation has been done
    double truncated_coverage_angle_;    // How wide is the LiDAR slice considered
    size_t truncated_start_index_;       // Start index after truncation
    size_t truncated_end_index_;         // End index after truncation
    double velocity_;                    // Nominal velocity (unused directly in logic above)
    std::map<std::string, double> error_based_velocities_; // Map to set different speeds based on steering angle
};

/// Main entry point
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "follow_the_gap_node"); // Initialize ROS node
    follow_the_gap gap_follower; // Create object
    ros::spin(); // Keep node alive, responding to callbacks
    return 0;
}