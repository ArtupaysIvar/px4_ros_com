#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <algorithm>
#include <vector>
#include <string>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class visNode : public rclcpp::Node
{
public:
    visNode(size_t num_drones = 3) : Node("visNode_multi"), num_drones_(num_drones)
    {
        trajectory_setpoint_pubs_.resize(num_drones_);
        pose_pubs_.resize(num_drones_);
        path_pubs_.resize(num_drones_);
        path_msgs_.resize(num_drones_);
        initial_positions_.resize(num_drones_);

        // Set initial positions for each drone
        // Drone 1: (0, 0), Drone 2: (-2, -2), Drone 3: (2, -2)
        initial_positions_[0] = {0.0, 0.0, 0.0};
        initial_positions_[1] = {-2.0, -2.0, 0.0};
        initial_positions_[2] = {2.0, -2.0, 0.0};

        // create pubs/subs for each drone
        for (size_t i = 0; i < num_drones_; ++i) {
            std::string idx = std::to_string(i + 1);

            std::string traj_topic = "/px4_" + idx + "/fmu/in/trajectory_setpoint";
            trajectory_setpoint_pubs_[i] = this->create_publisher<TrajectorySetpoint>(traj_topic, 10);

            std::string pose_topic = "/drone" + idx + "/pose";
            pose_pubs_[i] = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

            std::string path_topic = "/drone" + idx + "/path";
            path_pubs_[i] = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);

            // initialize path frame
            path_msgs_[i].header.frame_id = "odom";

            std::string odom_topic = "/px4_" + idx + "/fmu/out/vehicle_odometry";
            auto qos = rclcpp::QoS(10).best_effort();

            // subscribe, capture index by value
            odom_subs_.push_back(
                this->create_subscription<VehicleOdometry>(
                    odom_topic, qos,
                    [this, i](const VehicleOdometry::SharedPtr msg) { this->odom_callback(i, msg); }
                )
            );
        }

        RCLCPP_INFO(this->get_logger(), "visNode_multi started for %zu drones", num_drones_);
    }

private:
    size_t num_drones_;

    std::vector<rclcpp::Publisher<TrajectorySetpoint>::SharedPtr> trajectory_setpoint_pubs_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs_;
    std::vector<rclcpp::Subscription<VehicleOdometry>::SharedPtr> odom_subs_;

    std::vector<nav_msgs::msg::Path> path_msgs_;
    std::vector<std::array<double, 3>> initial_positions_;

    // convert each vehicle_odometry msg into PoseStamped and Path (NED -> ENU)
    void odom_callback(size_t idx, const VehicleOdometry::SharedPtr msg)
    {
        if (idx >= num_drones_) return;

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "odom";

        // --- NED -> ENU position with offset ---
        // msg->position = [x_NED, y_NED, z_NED]
        // Apply initial position offset in ENU frame
        pose_msg.pose.position.x = msg->position[1] + initial_positions_[idx][0];   // y_NED -> x_ENU + offset
        pose_msg.pose.position.y = msg->position[0] + initial_positions_[idx][1];   // x_NED -> y_ENU + offset
        pose_msg.pose.position.z = -msg->position[2] + initial_positions_[idx][2];  // -z_NED -> z_ENU + offset

        // --- NED -> ENU orientation (arrow pointing UP perpendicular to grid) ---
        // msg->q is [x, y, z, w] (PX4 ordering)
        tf2::Quaternion q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]); // (x,y,z,w)
        tf2::Quaternion q_rot;
        // Rotate to make arrow point up (perpendicular to XY plane)
        // Rotate 90 degrees around Y axis to point arrow up
        q_rot.setRPY(0.0, -M_PI / 2.0, 0.0);
        tf2::Quaternion q_enu = q_rot * q_ned;
        q_enu.normalize();

        pose_msg.pose.orientation.x = q_enu.x();
        pose_msg.pose.orientation.y = q_enu.y();
        pose_msg.pose.orientation.z = q_enu.z();
        pose_msg.pose.orientation.w = q_enu.w();

        // publish pose
        pose_pubs_[idx]->publish(pose_msg);

        // append to path and publish (no trimming to keep path persistent)
        path_msgs_[idx].header.stamp = pose_msg.header.stamp;
        path_msgs_[idx].poses.push_back(pose_msg);

        path_pubs_[idx]->publish(path_msgs_[idx]);
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    // default 3 drones
    auto node = std::make_shared<visNode>(3);

    // use single-threaded executor to avoid needing locks (path growth trimming is done inline)
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}