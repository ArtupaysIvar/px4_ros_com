#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/path.hpp>
//#include <geometry_msgs/msg/pose_array.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <algorithm>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class visNode : public rclcpp::Node
{
public:
visNode() : Node("visNode")
    {
        trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/drone/pose", 10);

        odom_sub_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            std::bind(&visNode::odom_callback, this, std::placeholders::_1));
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone/path", 10);
        
        path_msg_.header.frame_id = "odom";  // Only needs to be set once
        // timer_ = this->create_wall_timer(100ms, std::bind(&visNode::odom_callback, this));
    }
    private:
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;

    float current_x_ = 0.0f, current_y_ = 0.0f, current_z_ = 0.0f;
    float current_yaw_ = 0.0f;
    

    void odom_callback(const VehicleOdometry::SharedPtr msg)
    {
        current_x_ = msg->position[0];
        current_y_ = msg->position[1];
        current_z_ = msg->position[2];

        float q0 = msg->q[0], q1 = msg->q[1], q2 = msg->q[2], q3 = msg->q[3];
        current_yaw_ = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                                  1.0f - 2.0f * (q2 * q2 + q3 * q3));

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "odom";  // Change if you're using different TF root

        // --- NED → ENU: Position ---
        pose_msg.pose.position.x =  msg->position[1];   // y_NED → x_ENU
        pose_msg.pose.position.y =  msg->position[0];   // x_NED → y_ENU
        pose_msg.pose.position.z = -msg->position[2];   // -z_NED → z_ENU

        // --- NED → ENU: Orientation (q_NED to q_ENU) ---
        // Conversion: q_ENU = q_rot * q_NED
        // Where q_rot = [0.5, 0.5, -0.5, 0.5] (90 deg around Z then X)
        tf2::Quaternion q_ned(msg->q[1], msg->q[0], msg->q[2], msg->q[3]);  // PX4 uses [x, y, z, w] in NED
        tf2::Quaternion q_rot;
        q_rot.setRPY(M_PI, 0, M_PI/2);  // Rotate 90 deg around Z, then 180 around X

        tf2::Quaternion q_enu = q_rot * q_ned;
        q_enu.normalize();

        pose_msg.pose.orientation.x = q_enu.x();
        pose_msg.pose.orientation.y = q_enu.y();
        pose_msg.pose.orientation.z = q_enu.z();
        pose_msg.pose.orientation.w = q_enu.w();

        pose_pub_->publish(pose_msg);
        /*
        trajectory_.header.frame_id = msg.header.frame_id;
        geometry_msgs::msg::PoseStamped curr_pose;
        curr_pose.header.frame_id = msg.header.frame_id;
        curr_pose.header.stamp = msg.header.stamp;
        curr_pose.pose = msg.pose.pose;
        trajectory_.poses.push_back(curr_pose);*/

   
        // path_msg_.header.frame_id = "map";  // Same frame as PoseStamped
        path_msg_.header.stamp = this->get_clock()->now();
        path_msg_.poses.push_back(pose_msg);
        path_pub_->publish(path_msg_);
    }
};


int main(int argc, char* argv[])
{
    std::cout << "Starting 3d visualization node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<visNode>());
    rclcpp::shutdown();
    return 0;
}

