#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <algorithm>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ArucoWaypointLandingNode : public rclcpp::Node
{
public:
    ArucoWaypointLandingNode() : Node("aruco_waypoint_landing_node")
    {
        offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/aruco_visualization", 10, std::bind(&ArucoWaypointLandingNode::aruco_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            std::bind(&ArucoWaypointLandingNode::odom_callback, this, std::placeholders::_1));
        local_position_sub_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::QoS(10).best_effort(),
            std::bind(&ArucoWaypointLandingNode::local_position_callback, this, std::placeholders::_1));
        clicked_wp_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/waypoint_array", 10, std::bind(&ArucoWaypointLandingNode::waypoints_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(100ms, std::bind(&ArucoWaypointLandingNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Aruco Waypoint Landing Node started.");
    }

private:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr clicked_wp_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose aruco_pose_;
    std::deque<bool> aruco_history_;
    const size_t required_aruco_detections_ = 5;

    bool aruco_detected_ = false;
    bool aruco_consistent_ = false;
    bool aruco_centered_ = false;
    bool armed_ = false;
    bool offboard_enabled_ = false;
    bool initialized_ = false;
    bool landing_ = false;
    bool got_waypoints_ = false;
    bool position_received_ = false;  // New flag to track if we got position data

    float current_x_ = 0.0f, current_y_ = 0.0f, current_z_ = 0.0f;
    float current_yaw_ = 0.0f;
    float initial_x_ = 0.0f, initial_y_ = 0.0f, initial_z_ = 0.0f;
    float step_ = 0.5f; // velocity
    float waypoint_hold_time_ = 5.0f;

    struct Waypoint { double x, y, z; }; 
    std::vector<Waypoint> pending_waypoints_;
    std::vector<Waypoint> waypoints_;
    
    size_t current_wp_index_ = 0;
    rclcpp::Time wp_reached_time_;
    bool holding_wp_ = false;

    void aruco_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        aruco_detected_ = !msg->poses.empty();
        if (aruco_detected_) aruco_pose_ = msg->poses[0];

        aruco_history_.push_back(aruco_detected_);
        if (aruco_history_.size() > required_aruco_detections_)
            aruco_history_.pop_front();

        aruco_consistent_ = static_cast<size_t>(
            std::count(aruco_history_.begin(), aruco_history_.end(), true)
        ) >= required_aruco_detections_;
    }

    void waypoints_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "WAYPOINT CALLBACK TRIGGERED!");
        pending_waypoints_.clear();
        
        for (const auto& pose : msg->poses) {
            pending_waypoints_.emplace_back(Waypoint{
                pose.position.x,
                pose.position.y,
                pose.position.z
            });
        }
        
        got_waypoints_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Received %ld waypoints", pending_waypoints_.size());
        
        // Log all waypoints:
        for (size_t i = 0; i < pending_waypoints_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Waypoint %ld: x=%.2f y=%.2f z=%.2f", 
                        i, pending_waypoints_[i].x, pending_waypoints_[i].y, pending_waypoints_[i].z);
        }

        // Try to initialize if we have both waypoints and position
        try_initialize();
    }
    
    void odom_callback(const VehicleOdometry::SharedPtr msg)
    {
        float q0 = msg->q[0], q1 = msg->q[1], q2 = msg->q[2], q3 = msg->q[3];
        current_yaw_ = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                                1.0f - 2.0f * (q2 * q2 + q3 * q3));
    }

    void local_position_callback(const VehicleLocalPosition::SharedPtr msg)
    {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = msg->z;
        
        if (!position_received_) {
            position_received_ = true;
            RCLCPP_INFO(this->get_logger(), "First position received: x=%.2f y=%.2f z=%.2f", 
                       current_x_, current_y_, current_z_);
        }

        // Try to initialize if we have both waypoints and position
        try_initialize();
    }

    void try_initialize()
    {
        if (!initialized_ && got_waypoints_ && position_received_) {
            initial_x_ = current_x_;
            initial_y_ = current_y_;
            initial_z_ = current_z_;
            
            // Clear any existing waypoints
            waypoints_.clear();
            
            // Transform waypoints from body frame to NED frame
            for (const auto& bp : pending_waypoints_) {
                float ned_x = initial_x_ + bp.x * std::cos(current_yaw_) - bp.y * std::sin(current_yaw_);
                float ned_y = initial_y_ + bp.x * std::sin(current_yaw_) + bp.y * std::cos(current_yaw_);
                float ned_z = initial_z_ + bp.z;
                waypoints_.push_back({ned_x, ned_y, ned_z});
            }

            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "INITIALIZED! Start pos: x=%.2f y=%.2f z=%.2f yaw=%.2f", 
                       initial_x_, initial_y_, initial_z_, current_yaw_);
            RCLCPP_INFO(this->get_logger(), "Transformed %ld waypoints to NED frame:", waypoints_.size());
            
            for (size_t i = 0; i < waypoints_.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "NED Waypoint %ld: x=%.2f y=%.2f z=%.2f", 
                           i, waypoints_[i].x, waypoints_[i].y, waypoints_[i].z);
            }
        }
    }

    void control_loop()
    {
        publish_offboard_control_mode();

        if (!offboard_enabled_) {
            for (int i = 0; i < 10; ++i) publish_offboard_control_mode();
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            offboard_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "Offboard mode enabled");
        }

        if (!armed_) {
            // Only try to arm if we're initialized
            if (initialized_) {
                arm();
                armed_ = true;
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                     "Waiting for initialization before arming...");
            }
            return;
        }

        if (!initialized_ || landing_) return;

        if (aruco_consistent_) {
            float dx = aruco_pose_.position.x;
            float dy = aruco_pose_.position.y;

            RCLCPP_INFO(this->get_logger(), "[ArUco] Pose in camera frame: dx=%.3f, dy=%.3f", dx, dy);

            float body_x = std::clamp(-dy, -0.15f, 0.15f);  // forward
            float body_y = std::clamp(dx, -0.15f, 0.15f);  // right
            
            float adjust_x = current_x_ + body_x * std::cos(current_yaw_) - body_y * std::sin(current_yaw_);
            float adjust_y = current_y_ + body_x * std::sin(current_yaw_) + body_y * std::cos(current_yaw_);

            RCLCPP_INFO(this->get_logger(), "[Adjustments] Moving to x=%.3f, y=%.3f", adjust_x, adjust_y);

            if (std::abs(dx) < 0.1f && std::abs(dy) < 0.1f) {
                RCLCPP_INFO(this->get_logger(), "ArUco centered. Landing...");
                land();
                landing_ = true;
                return;
            } else {
                publish_trajectory_setpoint(adjust_x, adjust_y, waypoints_[0].z, current_yaw_);
                return;
            }
        }

        if (current_wp_index_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "Path done. No ArUco. Landing...");
            land();
            landing_ = true;
            return;
        }

        Waypoint wp = waypoints_[current_wp_index_];
        float dx = wp.x - current_x_;
        float dy = wp.y - current_y_;
        float dz = wp.z - current_z_;
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < 0.3f) {
            if (!holding_wp_) {
                holding_wp_ = true;
                wp_reached_time_ = this->now();
                RCLCPP_INFO(this->get_logger(), "Reached waypoint %ld. Holding...", current_wp_index_);
            }
            if ((this->now() - wp_reached_time_).seconds() > waypoint_hold_time_) {
                current_wp_index_++;
                holding_wp_ = false;
                double elapsed = (this->now() - wp_reached_time_).seconds();
                RCLCPP_INFO(this->get_logger(), "Moving to next waypoint. Completed: %ld, time held: %.2f", 
                           current_wp_index_ - 1, elapsed);
            }
            publish_trajectory_setpoint(wp.x, wp.y, wp.z, current_yaw_);
        } else {
            float dir_x = dx / dist;
            float dir_y = dy / dist;
            float dir_z = dz / dist;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Moving to waypoint %ld: dist=%.2f, target=[%.2f,%.2f,%.2f]", 
                                current_wp_index_, dist, wp.x, wp.y, wp.z);
            publish_trajectory_setpoint(current_x_ + dir_x * step_,
                                        current_y_ + dir_y * step_,
                                        current_z_ + dir_z * step_,
                                        current_yaw_);
        }
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.timestamp = now().nanoseconds() / 1000;
        msg.position = true;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint(float x, float y, float z, float yaw)
    {
        TrajectorySetpoint msg{};
        msg.timestamp = now().nanoseconds() / 1000;
        msg.position = {x, y, z};
        msg.yaw = yaw;
        trajectory_setpoint_pub_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
    {
        VehicleCommand msg{};
        msg.timestamp = now().nanoseconds() / 1000;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(this->get_logger(), "Sending arm command...");
    }

    void land()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(this->get_logger(), "Landing...");
    }
};

int main(int argc, char* argv[])
{
    std::cout << "Starting ArUco Waypoint Landing Node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoWaypointLandingNode>());
    rclcpp::shutdown();
    return 0;
}