#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"

using namespace std::chrono_literals;

class OffboardArucoLand : public rclcpp::Node {
public:
    OffboardArucoLand() : Node("offboard_aruco_land") {
        // Publishers
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/in/vehicle_command", 10);
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);

        // Subscribers
        aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/aruco_visualization", 10,
            std::bind(&OffboardArucoLand::arucoCallback, this, std::placeholders::_1));

        // Timer
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardArucoLand::timerCallback, this));

        takeoff_height_ = 5.0;
        state_ = State::WAIT_FOR_CONNECTION;
        last_aruco_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Offboard Aruco Landing Node Initialized");
    }

private:
    enum class State { WAIT_FOR_CONNECTION, ARM_AND_TAKEOFF, HOVER, LANDING };

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_;
    double takeoff_height_;
    rclcpp::Time last_aruco_time_;
    geometry_msgs::msg::Pose last_marker_pose_;
    bool marker_detected_ = false;

    void timerCallback() {
        publishOffboardControlMode();

        switch (state_) {
            case State::WAIT_FOR_CONNECTION:
                arm();
                state_ = State::ARM_AND_TAKEOFF;
                break;
            case State::ARM_AND_TAKEOFF:
                publishSetpoint(0, 0, -takeoff_height_);
                state_ = State::HOVER;
                break;
            case State::HOVER:
                publishSetpoint(0, 0, -takeoff_height_);
                if (marker_detected_ && isMarkerCentered(last_marker_pose_)) {
                    RCLCPP_INFO(this->get_logger(), "Marker detected below. Landing...");
                    land();
                    state_ = State::LANDING;
                }
                break;
            case State::LANDING:
                // nothing to do, PX4 will land automatically
                break;
        }
    }

    void publishSetpoint(float x, float y, float z) {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = this->now().nanoseconds() / 1000;
        msg.position = {x, y, z};
        trajectory_setpoint_pub_->publish(msg);
    }

    void publishOffboardControlMode() {
        px4_msgs::msg::OffboardControlMode msg;
        msg.timestamp = this->now().nanoseconds() / 1000;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_control_mode_pub_->publish(msg);
    }

    void arm() {
        sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);  // PX4 OFFBOARD
        RCLCPP_INFO(this->get_logger(), "Sent ARM and OFFBOARD commands");
    }

    void land() {
        sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0);
    }

    void sendVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg;
        msg.timestamp = this->now().nanoseconds() / 1000;
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

    void arucoCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (!msg->poses.empty()) {
            last_marker_pose_ = msg->poses[0];
            last_aruco_time_ = this->now();
            marker_detected_ = true;
        } else {
            marker_detected_ = false;
        }
    }

    bool isMarkerCentered(const geometry_msgs::msg::Pose &pose) {
        const double threshold_xy = 0.1;  // meters
        return std::abs(pose.position.x) < threshold_xy && std::abs(pose.position.y) < threshold_xy;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardArucoLand>());
    rclcpp::shutdown();
    return 0;
}
