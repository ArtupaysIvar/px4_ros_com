#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ArucoLandingNode : public rclcpp::Node
{
public:
    ArucoLandingNode() : Node("aruco_landing_node")
    {
        offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/aruco_visualization", 10, std::bind(&ArucoLandingNode::aruco_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            std::bind(&ArucoLandingNode::odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&ArucoLandingNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Aruco landing node started.");
    }

private:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose aruco_pose_;
    bool aruco_detected_ = false;
    bool armed_ = false;
    bool offboard_enabled_ = false;
    bool mission_started_ = false;
    bool landing_initiated_ = false;
    bool initial_pos_recorded_ = false;

    float current_z_ = 0.0f;
    float current_yaw_ = 0.0f;
    float current_x_ = 0.0f;
    float current_y_ = 0.0f;

    float initial_x_ = 0.0f;
    float initial_y_ = 0.0f;
    float mission_target_x_ = 0.0f;
    float mission_target_y_ = 0.0f;
    float mission_target_z_ = -3.0f;

    float step_ = 0.3f; // 3 m/s * 0.1s

    void aruco_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (!msg->poses.empty()) {
            aruco_pose_ = msg->poses[0];
            aruco_detected_ = true;
        } else {
            aruco_detected_ = false;
        }
    }

    void odom_callback(const VehicleOdometry::SharedPtr msg)
    {
        current_x_ = msg->position[0];
        current_y_ = msg->position[1];
        current_z_ = msg->position[2];

        float q0 = msg->q[0], q1 = msg->q[1], q2 = msg->q[2], q3 = msg->q[3];
        current_yaw_ = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                                  1.0f - 2.0f * (q2 * q2 + q3 * q3));

        if (!initial_pos_recorded_) {
            initial_x_ = current_x_;
            initial_y_ = current_y_;
            mission_target_x_ = initial_x_ + 3.0f;  // Move 3 meters forward from initial
            mission_target_y_ = initial_y_;
            initial_pos_recorded_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial pos recorded: x=%.2f y=%.2f z=%.2f", initial_x_, initial_y_, current_z_);
        }
    }

    void control_loop()
    {
        publish_offboard_control_mode();

        if (!offboard_enabled_) {
            for (int i = 0; i < 10; ++i) publish_offboard_control_mode();
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            offboard_enabled_ = true;
        }

        if (!armed_) {
            arm();
            armed_ = true;
            return;
        }

        if (!mission_started_) {
            RCLCPP_INFO(this->get_logger(), "Taking off to 3m...");
            publish_trajectory_setpoint(current_x_, current_y_, -3.0f, current_yaw_);
            if (std::abs(current_z_ + 3.0f) < 0.2f) mission_started_ = true;
            return;
        }

        if (landing_initiated_) return;

        if (aruco_detected_) {
            float dx = aruco_pose_.position.x;
            float dy = aruco_pose_.position.y;

            if (std::abs(dx) < 0.05 && std::abs(dy) < 0.05) {
                RCLCPP_INFO(this->get_logger(), "ArUco centered. Landing...");
                land();
                landing_initiated_ = true;
                return;
            } else {
                float adjust_x = current_x_ + std::clamp(dx, -0.15f, 0.15f);
                float adjust_y = current_y_ + std::clamp(dy, -0.15f, 0.15f);
                publish_trajectory_setpoint(adjust_x, adjust_y, -3.0f, current_yaw_);
                RCLCPP_INFO(this->get_logger(), "Correcting to ArUco x=%.2f y=%.2f", dx, dy);
            }
        } else {
            float dx = mission_target_x_ - current_x_;
            float dy = mission_target_y_ - current_y_;
            float dist = std::hypot(dx, dy);

            if (dist < step_) {
                RCLCPP_INFO(this->get_logger(), "No ArUco. Landing at end point...");
                land();
                landing_initiated_ = true;
            } else {
                float direction_x = dx / dist;
                float direction_y = dy / dist;
                publish_trajectory_setpoint(current_x_ + direction_x * step_,
                                            current_y_ + direction_y * step_,
                                            mission_target_z_, current_yaw_);
                RCLCPP_INFO(this->get_logger(), "Moving to B. Current x=%.2f y=%.2f", current_x_, current_y_);
            }
        }
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint(float x, float y, float z, float yaw)
    {
        TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = {x, y, z};
        msg.yaw = yaw;
        trajectory_setpoint_pub_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
    {
        VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
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
        RCLCPP_INFO(this->get_logger(), "Arm command sent.");
    }

    void land()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting ArUco landing node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoLandingNode>());
    rclcpp::shutdown();
    return 0;
}
