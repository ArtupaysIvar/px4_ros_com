#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>
#include <limits>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

struct Waypoint {
    float x, y, z;
};

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                float q0 = msg->q[0];
                float q1 = msg->q[1];
                float q2 = msg->q[2];
                float q3 = msg->q[3];
                current_yaw_ = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                                          1.0f - 2.0f * (q2 * q2 + q3 * q3));

                if (collecting_initial_pos_) {
                    initial_x_sum_ += msg->position[0];
                    initial_y_sum_ += msg->position[1];
                    initial_z_sum_ += msg->position[2];
                    initial_pos_samples_++;
                }

                current_x_ = msg->position[0];
                current_y_ = msg->position[1];
                current_z_ = msg->position[2];
            }
        );

        // Local square trajectory
        waypoints_ = {
            {0.0f, 0.0f, -3.0f},
            {3.0f, 0.0f, -3.0f},
            {3.0f, 3.0f, -3.0f},
            {0.0f, 3.0f, -3.0f},
            {0.0f, 0.0f, -3.0f}
        };

        mission_state_ = TAKEOFF;
        waypoint_idx_ = 0;
        offboard_setpoint_counter_ = 0;
        initial_pos_samples_ = 0;
        collecting_initial_pos_ = true;
        initial_collect_start_ = this->now();

        timer_ = this->create_wall_timer(100ms, [this]() {
            if (collecting_initial_pos_) {
                if ((this->now() - initial_collect_start_).seconds() < 2.0) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Collecting initial position...");
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint();
                    offboard_setpoint_counter_++;
                    return;
                } else {
                    initial_x_avg_ = initial_x_sum_ / std::max(initial_pos_samples_, 1);
                    initial_y_avg_ = initial_y_sum_ / std::max(initial_pos_samples_, 1);
                    initial_z_avg_ = initial_z_sum_ / std::max(initial_pos_samples_, 1);
                    collecting_initial_pos_ = false;
                    RCLCPP_INFO(this->get_logger(), "Initial pos: x=%.2f y=%.2f z=%.2f", initial_x_avg_, initial_y_avg_, initial_z_avg_);
                    offboard_setpoint_counter_ = 0;
                }
            }

            if (!collecting_initial_pos_ && offboard_setpoint_counter_ == 10) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                arm();
            }

            if (std::isnan(current_yaw_)) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for yaw...");
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
                offboard_setpoint_counter_++;
                return;
            }

            const Waypoint &target_local = waypoints_[waypoint_idx_];
            float target_x = target_local.x + initial_x_avg_;
            float target_y = target_local.y + initial_y_avg_;
            float target_z = target_local.z + initial_z_avg_;

            float dx = target_x - current_x_;
            float dy = target_y - current_y_;
            float dz = target_z - current_z_;
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            float step = 0.3f; // 3 m/s * 0.1s
            if (dist < 0.1f) {
                waypoint_idx_++;
                if (waypoint_idx_ >= waypoints_.size()) {
                    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0);
                    RCLCPP_INFO(this->get_logger(), "Landing initiated.");
                    mission_state_ = DONE;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", waypoint_idx_);
                }
            } else {
                current_x_ += (dx / dist) * std::min(step, dist);
                current_y_ += (dy / dist) * std::min(step, dist);
                current_z_ += (dz / dist) * std::min(step, dist);
            }

            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            offboard_setpoint_counter_++;
        });
    }

private:
    enum MissionState { TAKEOFF, DONE };
    MissionState mission_state_;
    std::vector<Waypoint> waypoints_;
    size_t waypoint_idx_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

    uint64_t offboard_setpoint_counter_;
    float current_x_, current_y_, current_z_;
    float current_yaw_;

    bool collecting_initial_pos_;
    int initial_pos_samples_;
    float initial_x_sum_, initial_y_sum_, initial_z_sum_;
    float initial_x_avg_, initial_y_avg_, initial_z_avg_;
    rclcpp::Time initial_collect_start_;

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        TrajectorySetpoint msg{};
        msg.position = {current_x_, current_y_, current_z_};
        msg.yaw = current_yaw_;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Arm command sent.");
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
