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
#include <std_msgs/msg/bool.hpp>


using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// BIKIN STRUCT
struct Waypoint {
    float x, y, z;
};

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control") // method constructor
    {
        // PUBLISHERS
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // SUBSCRIPTIONS
            detection_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/detection",
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    detection_triggered_ = true;
                    RCLCPP_INFO(this->get_logger(), "Detection TRUE → landing!");
                }
            }
        );
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                //quaternion values for orientation, coming from the odometry message.
                float q0 = msg->q[0];
                float q1 = msg->q[1];
                float q2 = msg->q[2];
                float q3 = msg->q[3];
                current_yaw_ = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                                          1.0f - 2.0f * (q2 * q2 + q3 * q3));
		
                if (collecting_initial_pos_) {
                    initial_x_sum_ += msg->position[0]; // ini ngambil posisi awal dari "VehicleOdometry"
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

        // Initialize variables
        mission_state_ = TAKEOFF;
        waypoint_idx_ = 0;
        offboard_setpoint_counter_ = 0;
        initial_pos_samples_ = 0;
        collecting_initial_pos_ = true;
        initial_collect_start_ = this->now();

        //timer_ = this->create_wall_timer(100ms, [this]() {
        timer_ = this->create_wall_timer(100ms, [this]() {
            // --- detection check ---
            if (detection_triggered_ && mission_state_ != DONE) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0);
                RCLCPP_INFO(this->get_logger(), "Landing due to detection.");
                mission_state_ = DONE;
                return;  // stop mission updates
            }
            if (collecting_initial_pos_) {
                if ((this->now() - initial_collect_start_).seconds() < 5.0) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Collecting initial position");
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "YAW = %.2f", current_yaw_);
                    publish_offboard_control_mode(); // void function1
                    publish_trajectory_setpoint(); // void function1
                    offboard_setpoint_counter_++; // aslinya 0
                    return;
                } else {
                    initial_x_avg_ = initial_x_sum_ / std::max(initial_pos_samples_, 1); // ini ngitung rata-rata posisi awal
                    initial_y_avg_ = initial_y_sum_ / std::max(initial_pos_samples_, 1);
                    initial_z_avg_ = initial_z_sum_ / std::max(initial_pos_samples_, 1);
                    collecting_initial_pos_ = false; // stop collecting initial position
                    RCLCPP_INFO(this->get_logger(), "Initial pos: x=%.2f y=%.2f z=%.2f", initial_x_avg_, initial_y_avg_, initial_z_avg_);
                    offboard_setpoint_counter_ = 0;
                }
            }

            if (!collecting_initial_pos_ && offboard_setpoint_counter_ == 10) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                arm();
            }

            if (std::isnan(current_yaw_)) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for yaw");
                publish_offboard_control_mode(); // void function2
                publish_trajectory_setpoint(); // void function2
                offboard_setpoint_counter_++;
                return;
            }

            // VARIABLE DECLARATIONS
            const Waypoint &target_local = waypoints_[waypoint_idx_]; //reference ke waypoint saat ini makanya pake waypoint_idx_
            // Rotate body-relative waypoint to NED using yaw
            float body_x = target_local.x;
            float body_y = target_local.y;
            float target_x = initial_x_avg_ + body_x * std::cos(current_yaw_) - body_y * std::sin(current_yaw_);
            float target_y = initial_y_avg_ + body_x * std::sin(current_yaw_) + body_y * std::cos(current_yaw_);
            float target_z = target_local.z + initial_z_avg_;
            float dx = target_x - current_x_; // harunya makin ngecil dia
            float dy = target_y - current_y_;
            float dz = target_z - current_z_;
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz); // jarak ke waypoint
            float step = 0.35f; // 3 m/s * 0.1s

            if (dist < 0.1f) {
                waypoint_idx_++;
                if (waypoint_idx_ >= waypoints_.size()) { // udah  sampai waypoint terakhir
                    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0); // void function
                    RCLCPP_INFO(this->get_logger(), "Landing initiated.");
                    mission_state_ = DONE;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", waypoint_idx_);
                }
            } else {
                setpoint_x_= current_x_ + (dx / dist) * std::min(step, dist);
                setpoint_y_= current_y_ + (dy / dist) * std::min(step, dist);
                setpoint_z_= current_z_ + (dz / dist) * std::min(step, dist);
            }

            publish_offboard_control_mode(); // void function3
            publish_trajectory_setpoint(); // void function3
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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detection_sub_;


    uint64_t offboard_setpoint_counter_;
    float current_x_, current_y_, current_z_;
    float current_yaw_;
    float setpoint_x_, setpoint_y_, setpoint_z_;
    bool detection_triggered_ = false;
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
        msg.position = {setpoint_x_, setpoint_y_, setpoint_z_};
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
