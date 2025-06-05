/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

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

        // Subscribe to odometry for yaw lock
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
            }
        );

        // Mission waypoints (z is always -3.0)
        waypoints_ = {
            {0.0f, 0.0f, 4.0f}, // Takeoff point
            {3.0f, 0.0f, 4.0f},
            {3.0f, 3.0f, 4.0f},
            {0.0f, 3.0f, 4.0f},
            {0.0f, 0.0f, 4.0f}
        };

        current_x_ = 0.0f;
        current_y_ = 0.0f;
        current_z_ = 0.0f;
        current_yaw_ = std::numeric_limits<float>::quiet_NaN(); // Not set yet
        mission_state_ = TAKEOFF;
        waypoint_idx_ = 0;
        reached_time_ = this->now();

        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void {
            // Arm and switch to offboard after 10 setpoints
            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
            }

            // If yaw is not set yet, do nothing (wait for odometry)
            if (std::isnan(current_yaw_)) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for odometry to lock yaw...");
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
                offboard_setpoint_counter_++;
                return;
            }

            switch (mission_state_) {
                case TAKEOFF:
                    // Takeoff to z = -3.0 at 0.3 m/s
                    if (current_z_ > 4.0f) {
                        current_z_ -= 0.3f * 0.1f; // 0.1s timer
                        if (current_z_ < 4.0f) current_z_ = 4.0f;
                    } else {
                        mission_state_ = HOLD;
                        reached_time_ = this->now();
                        RCLCPP_INFO(this->get_logger(), "Takeoff complete, holding at (%.1f, %.1f, %.1f)", current_x_, current_y_, current_z_);
                    }
                    break;
                case HOLD:
                    // Wait 5 seconds at current waypoint
                    if ((this->now() - reached_time_).seconds() >= 5.0) {
                        if (waypoint_idx_ < waypoints_.size() - 1) {
                            mission_state_ = GOTO;
                            RCLCPP_INFO(this->get_logger(), "Proceeding to waypoint %zu: (%.1f, %.1f, %.1f)",
                                waypoint_idx_ + 1,
                                waypoints_[waypoint_idx_ + 1].x,
                                waypoints_[waypoint_idx_ + 1].y,
                                waypoints_[waypoint_idx_ + 1].z);
                        } else {
                            mission_state_ = LAND;
                            RCLCPP_INFO(this->get_logger(), "Mission complete, landing...");
                        }
                    }
                    break;
                case GOTO: {
                    // Move to next waypoint at 0.4 m/s
                    const Waypoint& target = waypoints_[waypoint_idx_ + 1];
                    float dx = target.x - current_x_;
                    float dy = target.y - current_y_;
                    float dz = target.z - current_z_;
                    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                    float step = 0.2f * 0.1f; // 0.4 m/s * 0.1s

                    if (dist < 0.05f) { // Arrived
                        current_x_ = target.x;
                        current_y_ = target.y;
                        current_z_ = target.z;
                        waypoint_idx_++;
                        mission_state_ = HOLD;
                        reached_time_ = this->now();
                        RCLCPP_INFO(this->get_logger(), "Arrived at waypoint %zu: (%.1f, %.1f, %.1f)",
                            waypoint_idx_,
                            current_x_, current_y_, current_z_);
                    } else {
                        // Move towards target
                        current_x_ += (dx/dist) * std::min(step, dist);
                        current_y_ += (dy/dist) * std::min(step, dist);
                        current_z_ += (dz/dist) * std::min(step, dist);
                    }
                    break;
                }
                case LAND:
                    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0);
                    RCLCPP_INFO_ONCE(this->get_logger(), "Landing command sent.");
                    mission_state_ = DONE;
                    break;
                case DONE:
                    // Do nothing
                    break;
            }

            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            offboard_setpoint_counter_++;
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

private:
    enum MissionState { TAKEOFF, HOLD, GOTO, LAND, DONE };
    MissionState mission_state_;
    std::vector<Waypoint> waypoints_;
    size_t waypoint_idx_;
    rclcpp::Time reached_time_;

    // Publishers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

    // Variables
    uint64_t offboard_setpoint_counter_;
    float current_z_;
    float current_x_;
    float current_y_;
    float current_yaw_;

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        TrajectorySetpoint msg{};
        msg.position = {current_x_, current_y_, current_z_};
        msg.yaw = current_yaw_; // Always use current yaw from odometry
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
        RCLCPP_INFO(this->get_logger(), "Arm command send");
    }

    void disarm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        RCLCPP_INFO(this->get_logger(), "Disarm command send");
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
