#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class DistanceStepControl : public rclcpp::Node {
public:
    DistanceStepControl() : Node("distance_step_drone2") {
        // Publishers for drone 2 namespace
        offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/px4_2/fmu/in/offboard_control_mode", 10);

        trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/px4_2/fmu/in/trajectory_setpoint", 10);

        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/px4_2/fmu/in/vehicle_command", 10);

        // Timer to send setpoints periodically
        timer_ = create_wall_timer(100ms, std::bind(&DistanceStepControl::step_control, this));

        counter_ = 0;
        current_step_ = 0;

        // Define a simple distance-step mission: forward 3m, right 3m, back to start
        steps_ = {
            {0.0, 0.0, -3.0},  // Takeoff point (3m above ground)
            {3.0, 0.0, -3.0},  // Move forward
            {3.0, 3.0, -3.0},  // Move right
            {0.0, 3.0, -3.0},  // Move left
            {0.0, 0.0, -3.0}   // Return to start
        };

        RCLCPP_INFO(this->get_logger(), "DistanceStepControl initialized for Drone 2 (target_system=3)");
    }

private:
    void step_control() {
        // Always publish offboard control mode first
        px4_msgs::msg::OffboardControlMode offboard_msg{};
        offboard_msg.position = true;
        offboard_msg.velocity = false;
        offboard_msg.acceleration = false;
        offboard_msg.attitude = false;
        offboard_msg.body_rate = false;
        offboard_control_mode_pub_->publish(offboard_msg);

        // After some cycles, send arm + offboard request
        if (counter_ == 10) {
            arm();
            set_offboard_mode();
        }

        // Publish current step setpoint
        if (current_step_ < steps_.size()) {
            auto target = steps_[current_step_];
            px4_msgs::msg::TrajectorySetpoint traj{};
            traj.position = {target[0], target[1], target[2]};
            traj.yaw = 0.0;
            trajectory_setpoint_pub_->publish(traj);

            if (counter_ % 20 == 0) {  // Print every 2s
                RCLCPP_INFO(this->get_logger(), "Drone 2 → Step %d target: [%.2f, %.2f, %.2f]",
                            current_step_, target[0], target[1], target[2]);
            }
        }

        // Switch step every 100 cycles (~10s at 100ms timer)
        if (counter_ > 0 && counter_ % 100 == 0) {
            current_step_++;
            if (current_step_ >= steps_.size()) {
                RCLCPP_INFO(this->get_logger(), "Mission complete for Drone 2!");
                rclcpp::shutdown();
            } else {
                RCLCPP_INFO(this->get_logger(), "Advancing to Step %d", current_step_);
            }
        }

        counter_++;
    }

    void arm() {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.param1 = 1.0;  // arm
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.target_system = 3;   // Drone 2
        cmd.target_component = 1;
        cmd.source_system = 2;
        cmd.source_component = 1;
        vehicle_command_pub_->publish(cmd);

        RCLCPP_INFO(this->get_logger(), "Sent ARM command to Drone 2");
    }

    void set_offboard_mode() {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1 = 1.0;  // custom
        cmd.param2 = 6.0;  // offboard
        cmd.target_system = 3;   // Drone 2
        cmd.target_component = 1;
        cmd.source_system = 2;
        cmd.source_component = 1;
        vehicle_command_pub_->publish(cmd);

        RCLCPP_INFO(this->get_logger(), "Switched Drone 2 to OFFBOARD mode");
    }

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int counter_;
    size_t current_step_;
    std::vector<std::array<float, 3>> steps_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceStepControl>());
    rclcpp::shutdown();
    return 0;
}
