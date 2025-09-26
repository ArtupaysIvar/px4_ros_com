#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class SimpleTakeoff : public rclcpp::Node {
public:
    SimpleTakeoff() : Node("simple_takeoff_drone2") {
        // Publishers for drone 2 namespace
        offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/px4_2/fmu/in/offboard_control_mode", 10);

        trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/px4_2/fmu/in/trajectory_setpoint", 10);

        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/px4_2/fmu/in/vehicle_command", 10);

        // Timer to send setpoints periodically
        timer_ = create_wall_timer(100ms, std::bind(&SimpleTakeoff::publish_setpoints, this));

        counter_ = 0;
    }

private:
    void publish_setpoints() {
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

        // Send position setpoint (x=0, y=0, z=-3 → 3m above ground)
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.position = {0.0, 0.0, -3.0};  
        traj.yaw = 0.0;
        trajectory_setpoint_pub_->publish(traj);

        counter_++;
    }

    void arm() {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.param1 = 1.0;  // arm
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.target_system = 3;      // Drone 2
        cmd.target_component = 1;
        cmd.source_system = 2;
        cmd.source_component = 1;
        vehicle_command_pub_->publish(cmd);
    }

    void set_offboard_mode() {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1 = 1.0;  // custom
        cmd.param2 = 6.0;  // offboard
        cmd.target_system = 3;
        cmd.target_component = 1;
        cmd.source_system = 2;
        cmd.source_component = 1;
        vehicle_command_pub_->publish(cmd);
    }

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTakeoff>());
    rclcpp::shutdown();
    return 0;
}
