#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class SquareMission : public rclcpp::Node {
public:
    SquareMission() : Node("square_mission_drone1") {
        
        offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/px4_1/fmu/in/offboard_control_mode", 10);

        trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/px4_1/fmu/in/trajectory_setpoint", 10);

        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/px4_1/fmu/in/vehicle_command", 10);

        // Timer for periodic publishing
        timer_ = create_wall_timer(100ms, std::bind(&SquareMission::publish_setpoints, this));

        counter_ = 0;
        waypoint_index_ = 0;

        // Define waypoints (square at z = -3 → 3m above ground)
        waypoints_ = {
        {0.0, 0.0, -3.0},
        {20.0, 0.0, -3.0}
        };
    }

private:
   void publish_setpoints() {
    // Always publish offboard control mode
    px4_msgs::msg::OffboardControlMode offboard_msg{};
    offboard_msg.position = true;
    offboard_msg.velocity = false;
    offboard_msg.acceleration = false;
    offboard_msg.attitude = false;
    offboard_msg.body_rate = false;
    offboard_control_mode_pub_->publish(offboard_msg);

    // Arm + set offboard after a short startup (keep your original timing)
    if (counter_ == 10) {
        arm();
        set_offboard_mode();
    }

    // If we've exhausted waypoints, keep publishing the last one and return
    if (waypoint_index_ >= waypoints_.size()) {
        auto last = waypoints_.back();
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.position = {last[0], last[1], last[2]};
        traj.yaw = 0.0;
        trajectory_setpoint_pub_->publish(traj);
        counter_++;
        return;
    }

    // If not currently holding, start a hold for the current waypoint.
    // For waypoint 0 use 500 seconds; otherwise use 5 seconds.
    if (!holding_) {
        holding_ = true;
        hold_counter_ = 0;
        // 100 ms timer => 10 ticks per second
        if (waypoint_index_ == 0) {
            hold_duration_ticks_ = 50 * 10; // 500 s
        } else {
            hold_duration_ticks_ = 20 * 10;   // 5 s
        }
    }

    // Always publish the current waypoint while holding
    auto wp = waypoints_[waypoint_index_];
    px4_msgs::msg::TrajectorySetpoint traj{};
    traj.position = { wp[0], wp[1], wp[2] };
    traj.yaw = 0.0;
    trajectory_setpoint_pub_->publish(traj);

    // Count hold ticks
    hold_counter_++;

    // If hold finished, move to next waypoint (if any) and reset holding state
    if (hold_counter_ >= hold_duration_ticks_) {
        holding_ = false;
        hold_counter_ = 0;
        hold_duration_ticks_ = 0;

        // advance to next waypoint if available
        if (waypoint_index_ < waypoints_.size() - 1) {
            waypoint_index_++;
            // Next waypoint will start its hold at the top of the next timer tick
        } else {
            // reached final waypoint: we will keep publishing it (see top)
            waypoint_index_ = waypoints_.size(); // mark finished
        }
    }

    counter_++;
}


    void arm() {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.param1 = 1.0;  // arm
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.target_system = 2;     
        cmd.target_component = 1;
        cmd.source_system = 2;
        cmd.source_component = 1;
        vehicle_command_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }

    void set_offboard_mode() {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1 = 1.0;  // custom
        cmd.param2 = 6.0;  // offboard
        cmd.target_system = 2;
        cmd.target_component = 1;
        cmd.source_system = 2;
        cmd.source_component = 1;
        vehicle_command_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
    }

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int counter_;
    size_t waypoint_index_;
    std::vector<std::array<float, 3>> waypoints_;
    bool holding_ = false;
    int hold_counter_ = 0;
    int hold_duration_ticks_ = 0; 
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareMission>());
    rclcpp::shutdown();
    return 0;
}