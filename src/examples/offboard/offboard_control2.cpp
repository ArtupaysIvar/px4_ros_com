#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <cmath>

using namespace std::chrono_literals;

class DistanceStepDrone2 : public rclcpp::Node
{
public:
    DistanceStepDrone2() : Node("distance_step_drone2")
    {
        // Publishers for drone 2 namespace
        offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/px4_2/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/px4_2/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/px4_2/fmu/in/vehicle_command", 10);

        // Subscribers for drone odometry with RELIABLE QoS (matching PX4 publisher)
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();

        odom1_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/px4_1/fmu/out/vehicle_odometry", 
            qos_profile,
            std::bind(&DistanceStepDrone2::odom1_callback, this, std::placeholders::_1));

        odom2_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/px4_2/fmu/out/vehicle_odometry", 
            qos_profile,
            std::bind(&DistanceStepDrone2::odom2_callback, this, std::placeholders::_1));

        // Timer to send setpoints periodically
        timer_ = create_wall_timer(100ms, std::bind(&DistanceStepDrone2::publish_setpoints, this));
        
        // Initialize variables
        counter_ = 0;
        collecting_initial_pos_ = true;
        initial_samples_ = 0;
        
        // Distance-step control parameters
        step_gain_ = 0.1f;  // Control gain
        
        // Desired relative position from Drone 1 to Drone 2
        // P2_des = [x_des, y_des] - where drone 2 should be relative to drone 1
        desired_rel_x_ = 0.0f;   // 0m offset in x
        desired_rel_y_ = -2.0f;  // 2m behind drone 1 (negative y)
        desired_rel_z_ = 0.0f;   // Same altitude
        
        // Initialize positions
        drone1_x_ = 0.0f; drone1_y_ = 0.0f; drone1_z_ = 0.0f;
        drone2_x_ = 0.0f; drone2_y_ = 0.0f; drone2_z_ = 0.0f;
        drone1_data_received_ = false;
        drone2_data_received_ = false;
        setpoint_x_ = 0.0f; setpoint_y_ = 0.0f; setpoint_z_ = -3.0f;
        prev_setpoint_x_ = 0.0f; prev_setpoint_y_ = 0.0f; prev_setpoint_z_ = -3.0f;
        current_yaw_ = 0.0f;
        
        // Control input variables
        u2_x_ = 0.0f; u2_y_ = 0.0f; u2_z_ = 0.0f;
        
        // Initial position collection
        initial_x_sum_ = 0.0f; initial_y_sum_ = 0.0f; initial_z_sum_ = 0.0f;
        initial_x_avg_ = 0.0f; initial_y_avg_ = 0.0f; initial_z_avg_ = -3.0f;
        
        // RCLCPP_INFO(this->get_logger(), "=== DISTANCE-STEP CONTROL DRONE 2 STARTED ===");
        // RCLCPP_INFO(this->get_logger(), "Theory: P2(k+1) = P2(k) + u2(k)");
        // RCLCPP_INFO(this->get_logger(), "Where: u2(k) = -(P2(k) - P1(k)) + (P2_des - P1_des)");
        // RCLCPP_INFO(this->get_logger(), "Desired relative position: [%.1f, %.1f, %.1f]", 
        //     desired_rel_x_, desired_rel_y_, desired_rel_z_);
        // RCLCPP_INFO(this->get_logger(), "Subscribing to: /px4_1/fmu/out/vehicle_odometry");
        // RCLCPP_INFO(this->get_logger(), "Subscribing to: /px4_2/fmu/out/vehicle_odometry");
    }

private:
    void publish_setpoints()
    {
        // Always publish offboard control mode first
        px4_msgs::msg::OffboardControlMode offboard_msg{};
        offboard_msg.position = true;
        offboard_msg.velocity = false;
        offboard_msg.acceleration = false;
        offboard_msg.attitude = false;
        offboard_msg.body_rate = false;
        offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(offboard_msg);

        // Phase 1: Collect initial position (first 50 cycles = 5 seconds)
        if (counter_ < 50 && collecting_initial_pos_) {
            collect_initial_position();
            setpoint_x_ = drone2_x_;
            setpoint_y_ = drone2_y_;
            setpoint_z_ = -3.0f;
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "[INIT] Collecting initial position... %d/50", counter_);
        }
        // Phase 2: Arm and set offboard mode
        else if (counter_ == 50) {
            finalize_initial_position();
            arm();
            set_offboard_mode();
        }
        // Phase 3: Wait after arming (next 30 cycles = 3 seconds)
        else if (counter_ < 80) {
            setpoint_x_ = initial_x_avg_;
            setpoint_y_ = initial_y_avg_;
            setpoint_z_ = initial_z_avg_;
            
            if (counter_ == 79) {
                RCLCPP_INFO(this->get_logger(), "[READY] Starting distance-step control algorithm...");
                // Initialize previous setpoint for the algorithm
                prev_setpoint_x_ = setpoint_x_;
                prev_setpoint_y_ = setpoint_y_;
                prev_setpoint_z_ = setpoint_z_;
            }
        }
        // Phase 4: Distance-step control algorithm
        else {
            if (drone2_data_received_ && drone2_z_ <= -2.5f) {  
                apply_distance_step_algorithm();
            } else {
                // Hold position at initial setpoint until altitude condition is met
                setpoint_x_ = prev_setpoint_x_;
                setpoint_y_ = prev_setpoint_y_;
                setpoint_z_ = initial_z_avg_;

                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "[WAIT] Holding until Drone 2 reaches safe altitude (z=-3m). Current z=%.2f",
                    drone2_z_);
            }
        }
        // Send position setpoint
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.position = {setpoint_x_, setpoint_y_, setpoint_z_};
        traj.yaw = 0.0f;  // Face forward like drone 1
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_pub_->publish(traj);

        counter_++;
    }

    void collect_initial_position()
    {
        if (initial_samples_ == 0 || 
            (std::abs(drone2_x_) > 0.01f || std::abs(drone2_y_) > 0.01f)) {
            initial_x_sum_ += drone2_x_;
            initial_y_sum_ += drone2_y_;
            initial_z_sum_ += drone2_z_;
            initial_samples_++;
        }
    }

    void finalize_initial_position()
    {
        if (initial_samples_ > 0) {
            initial_x_avg_ = initial_x_sum_ / initial_samples_;
            initial_y_avg_ = initial_y_sum_ / initial_samples_;
            // initial_z_avg_ = std::max(initial_z_sum_ / initial_samples_, -3.0f);
            initial_z_avg_ = -3.0f;
            collecting_initial_pos_ = false;
            
            RCLCPP_INFO(this->get_logger(), 
                "[INIT] Initial position: x=%.2f, y=%.2f, z=%.2f (samples: %d)", 
                initial_x_avg_, initial_y_avg_, initial_z_avg_, initial_samples_);
        }
    }

    void apply_distance_step_algorithm()
    {
        // Check if we have valid odometry data from both drones
        if (!drone1_data_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "[WARNING] Waiting for Drone 1 odometry data. Check if Drone 1 is running!");
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[INFO] Expected topic: /px4_1/fmu/out/vehicle_odometry");
            setpoint_x_ = prev_setpoint_x_;
            setpoint_y_ = prev_setpoint_y_;
            setpoint_z_ = prev_setpoint_z_;
            return;
        }
        
        if (!drone2_data_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "[WARNING] Waiting for Drone 2 odometry data");
            setpoint_x_ = prev_setpoint_x_;
            setpoint_y_ = prev_setpoint_y_;
            setpoint_z_ = prev_setpoint_z_;
            return;
        }

        float P2_actual_x = drone2_x_ - drone1_x_; 
        float P2_actual_y = drone2_y_ - drone1_y_;
        float P2_actual_z = drone2_z_ - drone1_z_;
        
        // P1_des is [0,0,0] (drone 1 position is the reference)
        // P2_des - P1_des = P2_des - [0,0,0] = P2_des
        float P2_des_x = desired_rel_x_;
        float P2_des_y = desired_rel_y_;
        float P2_des_z = desired_rel_z_;
        
        // Calculate control input: u2(k) = -(P2_actual) + (P2_des)
        u2_x_ = -(P2_actual_x) + P2_des_x;
        u2_y_ = -(P2_actual_y) + P2_des_y;
        u2_z_ = -(P2_actual_z) + P2_des_z;
        
        // Apply gain to control input
        u2_x_ *= step_gain_;
        u2_y_ *= step_gain_;
        u2_z_ *= step_gain_;
        
        // Update setpoint: P2(k+1) = P2(k) + u2(k)
        // P2(k) is the previous setpoint we sent
        setpoint_x_ = prev_setpoint_x_ + std::clamp(u2_x_, -0.2f, 0.2f);
        setpoint_y_ = prev_setpoint_y_ + std::clamp(u2_y_, -0.2f, 0.2f);
        setpoint_z_ = initial_z_avg_ + u2_z_;  // Maintain altitude reference
        
        // Store current setpoint for next iteration
        prev_setpoint_x_ = setpoint_x_;
        prev_setpoint_y_ = setpoint_y_;
        prev_setpoint_z_ = setpoint_z_;
        
        // Calculate position error for monitoring
        float pos_error_x = P2_actual_x - P2_des_x;
        float pos_error_y = P2_actual_y - P2_des_y;
        float distance_error = std::sqrt(pos_error_x * pos_error_x + pos_error_y * pos_error_y);
        
        // Debug messages every 1 second
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[POSITIONS] D1: [%.2f, %.2f] | D2: [%.2f, %.2f]", 
            drone1_x_, drone1_y_, drone2_x_, drone2_y_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[RELATIVE] Actual: [%.2f, %.2f] | Desired: [%.2f, %.2f]", 
            P2_actual_x, P2_actual_y, P2_des_x, P2_des_y);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[CONTROL] u2(k): [%.3f, %.3f, %.3f]", 
            u2_x_, u2_y_, u2_z_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[SETPOINT] P2(k+1): [%.2f, %.2f, %.2f]", 
            setpoint_x_, setpoint_y_, setpoint_z_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[ERROR] Distance error: %.3f meters", distance_error);
    }

    void odom1_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        drone1_x_ = msg->position[0];
        drone1_y_ = msg->position[1];
        drone1_z_ = msg->position[2];
        
        if (!drone1_data_received_) {
            drone1_data_received_ = true;
            RCLCPP_INFO(this->get_logger(), "[SUCCESS] Drone 1 odometry data received!");
        }
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[ODOM1] Drone 1 position: [%.2f, %.2f, %.2f]", 
            drone1_x_, drone1_y_, drone1_z_);
    }

    void odom2_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        drone2_x_ = msg->position[0];
        drone2_y_ = msg->position[1];
        drone2_z_ = msg->position[2];
        
        if (!drone2_data_received_) {
            drone2_data_received_ = true;
            RCLCPP_INFO(this->get_logger(), "[SUCCESS] Drone 2 odometry data received!");
        }
        
        // Calculate yaw from quaternion
        float q0 = msg->q[0], q1 = msg->q[1], q2 = msg->q[2], q3 = msg->q[3];
        current_yaw_ = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                                  1.0f - 2.0f * (q2 * q2 + q3 * q3));
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[ODOM2] Drone 2 position: [%.2f, %.2f, %.2f], yaw: %.2f", 
            drone2_x_, drone2_y_, drone2_z_, current_yaw_);
    }

    void arm()
    {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.param1 = 1.0; // arm
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.target_system = 3; // Drone 2
        cmd.target_component = 1;
        cmd.source_system = 2;
        cmd.source_component = 1;
        cmd.from_external = true;
        cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(cmd);
        
        RCLCPP_INFO(this->get_logger(), "[ARM] Arm command sent to Drone 2");
    }

    void set_offboard_mode()
    {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1 = 1.0; // custom
        cmd.param2 = 6.0; // offboard
        cmd.target_system = 3; // Drone 2
        cmd.target_component = 1;
        cmd.source_system = 2;
        cmd.source_component = 1;
        cmd.from_external = true;
        cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(cmd);
        
        RCLCPP_INFO(this->get_logger(), "[MODE] Offboard mode command sent to Drone 2");
    }

    // Publishers and subscribers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom1_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom2_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Control variables
    int counter_;
    bool collecting_initial_pos_;
    int initial_samples_;
    
    // Distance-step algorithm parameters
    float step_gain_;
    float desired_rel_x_, desired_rel_y_, desired_rel_z_;  // P2_des
    
    // Position tracking
    float drone1_x_, drone1_y_, drone1_z_;  // P1(k) - drone 1 current position
    float drone2_x_, drone2_y_, drone2_z_;  // Drone 2 current position (for comparison)
    bool drone1_data_received_, drone2_data_received_;  // Data received flags
    float setpoint_x_, setpoint_y_, setpoint_z_;  // P2(k+1) - next setpoint
    float prev_setpoint_x_, prev_setpoint_y_, prev_setpoint_z_;  // P2(k) - previous setpoint
    float current_yaw_;
    
    // Control input
    float u2_x_, u2_y_, u2_z_;  // u2(k)
    
    // Initial position averaging
    float initial_x_sum_, initial_y_sum_, initial_z_sum_;
    float initial_x_avg_, initial_y_avg_, initial_z_avg_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceStepDrone2>());
    rclcpp::shutdown();
    return 0;
}