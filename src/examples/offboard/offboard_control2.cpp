#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <cmath>

using namespace std::chrono_literals;

struct displacement_based_control {
    double u_x, u_y, u_z;
    double x=0.0, y=0.0, z=0.0;
    double x_lead=0.0, y_lead=0.0, z_lead=0.0;
    double x_des=-2.0, y_des=-2.0, z_des=0.0;
    double x_lead_des=0.0, y_lead_des=0.0, z_lead_des=0.0;
        
    // void displacement_algo() {
    //     double u_x = -(x - x_lead) + (x_des - x_lead_des);
    //     double u_y = -(y - y_lead) + (y_des - y_lead_des);
    //     double u_z = -(z - z_lead) + (z_des - z_lead_des);
    // }

    displacement_based_control displacement_algo() {
    displacement_based_control u;
 
    u.u_x = -(x - x_lead) + (x_des - x_lead_des);
    u.u_y = -(y - y_lead) + (y_des - y_lead_des);
    u.u_z = -(z - z_lead) + (z_des - z_lead_des);
    return u;
}

};


struct collecting_data {
    int initial_samples_ = 0;
    float initial_x_sum_ = 0.0f, initial_y_sum_ = 0.0f, initial_z_sum_ = 0.0f;
    float initial_x_avg_ = 0.0f, initial_y_avg_ = 0.0f, initial_z_avg_ = 0.0f;

    // Pass the values as parameters
    void collect_initial_position(double x, double y, double z) {
        if (initial_samples_ == 0 || (std::abs(x) > 0.01f || std::abs(y) > 0.01f)) {
            initial_x_sum_ += x;
            initial_y_sum_ += y;
            initial_z_sum_ += z;
            initial_samples_++;
        }
    }

    // Pass logger and bool flag as parameters
    void finalize_initial_position(rclcpp::Logger logger, bool& collecting_flag) {
        if (initial_samples_ > 0) {
            initial_x_avg_ = initial_x_sum_ / initial_samples_;
            initial_y_avg_ = initial_y_sum_ / initial_samples_;
            initial_z_avg_ = -3.0f;
            collecting_flag = false;
            
            RCLCPP_INFO(logger, 
                "[INIT] Initial position: x=%.2f, y=%.2f, z=%.2f (samples: %d)", 
                initial_x_avg_, initial_y_avg_, initial_z_avg_, initial_samples_);
        }
    }
};

// struct collecting_data{
//     int initial_samples_ = 0;
//     float initial_x_sum_ = 0.0f, initial_y_sum_ = 0.0f, initial_z_sum_ = 0.0f;
//     float initial_x_avg_ = 0.0f, initial_y_avg_ = 0.0f, initial_z_avg_ = 0.0f;

//     void collect_initial_position()
//     {

//         // use dbc_.x / dbc_.y / dbc_.z (follower actual)
//         if (initial_samples_ == 0 || (std::abs(dbc_.x) > 0.01f || std::abs(dbc_.y) > 0.01f)) {
//             initial_x_sum_ += dbc_.x;
//             initial_y_sum_ += dbc_.y;
//             initial_z_sum_ += dbc_.z;
//             initial_samples_++;
//         }
//     }

//     void finalize_initial_position()
//     {
//         if (initial_samples_ > 0) {
//             initial_x_avg_ = initial_x_sum_ / initial_samples_;
//             initial_y_avg_ = initial_y_sum_ / initial_samples_;
//             // keep z fixed to -3.0
//             initial_z_avg_ = -3.0f;
//             collecting_initial_pos_ = false;
            
//             RCLCPP_INFO(this->get_logger(), 
//                 "[INIT] Initial position: x=%.2f, y=%.2f, z=%.2f (samples: %d)", 
//                 initial_x_avg_, initial_y_avg_, initial_z_avg_, initial_samples_);
//         }
//     }

// };


class DistanceStepDrone2 : public rclcpp::Node
{
public:
    DistanceStepDrone2() : Node("distance_step_drone2")
    {
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
            std::bind(&DistanceStepDrone2::odom_lead_callback, this, std::placeholders::_1));

        odom2_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/px4_2/fmu/out/vehicle_odometry", 
            qos_profile,
            std::bind(&DistanceStepDrone2::odom_own_callback, this, std::placeholders::_1));

        // Timer to send setpoints periodically
        timer_ = create_wall_timer(100ms, std::bind(&DistanceStepDrone2::publish_setpoints, this));
        
        
        counter_ = 0;
        collecting_initial_pos_ = true;
        initial_samples_ = 0;
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
            // collecting_data.collect_initial_position();
            setpoint_x_ = dbc_.x;       // use struct follower's current x
            setpoint_y_ = dbc_.y;
            setpoint_z_ = dbc_.z;

            // setpoint_z_ = -3.0f;
            collecting_data_.collect_initial_position(dbc_.x, dbc_.y, dbc_.z);

            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "[INIT] Collecting initial position... %d/50", counter_);
        }
        // Phase 2: Arm and set offboard mode
        else if (counter_ == 50) {
            collecting_data_.finalize_initial_position(this->get_logger(), collecting_initial_pos_);
            arm();
            set_offboard_mode();
        }
        // Phase 3: Wait after arming (next 30 cycles = 3 seconds)
        else if (counter_ < 80) {
            setpoint_x_ = collecting_data_.initial_x_avg_;
            setpoint_y_ = collecting_data_.initial_y_avg_;
            setpoint_z_ = collecting_data_.initial_z_avg_;
            
            if (counter_ == 79) {
                RCLCPP_INFO(this->get_logger(), "[READY] Starting distance-step control algorithm...");
                // Initialize previous setpoint for the algorithm
                prev_setpoint_x_ = setpoint_x_;
                prev_setpoint_y_ = setpoint_y_;
                prev_setpoint_z_ = setpoint_z_;
            }
        }
        // Phase 4: Normal control - send current setpoint (algorithm may update elsewhere)
        else {
            // call algorithm update when both odoms are present and drone is roughly at safe altitude
            if (drone_lead_data_received_ && dbc_.z <= -2.5) {
                apply_distance_step_algorithm();
            } else {
                // hold previous setpoint
                setpoint_x_ = prev_setpoint_x_;
                setpoint_y_ = prev_setpoint_y_;
                setpoint_z_ = prev_setpoint_z_;

                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "[WAIT] Holding until Drone 2 reaches safe altitude (z=-3m). Current z=%.2f",
                    dbc_.z);
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

    

    void apply_distance_step_algorithm()
    {
        // Check odometry
        if (!drone_own_data_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "[WARNING] Waiting for Drone 1 odometry data. Check if Drone 1 is running!");
            setpoint_x_ = prev_setpoint_x_;
            setpoint_y_ = prev_setpoint_y_;
            setpoint_z_ = prev_setpoint_z_;
            return;
        }
        if (!drone_lead_data_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "[WARNING] Waiting for Drone 2 odometry data");
            setpoint_x_ = prev_setpoint_x_;
            setpoint_y_ = prev_setpoint_y_;
            setpoint_z_ = prev_setpoint_z_;
            return;
        }

        // Compute u using the struct (P1_des assumed zero unless set otherwise)
        auto u = dbc_.displacement_algo();

        setpoint_x_ = prev_setpoint_x_ + u.u_x * step_gain_;
        setpoint_y_ = prev_setpoint_y_ + u.u_y * step_gain_;
        setpoint_z_ = prev_setpoint_z_ + u.u_z * step_gain_;
        
        prev_setpoint_x_ = setpoint_x_;
        prev_setpoint_y_ = setpoint_y_;
        prev_setpoint_z_ = setpoint_z_;

        // // Compute errors for logging
        // double P2_actual_x = dbc_.x - dbc_.x_lead;
        // double P2_actual_y = dbc_.y - dbc_.y_lead;
        // double pos_error_x = P2_actual_x - dbc_.x_des;
        // double pos_error_y = P2_actual_y - dbc_.y_des;
        // double distance_error = std::sqrt(pos_error_x * pos_error_x + pos_error_y * pos_error_y);
                

        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //     "[POSITIONS] D1: [%.2f, %.2f] | D2: [%.2f, %.2f]", 
        //     dbc_.x_lead, dbc_.y_lead, dbc_.x, dbc_.y);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //     "[RELATIVE] Actual: [%.2f, %.2f] | Desired: [%.2f, %.2f]", 
        //     P2_actual_x, P2_actual_y, dbc_.x_des, dbc_.y_des);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //     "[CONTROL] u(k): [%.3f, %.3f, %.3f]", 
        //     u_x, u_y, u_z);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //     "[SETPOINT] P2(k+1): [%.2f, %.2f, %.2f]", 
        //     setpoint_x_, setpoint_y_, setpoint_z_);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        //     "[ERROR] Distance error: %.3f meters", distance_error);
    }

    void odom_own_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        dbc_.x = msg->position[0];
        dbc_.y = msg->position[1];
        dbc_.z = msg->position[2];
        
        if (!drone_own_data_received_) {
            drone_own_data_received_ = true;
            RCLCPP_INFO(this->get_logger(), "[SUCCESS] Drone 1 odometry data received!");
        }
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[ODOM1] Drone 1 position: [%.2f, %.2f, %.2f]", 
            dbc_.x, dbc_.y, dbc_.z);
    }

    void odom_lead_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        dbc_.x_lead = msg->position[0];
        dbc_.y_lead = msg->position[1];
        dbc_.z_lead = msg->position[2];
        
        if (!drone_lead_data_received_) {
            drone_lead_data_received_ = true;
            RCLCPP_INFO(this->get_logger(), "[SUCCESS] Drone 2 odometry data received!");
        }
        
        // Calculate yaw from quaternion
        float q0 = msg->q[0], q1 = msg->q[1], q2 = msg->q[2], q3 = msg->q[3];
        current_yaw_ = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                                  1.0f - 2.0f * (q2 * q2 + q3 * q3));
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[ODOM2] Drone 2 position: [%.2f, %.2f, %.2f], yaw: %.2f", 
            dbc_.x_lead, dbc_.y_lead, dbc_.z_lead, current_yaw_);
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

    displacement_based_control dbc_;
    collecting_data collecting_data_;
    
    int counter_ = 0;
    bool collecting_initial_pos_;
    int initial_samples_ = 0;    
    bool drone_own_data_received_= false;
    bool drone_lead_data_received_ = false;
    
    float step_gain_ = 0.01;
    float prev_setpoint_x_, prev_setpoint_y_, prev_setpoint_z_;
    float setpoint_x_, setpoint_y_, setpoint_z_;
    float current_yaw_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceStepDrone2>());
    rclcpp::shutdown();
    return 0;
}