// claude
/*
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

struct Position {
    float x, y, z;
    Position(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {}
};

class DistanceStepControl : public rclcpp::Node
{
public:
    DistanceStepControl() : Node("distance_step_control")
    {
        // Initialize state variables
        mission_state_ = INITIALIZING;
        offboard_setpoint_counter_ = 0;
        collecting_initial_pos_ = true;
        initial_collect_start_ = this->now();
        
        // Control parameters
        step_size_ = 0.35f;
        desired_distance_ = 2.0f;  // Desired distance between drones
        position_tolerance_ = 0.1f;
        
        // Initialize positions
        drone1_pos_ = Position();
        drone2_pos_ = Position();
        initial_pos_ = Position();
        setpoint_ = Position();
        
        // Publishers
        offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // Subscribers
        odom1_sub_ = this->create_subscription<VehicleOdometry>(
            "px4_1/fmu/out/vehicle_odometry",
            rclcpp::QoS(10).best_effort(),
            [this](const VehicleOdometry::SharedPtr msg) {
                update_drone1_odometry(msg);
            }
        );

        odom2_sub_ = this->create_subscription<VehicleOdometry>(
            "px4_2/fmu/out/vehicle_odometry", 
            rclcpp::QoS(10).best_effort(),
            [this](const VehicleOdometry::SharedPtr msg) {
                update_drone2_odometry(msg);
            }
        );

        // Main control timer (10Hz)
        timer_ = this->create_wall_timer(100ms, [this]() {
            control_loop();
        });

        RCLCPP_INFO(this->get_logger(), "Distance Step Control Node Started");
    }

private:
    enum MissionState { 
        INITIALIZING, 
        READY_TO_ARM, 
        ARMED, 
        DISTANCE_CONTROL, 
        LANDING, 
        DONE 
    };

    // State variables
    MissionState mission_state_;
    uint64_t offboard_setpoint_counter_;
    bool collecting_initial_pos_;
    rclcpp::Time initial_collect_start_;
    
    // Control parameters
    float step_size_;
    float desired_distance_;
    float position_tolerance_;
    
    // Position tracking
    Position drone1_pos_;
    Position drone2_pos_;
    Position initial_pos_;
    Position setpoint_;
    float current_yaw_;
    
    // Initial position averaging
    Position initial_sum_;
    int initial_samples_;
    
    // ROS2 components
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom1_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom2_sub_;

    void update_drone1_odometry(const VehicleOdometry::SharedPtr msg)
    {
        drone1_pos_.x = msg->position[0];
        drone1_pos_.y = msg->position[1];
        drone1_pos_.z = msg->position[2];
        
        // Calculate yaw from quaternion
        float q0 = msg->q[0], q1 = msg->q[1], q2 = msg->q[2], q3 = msg->q[3];
        current_yaw_ = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                                  1.0f - 2.0f * (q2 * q2 + q3 * q3));
        
        // Collect initial position data
        if (collecting_initial_pos_) {
            initial_sum_.x += msg->position[0];
            initial_sum_.y += msg->position[1]; 
            initial_sum_.z += msg->position[2];
            initial_samples_++;
        }
    }

    void update_drone2_odometry(const VehicleOdometry::SharedPtr msg)
    {
        drone2_pos_.x = msg->position[0];
        drone2_pos_.y = msg->position[1];
        drone2_pos_.z = msg->position[2];
    }

    void control_loop()
    {
        switch (mission_state_) {
            case INITIALIZING:
                handle_initialization();
                break;
            case READY_TO_ARM:
                handle_arming();
                break;
            case ARMED:
                handle_armed_state();
                break;
            case DISTANCE_CONTROL:
                handle_distance_control();
                break;
            case LANDING:
                handle_landing();
                break;
            case DONE:
                // Mission complete
                break;
        }
        
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
        offboard_setpoint_counter_++;
    }

    void handle_initialization()
    {
        double elapsed_time = (this->now() - initial_collect_start_).seconds();
        
        if (elapsed_time < 5.0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Collecting initial position... %.1fs", elapsed_time);
            
            // Set initial setpoint to current position
            setpoint_ = drone1_pos_;
        } else {
            // Calculate average initial position
            if (initial_samples_ > 0) {
                initial_pos_.x = initial_sum_.x / initial_samples_;
                initial_pos_.y = initial_sum_.y / initial_samples_;
                initial_pos_.z = initial_sum_.z / initial_samples_;
                
                collecting_initial_pos_ = false;
                mission_state_ = READY_TO_ARM;
                
                RCLCPP_INFO(this->get_logger(), 
                    "Initial position collected: x=%.2f, y=%.2f, z=%.2f", 
                    initial_pos_.x, initial_pos_.y, initial_pos_.z);
            }
        }
    }

    void handle_arming()
    {
        if (offboard_setpoint_counter_ >= 10) {
            // Switch to offboard mode and arm
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm_drone();
            mission_state_ = ARMED;
            RCLCPP_INFO(this->get_logger(), "Switching to offboard mode and arming");
        }
        
        // Keep publishing setpoints at initial position
        setpoint_ = initial_pos_;
    }

    void handle_armed_state()
    {
        // Wait a bit after arming, then start distance control
        if (offboard_setpoint_counter_ > 30) {  // ~3 seconds at 10Hz
            mission_state_ = DISTANCE_CONTROL;
            RCLCPP_INFO(this->get_logger(), "Starting distance control");
        }
        
        // Maintain initial position
        setpoint_ = initial_pos_;
    }

    void handle_distance_control()
    {
        // Check if we have valid yaw data
        if (std::isnan(current_yaw_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Waiting for valid yaw data");
            return;
        }

        // Calculate current distance between drones
        float dx = drone2_pos_.x - drone1_pos_.x;
        float dy = drone2_pos_.y - drone1_pos_.y;
        float current_distance = std::sqrt(dx * dx + dy * dy);
        
        // Calculate distance error
        float distance_error = current_distance - desired_distance_;
        
        // Calculate control step
        if (std::abs(distance_error) > position_tolerance_) {
            // Move towards desired relative position
            float control_x = -dx * step_size_;  // Move opposite to distance vector
            float control_y = -dy * step_size_;
            
            // Apply step control
            setpoint_.x = drone1_pos_.x + control_x;
            setpoint_.y = drone1_pos_.y + control_y;
            setpoint_.z = initial_pos_.z;  // Maintain initial altitude
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Distance: %.2f, Error: %.2f, Setpoint: [%.2f, %.2f, %.2f]",
                current_distance, distance_error, setpoint_.x, setpoint_.y, setpoint_.z);
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Maintaining desired distance: %.2f", current_distance);
        }
        
        // Check for landing condition (you can modify this)
        // For now, let's land after maintaining distance for a while
        static int stable_counter = 0;
        if (std::abs(distance_error) <= position_tolerance_) {
            stable_counter++;
            if (stable_counter > 200) {  // ~20 seconds at 10Hz
                mission_state_ = LANDING;
                RCLCPP_INFO(this->get_logger(), "Distance maintained, initiating landing");
            }
        } else {
            stable_counter = 0;
        }
    }

    void handle_landing()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0);
        mission_state_ = DONE;
        RCLCPP_INFO(this->get_logger(), "Landing command sent");
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        TrajectorySetpoint msg{};
        msg.position = {setpoint_.x, setpoint_.y, setpoint_.z};
        msg.yaw = current_yaw_;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_pub_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
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
        vehicle_command_pub_->publish(msg);
    }

    void arm_drone()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting Distance Step Control Node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceStepControl>());
    rclcpp::shutdown();
    
    return 0;
}
*/

//GPT

// offboard_distance_step.cpp
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

struct Waypoint {
    float x;
    float y;
    float z;
};

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl()
    : Node("offboard_distance_step"),
      offboard_setpoint_counter_(0),
      collecting_initial_pos_(true),
      initial_pos_samples_(0),
      initial_x_sum_(0.0f),
      initial_y_sum_(0.0f),
      initial_z_sum_(0.0f),
      current_x_(NAN),
      current_y_(NAN),
      current_z_(NAN),
      current_yaw_(NAN)
    {
        // params
        this->declare_parameter<std::string>("odom_topic", "px4_1/fmu/out/vehicle_odometry");
        this->declare_parameter<double>("control_rate", 10.0); // Hz
        this->declare_parameter<double>("initial_collect_time", 5.0); // seconds
        this->declare_parameter<double>("step_size", 0.35); // meters per tick
        this->declare_parameter<double>("reach_tol", 0.10); // meters
        this->declare_parameter<std::vector<double>>("waypoint0", std::vector<double>{3.0, 0.0, -3.0});

        odom_topic_ = this->get_parameter("odom_topic").as_string();
        double control_rate = this->get_parameter("control_rate").as_double();
        initial_collect_time_ = this->get_parameter("initial_collect_time").as_double();
        step_size_ = this->get_parameter("step_size").as_double();
        reach_tol_ = this->get_parameter("reach_tol").as_double();

        std::vector<double> wp0 = this->get_parameter("waypoint0").as_double_array();
        Waypoint w; w.x = static_cast<float>(wp0[0]); w.y = static_cast<float>(wp0[1]); w.z = static_cast<float>(wp0[2]);
        waypoints_.push_back(w);

        // publishers
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // subscription
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            odom_topic_, rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                // store position
                current_x_ = msg->position[0];
                current_y_ = msg->position[1];
                current_z_ = msg->position[2];

                // extract yaw from quaternion (q = [w, x, y, z] typical)
                float q0 = msg->q[0];
                float q1 = msg->q[1];
                float q2 = msg->q[2];
                float q3 = msg->q[3];
                current_yaw_ = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                                          1.0f - 2.0f * (q2 * q2 + q3 * q3));

                // accumulate initial position for averaging
                if (collecting_initial_pos_) {
                    initial_x_sum_ += current_x_;
                    initial_y_sum_ += current_y_;
                    initial_z_sum_ += current_z_;
                    initial_pos_samples_++;
                }
            }
        );

        // timer
        auto period = std::chrono::duration<double>(1.0 / control_rate);
        initial_collect_start_ = this->now();

        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            [this]() { control_loop(); }
        );

        // initial setpoints
        setpoint_x_ = 0.0f;
        setpoint_y_ = 0.0f;
        setpoint_z_ = 0.0f;
        waypoint_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "OffboardDistanceStep node started. Listening to: %s", odom_topic_.c_str());
    }

private:
    // Publishers / subscribers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // internal state
    std::string odom_topic_;
    std::vector<Waypoint> waypoints_;
    size_t waypoint_idx_;
    uint64_t offboard_setpoint_counter_;

    // position / yaw
    float current_x_, current_y_, current_z_;
    float current_yaw_;

    // initial position averaging
    bool collecting_initial_pos_;
    int initial_pos_samples_;
    float initial_x_sum_, initial_y_sum_, initial_z_sum_;
    float initial_x_avg_, initial_y_avg_, initial_z_avg_;
    rclcpp::Time initial_collect_start_;
    double initial_collect_time_;

    // control params
    double step_size_;
    double reach_tol_;

    // setpoint
    float setpoint_x_, setpoint_y_, setpoint_z_;

    // helpers
    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000); // µs
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        TrajectorySetpoint msg{};
        msg.position = {setpoint_x_, setpoint_y_, setpoint_z_};
        msg.yaw = current_yaw_;
        msg.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
        trajectory_setpoint_publisher_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
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
        msg.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
        vehicle_command_publisher_->publish(msg);
    }

    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
        RCLCPP_INFO(this->get_logger(), "Arm command sent.");
    }

    void control_loop()
    {
        // still collecting initial pos?
        if (collecting_initial_pos_) {
            double elapsed = (this->now() - initial_collect_start_).seconds();
            if (elapsed < initial_collect_time_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Collecting initial position... %.1f/%.1f s",
                                     elapsed, initial_collect_time_);
                // publish setpoints while waiting to make PX4 accept offboard
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
                offboard_setpoint_counter_++;
                return;
            } else {
                // compute averages
                if (initial_pos_samples_ > 0) {
                    initial_x_avg_ = initial_x_sum_ / static_cast<float>(initial_pos_samples_);
                    initial_y_avg_ = initial_y_sum_ / static_cast<float>(initial_pos_samples_);
                    initial_z_avg_ = initial_z_sum_ / static_cast<float>(initial_pos_samples_);
                } else {
                    // fallback to current if no samples
                    initial_x_avg_ = current_x_;
                    initial_y_avg_ = current_y_;
                    initial_z_avg_ = current_z_;
                }
                collecting_initial_pos_ = false;
                offboard_setpoint_counter_ = 0;
                RCLCPP_INFO(this->get_logger(), "Initial position avg: x=%.3f y=%.3f z=%.3f",
                            initial_x_avg_, initial_y_avg_, initial_z_avg_);
                // set initial setpoint to hover at initial altitude
                setpoint_x_ = initial_x_avg_;
                setpoint_y_ = initial_y_avg_;
                setpoint_z_ = initial_z_avg_;
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
                return;
            }
        }

        // safety: wait for a valid yaw and position
        if (std::isnan(current_x_) || std::isnan(current_y_) || std::isnan(current_z_) || std::isnan(current_yaw_)) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for valid odometry...");
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            offboard_setpoint_counter_++;
            return;
        }

        // start offboard & arm once we've published a few setpoints
        if (offboard_setpoint_counter_ == 10) {
            // set mode: (params used in your previous code)
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
            arm();
        }

        // compute current target in NED from body-relative waypoint
        // waypoint stored as body-frame (x_forward, y_right, z_down)
        Waypoint body_wp = waypoints_.at(waypoint_idx_);

        // rotate body-relative waypoint to world NED using yaw
        float bx = body_wp.x;
        float by = body_wp.y;
        float bz = body_wp.z; // z is relative vertical (NED: negative up)

        float target_x = initial_x_avg_ + (bx * std::cos(current_yaw_) - by * std::sin(current_yaw_));
        float target_y = initial_y_avg_ + (bx * std::sin(current_yaw_) + by * std::cos(current_yaw_));
        float target_z = initial_z_avg_ + bz;

        // compute distance from current to target
        float dx = target_x - current_x_;
        float dy = target_y - current_y_;
        float dz = target_z - current_z_;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist <= static_cast<float>(reach_tol_)) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu (dist=%.3f). Initiating land.", waypoint_idx_, dist);
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0f, 0.0f);
            // publish once then stop moving setpoints
            return;
        }

        // step toward target by step_size_
        float move_ratio = static_cast<float>(step_size_ / dist);
        if (move_ratio > 1.0f) move_ratio = 1.0f;

        float next_x = current_x_ + dx * move_ratio;
        float next_y = current_y_ + dy * move_ratio;
        float next_z = current_z_ + dz * move_ratio;

        setpoint_x_ = next_x;
        setpoint_y_ = next_y;
        setpoint_z_ = next_z;

        // publish offboard + trajectory setpoint
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
        offboard_setpoint_counter_++;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
