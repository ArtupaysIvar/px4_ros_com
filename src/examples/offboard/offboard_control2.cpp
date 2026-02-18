#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry> 


using namespace std::chrono_literals;

class DistanceStepDrone2 : public rclcpp::Node
{
public:
    DistanceStepDrone2();
private:
    void relative_setpoint();
    void trajectory_logic();
    void odom_lead_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr lead_odom_msg);
    void odom_own_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr own_odom_msg);
    void offboard_control_mode();
    void arm();
    void set_offboard_command();

    // Publishers and subscribers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom1_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom2_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    enum class OffboardState {
    INIT,
    OFFBOARD,
    ARMED
    };
    OffboardState state_{OffboardState::INIT};
    int setpoint_counter_{0};
    bool odom_received_ = false;

    Eigen::Vector2f target_pos;
    Eigen::Vector2f u_lead;
    
    Eigen::Vector3f lead_global_pos_3d;
    Eigen::Vector3f global_pos_3d;
    Eigen::Vector2f lead_global_pos_2d;
    Eigen::Vector2f global_pos_2d;

    Eigen::Vector3f global_des_3d;
    Eigen::Vector3f lead_global_des_3d;

    Eigen::Vector3f input_pos_3d;
    Eigen::Vector3f target_pos_3d;
    const float step_gain{0.5f};
};


DistanceStepDrone2::DistanceStepDrone2() : Node("distance_step_drone2")
    {
        offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/px4_2/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/px4_2/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/px4_2/fmu/in/vehicle_command", 10);

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
        timer_ = create_wall_timer(100ms, std::bind(&DistanceStepDrone2::relative_setpoint, this));
        
        global_des_3d << 0.0f, -10.0f, 0.0f;
        lead_global_des_3d << 0.0f, 0.0f, 0.0f;
        
    }

    void DistanceStepDrone2::relative_setpoint(){
    
    if (setpoint_counter_ < 10) {
        if (!odom_received_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Waiting for vehicle_odometry...");
        return;
        }

        offboard_control_mode();
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000; 
        traj.position = {
            global_pos_3d[0],
            global_pos_3d[1],
            global_pos_3d[2]};
        // traj.yaw = current_yaw;
        trajectory_setpoint_pub_->publish(traj);
        setpoint_counter_++;
        /*
        if(!initialized_pos){
            const auto &wp = body_3dpos_setpoint[current_wp_idx_];
            // init_global_position_3d << global_position_3d[0], global_position_3d[1], global_position_3d[2];
            init_global_position_3d = global_position_3d;
            initialized_pos = true;

            RCLCPP_INFO(this->get_logger(),
            "GANTI INIT_POS | WP %zu | body_wp = [%.2f, %.2f, %.2f] "
            "| init = [%.3f, %.3f, %.3f] | current = [%.3f, %.3f, %.3f]",
            current_wp_idx_,
            wp.x(), wp.y(), wp.z(),
            init_global_position_3d.x(),
            init_global_position_3d.y(),
            init_global_position_3d.z(),
            global_position_3d.x(),
            global_position_3d.y(),
            global_position_3d.z());
        }
        */
        return;
    }
    
    if (state_ == OffboardState::INIT) {
        RCLCPP_INFO(this->get_logger(), "Setting offboard mode...");
        set_offboard_command();
        state_ = OffboardState::OFFBOARD;
        return;
    }

    if (state_ == OffboardState::OFFBOARD) {
        RCLCPP_INFO(this->get_logger(), "Arming...");
        arm();
        state_ = OffboardState::ARMED;
        return;
    }

    // publish offboard_control_mode sama setpoint terus menerus
    offboard_control_mode();
    trajectory_logic();
    }

void DistanceStepDrone2::offboard_control_mode() {
    // PUBLISHER_COUNT (offboard control)
    px4_msgs::msg::OffboardControlMode offboard_msg{};
    offboard_msg.position = true;
    offboard_msg.velocity = false;
    offboard_msg.acceleration = false;
    offboard_msg.attitude = false;
    offboard_msg.body_rate = false;
    offboard_control_mode_pub_->publish(offboard_msg);
    }
    
void DistanceStepDrone2::arm()
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

void DistanceStepDrone2::set_offboard_command() {
    // PUBLISHER_COUNT (vehicle command :vehicle mode)
    px4_msgs::msg::VehicleCommand vehicle_mode_msg{};
    vehicle_mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_mode_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    vehicle_mode_msg.param1 = 1.0;  // custom
    vehicle_mode_msg.param2 = 6.0;  // offboard
    vehicle_mode_msg.target_system = 3; // Drone 2
    vehicle_mode_msg.target_component = 1;
    vehicle_mode_msg.source_system = 1;
    vehicle_mode_msg.source_component = 1;
    vehicle_command_pub_->publish(vehicle_mode_msg);
    RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
}

    // func count 1
void DistanceStepDrone2::trajectory_logic(){
    input_pos_3d = -(global_pos_3d - lead_global_pos_3d) + (global_des_3d - lead_global_des_3d);
    target_pos_3d = global_pos_3d + (input_pos_3d*step_gain);

    px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000; 
        traj.position = {
            target_pos_3d[0],
            target_pos_3d[1],
            target_pos_3d[2]};
        trajectory_setpoint_pub_->publish(traj);    
}

void DistanceStepDrone2::odom_own_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr own_odom_msg)
{
    // float qw = odom_msg->q[0];
    // float qx = odom_msg->q[1];
    // float qy = odom_msg->q[2];
    // float qz = odom_msg->q[3];

    // current_yaw = std::atan2(
    //     2.0f * (qw * qz + qx * qy),
    //     1.0f - 2.0f * (qy * qy + qz * qz)
    // );
    /* ubah dari quaternion to yaw using:
    yaw = atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y^2 + q_z^2)) for
    */
    // yaw_rotational_matrix =  Eigen::Rotation2Df(current_yaw).toRotationMatrix();
    // global_position << odom_msg->position[0], odom_msg->position[1], odom_msg->position[2];
    global_pos_3d << own_odom_msg->position[0], own_odom_msg->position[1], own_odom_msg->position[2];
    global_pos_2d << own_odom_msg->position[0], own_odom_msg->position[1];

    odom_received_ = true;
}

void DistanceStepDrone2::odom_lead_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr lead_odom_msg)
    {
    lead_global_pos_3d << lead_odom_msg->position[0], lead_odom_msg->position[1], lead_odom_msg->position[2];
    lead_global_pos_2d << lead_odom_msg->position[0], lead_odom_msg->position[1];
    
    // RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    //     "[ODOM2] Drone 2 position: [%.2f, %.2f, %.2f], yaw: %.2f", 
    //     dbc_.x_lead, dbc_.y_lead, dbc_.z_lead, current_yaw_);
    }

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceStepDrone2>());
    rclcpp::shutdown();
    return 0;
}
