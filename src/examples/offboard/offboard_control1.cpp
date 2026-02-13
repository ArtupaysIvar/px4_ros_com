#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

using namespace std::chrono_literals;

class Drone1Control : public rclcpp::Node {
public:
    Drone1Control();

private:
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);
    void offboard_control_mode();
    void arm();
    void set_offboard_command();
    void relative_setpoint();
    void trajectory_logic();
    bool waypoint_reached(const Eigen::Vector2f &target);

    // kopi dari state machine
    enum class OffboardState {
    INIT,
    OFFBOARD,
    ARMED
    };
    OffboardState state_{OffboardState::INIT};
    int setpoint_counter_{0};

    // publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    
    // subscriber and timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub_;

    // buat dapet orientasi dan kalkulasi rotational angle
    float current_yaw{0.0};
    // float current_z{0.0};
    // float target_z{0.0};
    Eigen::Matrix2f yaw_rotational_matrix;
    Eigen::Matrix2f yaw_rot_matrix;
    Eigen::Vector2f target_pos;

    std::vector<Eigen::Vector3f> body_3dpos_setpoint;
    Eigen::Vector2f body_2dpos_setpoint;
    Eigen::Vector3f global_position_3d;
    Eigen::Vector2f global_position_2d;

    size_t current_wp_idx_{0};
    float wp_reached_threshold_{0.1f};

    bool initialized_pos{0};
    Eigen::Vector3f init_global_position_3d;
    Eigen::Vector2f init_global_position_2d;

};

Drone1Control::Drone1Control(): Node("drone1_control_node") 
{

    /*
    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/px4_1/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/px4_1/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/px4_1/fmu/in/vehicle_command", 10);
    */

    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    vehicle_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
        std::bind(&Drone1Control::odom_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(100ms, std::bind(&Drone1Control::relative_setpoint, this));
    
    // JANGAN LUPA SET WAYPOINT NYA
    body_3dpos_setpoint.reserve(20);
    body_3dpos_setpoint.emplace_back(0.0f, 0.0f, -2.0f);
    body_3dpos_setpoint.emplace_back(2.0f, 0.0f, -2.0f);
    body_3dpos_setpoint.emplace_back(-3.0f, 0.0f, -2.0f);
    body_3dpos_setpoint.emplace_back(1.0f, 0.0f, -2.0f);
    body_3dpos_setpoint.emplace_back(5.0f, 0.0f, -2.0f);

    // body_3dpos_setpoint.emplace_back(2.0f, 0.0f, -2.0f);
    // body_3dpos_setpoint.emplace_back(0.0f, 5.0f, -2.0f);

    // target_z = -2.0f;
    // body_vel_setpoint << 1.0f, 1.0f, 1.0f; 
}

void Drone1Control::arm() {
    // PUBLISHER_COUNT (vehicle command: arm)
    px4_msgs::msg::VehicleCommand arm_msg{};
    arm_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    arm_msg.param1 = 1.0;  // arm
    arm_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    arm_msg.target_system = 1;     
    arm_msg.target_component = 1;
    arm_msg.source_system = 1;
    arm_msg.source_component = 1;
    vehicle_command_pub_->publish(arm_msg);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void Drone1Control::set_offboard_command() {
    // PUBLISHER_COUNT (vehicle command :vehicle mode)
    px4_msgs::msg::VehicleCommand vehicle_mode_msg{};
    vehicle_mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_mode_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    vehicle_mode_msg.param1 = 1.0;  // custom
    vehicle_mode_msg.param2 = 6.0;  // offboard
    vehicle_mode_msg.target_system = 1;
    vehicle_mode_msg.target_component = 1;
    vehicle_mode_msg.source_system = 1;
    vehicle_mode_msg.source_component = 1;
    vehicle_command_pub_->publish(vehicle_mode_msg);
    RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
}

void Drone1Control::offboard_control_mode() {
    // PUBLISHER_COUNT (offboard control)
    px4_msgs::msg::OffboardControlMode offboard_msg{};
    offboard_msg.position = true;
    // set true apa ya?
    offboard_msg.velocity = false;
    offboard_msg.acceleration = false;
    offboard_msg.attitude = false;
    offboard_msg.body_rate = false;
    offboard_control_mode_pub_->publish(offboard_msg);
}

void Drone1Control::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg)
{
    float qw = odom_msg->q[0];
    float qx = odom_msg->q[1];
    float qy = odom_msg->q[2];
    float qz = odom_msg->q[3];

    current_yaw = std::atan2(
        2.0f * (qw * qz + qx * qy),
        1.0f - 2.0f * (qy * qy + qz * qz)
    );
    /* ubah dari quaternion to yaw using:
    yaw = atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y^2 + q_z^2)) for
    */
    yaw_rotational_matrix =  Eigen::Rotation2Df(current_yaw).toRotationMatrix();
    // global_position << odom_msg->position[0], odom_msg->position[1], odom_msg->position[2];
    global_position_3d << odom_msg->position[0], odom_msg->position[1], odom_msg->position[2];
    global_position_2d << odom_msg->position[0], odom_msg->position[1];
}

void Drone1Control::trajectory_logic(){
    if (current_wp_idx_ >= body_3dpos_setpoint.size()) {
        RCLCPP_INFO(this->get_logger(), "compltd");
        return;
    }
    
    if(!initialized_pos){
            // init_global_position_3d << global_position_3d[0], global_position_3d[1], global_position_3d[2];
            init_global_position_3d = global_position_3d;
            initialized_pos = true;
            RCLCPP_INFO(this->get_logger(), "GANTI INIT_POS");
    }

    // global_position_2d << global_position_3d.x(), global_position_3d.y();
    init_global_position_2d << init_global_position_3d[0], init_global_position_3d[1];

    const auto &wp = body_3dpos_setpoint[current_wp_idx_];
    body_2dpos_setpoint << wp.x(), wp.y();

    target_pos = init_global_position_2d + (yaw_rotational_matrix * body_2dpos_setpoint);

    // PUBLISHER_COUNT (traj. setpoint)
    px4_msgs::msg::TrajectorySetpoint traj{};
    traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    traj.position = {target_pos[0], target_pos[1], wp.z()};
    // traj.velocity = {body_vel_setpoint[0], body_vel_setpoint[1], body_vel_setpoint[2]};
    traj.yaw = current_yaw;
    trajectory_setpoint_pub_->publish(traj);

    if (waypoint_reached(target_pos)) {
        RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_wp_idx_);
        current_wp_idx_++;
        initialized_pos = false;
    }
}

bool Drone1Control::waypoint_reached(const Eigen::Vector2f &target)
{
    return (global_position_2d - target).norm() < wp_reached_threshold_;
}

void Drone1Control::relative_setpoint(){
    
    if (setpoint_counter_ < 10) {
        offboard_control_mode();
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000; 
        traj.position = {
            global_position_3d[0],
            global_position_3d[1],
            global_position_3d[2]};
        traj.yaw = current_yaw;
        trajectory_setpoint_pub_->publish(traj);
        setpoint_counter_++;

        if(!initialized_pos){
            // init_global_position_3d << global_position_3d[0], global_position_3d[1], global_position_3d[2];
            init_global_position_3d = global_position_3d;
            initialized_pos = true;
            RCLCPP_INFO(this->get_logger(), "GANTI INIT_POS");
        }
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

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Drone1Control>());
    rclcpp::shutdown();
    return 0;
}
