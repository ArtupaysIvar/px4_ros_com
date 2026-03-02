#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

using namespace std::chrono_literals;

class Drone1Control : public rclcpp::Node {
public:
    Drone1Control();

private:
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);
    void gps_follower_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr gps_msg);
    void gps_own_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr gps_msg);
    void offboard_control_mode();
    void arm();
    void set_offboard_command();
    void relative_setpoint();
    void trajectory_logic();
    bool waypoint_reached(const Eigen::Vector2f &target);
    Eigen::Vector2f step_logic(const Eigen::Vector2f &current, const Eigen::Vector2f &target, float max_step);

    // publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr origin_pub;
    // subscriber and timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_own_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gps_own_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gps_follower_sub;
    


    // copy dari state machine
    enum class OffboardState {
    INIT,
    OFFBOARD,
    ARMED
    };
    OffboardState state_{OffboardState::INIT};
    int setpoint_counter_{0};


    // buat dapet orientasi dan kalkulasi rotational angle
    float current_yaw{0.0};
    // float current_z{0.0};
    // float target_z{0.0};
    Eigen::Matrix2f yaw_rotational_matrix;
    // Eigen::Matrix2f yaw_rot_matrix;
    Eigen::Vector2f target_pos;
    float target_z;

    std::vector<Eigen::Vector3f> body_3dpos_setpoint;
    Eigen::Vector2f body_2dpos_setpoint;
    Eigen::Vector3f global_position_3d;
    Eigen::Vector2f global_position_2d;

    Eigen::Vector3f follower_odom;

    size_t current_wp_idx_{0};
    float wp_reached_threshold_{0.1f};

    bool initialized_pos{0};
    Eigen::Vector3f init_global_position_3d;
    Eigen::Vector2f init_global_position_2d;

    bool check_if_done {true};

    // hold timer variables
    rclcpp::Time wp_reached_time_;
    bool holding_ = false;
    const double hold_duration_ = 5.0; 

    float fol_alt{0};
    float init_fol_alt{0};
    bool fol_alt_check = false;

    float max_step = 0.5f;

    bool odom_received_ {false}; // supaya pasti ada isi data nya
    float z_tolerance = 0.3f;

    bool origin_set = false;
    // double own_lat{0}, own_lon{0}, own_alt{0};
    // double fol_lat{0}, fol_lon{0}, fol_alt{0};
    bool gps_own_received{false};
    // bool gps_fol_received{false};

};

Drone1Control::Drone1Control(): Node("drone1_control_node") 
{
    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/px4_1/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/px4_1/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/px4_1/fmu/in/vehicle_command", 10);
    auto qos = rclcpp::QoS(1)
                        .reliable()
                        .transient_local();
    origin_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/swarm/global_origin", qos);

    
/*
    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);
*/  
    
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();

    odom_own_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/px4_1/fmu/out/vehicle_odometry", qos_profile,
        std::bind(&Drone1Control::odom_callback, this, std::placeholders::_1));

    gps_own_sub = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/px4_1/fmu/out/vehicle_global_position", qos_profile,
        std::bind(&Drone1Control::gps_own_callback, this, std::placeholders::_1));

    gps_follower_sub = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/px4_3/fmu/out/vehicle_global_position", qos_profile,
        std::bind(&Drone1Control::gps_follower_callback, this, std::placeholders::_1));
    
    timer_ = create_wall_timer(100ms, std::bind(&Drone1Control::relative_setpoint, this));
    
    // JANGAN LUPA SET WAYPOINT NYA
    body_3dpos_setpoint.reserve(20);
    // kalo z = 0 berarti dia simply tidak climb (bukan berarti landing) 
    body_3dpos_setpoint.emplace_back(0.0f, 0.0f, -5.0f);
    body_3dpos_setpoint.emplace_back(2.0f, 0.0f, 0.0f);
    body_3dpos_setpoint.emplace_back(-3.0f, 0.0f, 0.0f);
    body_3dpos_setpoint.emplace_back(1.0f, 0.0f, 0.0f);
    body_3dpos_setpoint.emplace_back(5.0f, 0.0f, 0.0f);
    body_3dpos_setpoint.emplace_back(0.0f, 3.0f, 0.0f);
    body_3dpos_setpoint.emplace_back(0.0f, 0.0f, 5.0f);
}

void Drone1Control::arm() {
    // PUBLISHER_COUNT (vehicle command: arm)
    px4_msgs::msg::VehicleCommand arm_msg{};
    arm_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    arm_msg.param1 = 1.0;  // arm
    arm_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    arm_msg.target_system = 2;     
    arm_msg.target_component = 1;
    arm_msg.source_system = 1;
    arm_msg.source_component = 1;
    vehicle_command_pub_->publish(arm_msg);
    RCLCPP_INFO(this->get_logger(), "Arm command sent (drone 1)");
}

void Drone1Control::set_offboard_command() {
    // PUBLISHER_COUNT (vehicle command :vehicle mode)
    px4_msgs::msg::VehicleCommand vehicle_mode_msg{};
    vehicle_mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_mode_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    vehicle_mode_msg.param1 = 1.0;  // custom
    vehicle_mode_msg.param2 = 6.0;  // offboard
    vehicle_mode_msg.target_system = 2;
    vehicle_mode_msg.target_component = 1;
    vehicle_mode_msg.source_system = 1;
    vehicle_mode_msg.source_component = 1;
    vehicle_command_pub_->publish(vehicle_mode_msg);
    RCLCPP_INFO(this->get_logger(), "Offboard mode command sent (drone 1)");
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

void Drone1Control::gps_own_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr gps_msg) {
    if (!origin_set && gps_msg->lat_lon_valid && gps_msg->alt_valid)
    {
    double own_lat = gps_msg -> lat;
    double own_lon = gps_msg -> lon;
    double own_alt = gps_msg -> alt;
    gps_own_received = true;
    
    sensor_msgs::msg::NavSatFix msg{};
    msg.latitude = own_lat;
    msg.longitude = own_lon;
    msg.altitude = own_alt;
    origin_pub -> publish(msg);

    RCLCPP_INFO(this->get_logger(),
    "Swarm origin published: lat=%.8f lon=%.8f alt=%.2f",
    own_lat, own_lon, own_alt);
    
    origin_set = true;
    }
    
}

void Drone1Control::gps_follower_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr gps_msg) {
    fol_alt = static_cast<float>(gps_msg->alt);
 
    // float down = static_cast<float>(-(alt - ref_alt));
    if(!fol_alt_check){
        init_fol_alt = fol_alt;
        fol_alt_check = true;
        RCLCPP_INFO(this->get_logger(),
            "Drone 3 initial altitude recorded: %.2f m", init_fol_alt);
    }
    // follower_odom << odom_msg->position[0], odom_msg->position[1], odom_msg->position[2];
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

    yaw_rotational_matrix <<
    std::cos(current_yaw), -std::sin(current_yaw),
    std::sin(current_yaw),  std::cos(current_yaw);

    global_position_3d << odom_msg->position[0], odom_msg->position[1], odom_msg->position[2];
    global_position_2d << odom_msg->position[0], odom_msg->position[1];

    odom_received_ = true;

}

void Drone1Control::trajectory_logic(){
    if (current_wp_idx_ >= body_3dpos_setpoint.size()) {
        if (check_if_done){
        RCLCPP_INFO(this->get_logger(),
            "MISSION COMPLETE | final position = [%.3f, %.3f, %.3f]",
            global_position_3d.x(),
            global_position_3d.y(),
            global_position_3d.z());
            check_if_done = false;
        }
        // RCLCPP_INFO(this->get_logger(), "compltd");
        return;
    }
    
    if(!initialized_pos){
        // init_global_position_3d << global_position_3d[0], global_position_3d[1], global_position_3d[2];
        init_global_position_3d = global_position_3d;
        initialized_pos = true;
        
        const auto &wp = body_3dpos_setpoint[current_wp_idx_];
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

    // global_position_2d << global_position_3d.x(), global_position_3d.y();
    init_global_position_2d << init_global_position_3d[0], init_global_position_3d[1];

    const auto &wp = body_3dpos_setpoint[current_wp_idx_];
    body_2dpos_setpoint << wp.x(), wp.y();

    target_pos = init_global_position_2d + (yaw_rotational_matrix * body_2dpos_setpoint);
    
    target_z = init_global_position_3d[2] + wp.z();

    // check if drone 3 has ascended
    if (current_wp_idx_ == 1){
        // const auto &wp = body_3dpos_setpoint[current_wp_idx_];
        // harusnya ga hard-coded
        float target_drone3_z = init_fol_alt + 5.0f;
        
        bool drone3_ready =
            fol_alt_check &&
            std::abs(fol_alt - target_drone3_z) < z_tolerance;

        if (!drone3_ready)
        {
            // hold current position
            px4_msgs::msg::TrajectorySetpoint traj{};
            traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            traj.position = {
                global_position_3d.x(),
                global_position_3d.y(),
                global_position_3d.z()
            };
            traj.yaw = current_yaw;
            trajectory_setpoint_pub_->publish(traj);
            return;
        }
}
    // 5 sec hold in between wp logic
    if (holding_) {
        double elapsed = (this->get_clock()->now() - wp_reached_time_).seconds();
        
        if (elapsed >= hold_duration_) {
            RCLCPP_INFO(this->get_logger(),
                 "Hold complete at waypoint %zu", current_wp_idx_);
            current_wp_idx_++;
            initialized_pos = false;
            holding_ = false;
        }

        Eigen::Vector2f stepped_pos = step_logic(global_position_2d, target_pos, max_step);
        
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        traj.position = {stepped_pos.x(), stepped_pos.y(), target_z};
        // traj.velocity = {body_vel_setpoint[0], body_vel_setpoint[1], body_vel_setpoint[2]};
        traj.yaw = current_yaw;
        trajectory_setpoint_pub_->publish(traj);
        return;
    }
    

    if (waypoint_reached(target_pos)) {
        RCLCPP_INFO(this->get_logger(), "Reached waypoint");
        wp_reached_time_ = this->get_clock()->now();
        holding_ = true;
    }
    
    Eigen::Vector2f stepped_pos = step_logic(global_position_2d, target_pos, max_step);
    px4_msgs::msg::TrajectorySetpoint traj{};
    traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    traj.position = {stepped_pos.x(), stepped_pos.y(), target_z};
    traj.yaw = current_yaw;
    trajectory_setpoint_pub_->publish(traj);

}

bool Drone1Control::waypoint_reached(const Eigen::Vector2f &target)
{
    float distance_2d = (global_position_2d - target).norm();
    float distance_z = std::abs(global_position_3d[2] - target_z);
    
    return (distance_2d < wp_reached_threshold_) && 
           (distance_z < wp_reached_threshold_);
}

Eigen::Vector2f Drone1Control::step_logic( // this one is to control velocity
    const Eigen::Vector2f &current,
    const Eigen::Vector2f &target,
    float max_step)
{
    Eigen::Vector2f wp_disp = target - current;
    float wp_dist = wp_disp.norm();

    if (wp_dist <= max_step || wp_dist < 1e-3f)
        return target;

    return current + wp_disp.normalized() * max_step;
}

// function buat starting
void Drone1Control::relative_setpoint(){
    if (setpoint_counter_ < 10) {
        if (!odom_received_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Waiting for vehicle_odometry (drone 1)...");
        return;
        }

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
        return;
    }
    
    if (state_ == OffboardState::INIT) {
        RCLCPP_INFO(this->get_logger(), "Setting offboard mode (drone 1)...");
        set_offboard_command();
        state_ = OffboardState::OFFBOARD;
        return;
    }

    if (state_ == OffboardState::OFFBOARD) {
        RCLCPP_INFO(this->get_logger(), "Arming (drone 1)...");
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
