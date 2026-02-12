#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

using namespace std::chrono_literals;

class Drone1Control : public rclcpp::Node {
public:
    Drone1Control();

private:
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);
    // void publish_displacement();
    void offboard_control();
    void arm();
    void set_offboard_mode();
    // void trajectory_setpoint();
    void relative_setpoint();

    // publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    
    // subscriber and timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_sub_;

    // buat dapet orientasi dan kalkulasi rotational angle
    float current_yaw_{0.0};
    float current_z{0.0};
    Eigen::Matrix2f yaw_rotational_matrix;
    Eigen::Vector3f global_position;
    Eigen::Vector2f global_position_2d;

    Eigen::Vector2f global_pos_2d_;
    Eigen::Vector2f body_setpoint;   
    Eigen::Matrix2f yaw_rot_matrix;
    Eigen::Vector2f target_pos;
};

Drone1Control::Drone1Control(): Node("drone1_control_node") 
{
    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/px4_1/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/px4_1/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/px4_1/fmu/in/vehicle_command", 10);

    vehicle_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
        std::bind(&Drone1Control::odom_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(100ms, std::bind(&Drone1Control::relative_setpoint, this));
    
    // JANGAN LUPA SET WAYPOINT NYA
    body_setpoint << 0, 1; 
}
void Drone1Control::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg)
{
    float qw = odom_msg->q[0];
    float qx = odom_msg->q[1];
    float qy = odom_msg->q[2];
    float qz = odom_msg->q[3];

    current_yaw_ = std::atan2(
        2.0f * (qw * qz + qx * qy),
        1.0f - 2.0f * (qy * qy + qz * qz)
    );
    /* ubah dari quaternion to yaw using:
    yaw = atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y^2 + q_z^2)) for
    */
    yaw_rotational_matrix =  Eigen::Rotation2Df(current_yaw_).toRotationMatrix();
    // global_position << odom_msg->position[0], odom_msg->position[1], odom_msg->position[2];
    global_position_2d << odom_msg->position[0], odom_msg->position[1];
    current_z = odom_msg->position[2];
}

void Drone1Control::relative_setpoint(){
    // body_setpoint << 0, 1; 
    target_pos = global_position_2d + (yaw_rotational_matrix * body_setpoint);
    px4_msgs::msg::TrajectorySetpoint traj{};
    traj.position = {target_pos[0], target_pos[1], current_z};
    trajectory_setpoint_pub_->publish(traj);
}



void Drone1Control::offboard_control() {
// publisher pertama (offboard)
    px4_msgs::msg::OffboardControlMode offboard_msg{};
    offboard_msg.position = true;
    offboard_msg.velocity = false;
    offboard_msg.acceleration = false;
    offboard_msg.attitude = false;
    offboard_msg.body_rate = false;
    offboard_control_mode_pub_->publish(offboard_msg);
}

void Drone1Control::arm() {
    px4_msgs::msg::VehicleCommand arm_msg{};
    arm_msg.param1 = 1.0;  // arm
    arm_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    arm_msg.target_system = 2;     
    arm_msg.target_component = 1;
    arm_msg.source_system = 2;
    arm_msg.source_component = 1;
    vehicle_command_pub_->publish(arm_msg);
    // RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void Drone1Control::set_offboard_mode() {
    px4_msgs::msg::VehicleCommand vehicle_mode_msg{};
    vehicle_mode_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    vehicle_mode_msg.param1 = 1.0;  // custom
    vehicle_mode_msg.param2 = 6.0;  // offboard
    vehicle_mode_msg.target_system = 2;
    vehicle_mode_msg.target_component = 1;
    vehicle_mode_msg.source_system = 2;
    vehicle_mode_msg.source_component = 1;
    vehicle_command_pub_->publish(vehicle_mode_msg);
    // RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Drone1Control>());
    rclcpp::shutdown();
    return 0;
}