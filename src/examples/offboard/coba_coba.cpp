#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cmath>

using std::placeholders::_1;

class ArucoLandingNode : public rclcpp::Node
{
public:
  ArucoLandingNode() : Node("aruco_landing_node")
  {
    using namespace std::chrono_literals;

    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>("/fmu/in/trajectory_setpoint", 10);
    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", 10, std::bind(&ArucoLandingNode::odom_callback, this, _1));
    aruco_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/aruco_visualization", 10, std::bind(&ArucoLandingNode::aruco_callback, this, _1));

    timer_ = create_wall_timer(100ms, std::bind(&ArucoLandingNode::control_loop, this));

    setpoint_sent_ = false;
    landed_ = false;
    aruco_detected_ = false;
    current_yaw_ = std::numeric_limits<float>::quiet_NaN();
    current_x_ = current_y_ = current_z_ = 0.0;

    arm();
    set_offboard_mode();
  }

private:
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool setpoint_sent_, landed_, aruco_detected_;
  geometry_msgs::msg::Pose aruco_pose_;
  float current_x_, current_y_, current_z_;
  float current_yaw_;

  void arm()
  {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.param1 = 1.0;
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void set_offboard_mode()
  {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    msg.param1 = 1;
    msg.param2 = 6;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void send_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_pub_->publish(msg);
  }

  void aruco_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (!msg->poses.empty()) {
      aruco_pose_ = msg->poses[0];
      aruco_detected_ = true;
    } else {
      aruco_detected_ = false;
    }
  }

  void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    float q0 = msg->q[0];
    float q1 = msg->q[1];
    float q2 = msg->q[2];
    float q3 = msg->q[3];
    current_yaw_ = std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    current_x_ = msg->position[0];
    current_y_ = msg->position[1];
    current_z_ = msg->position[2];
  }

  void control_loop()
  {
    send_offboard_control_mode();
    auto sp = px4_msgs::msg::VehicleLocalPositionSetpoint();
    sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    sp.yaw = current_yaw_;

    if (!setpoint_sent_) {
      sp.x = current_x_;
      sp.y = current_y_;
      sp.z = -5.0;
      trajectory_setpoint_pub_->publish(sp);
      setpoint_sent_ = true;
      return;
    }

    if (landed_) return;

    if (aruco_detected_) {
      float ax = aruco_pose_.position.x;
      float ay = aruco_pose_.position.y;

      float threshold = 0.03;
      float k = 0.5;

      if (std::abs(ax) < threshold && std::abs(ay) < threshold) {
        land();
        landed_ = true;
        return;
      }

      // Move towards marker
      sp.x = current_x_ + k * ax;
      sp.y = current_y_ + k * ay;  // RIGHT marker = NEGATIVE y => drone must move RIGHT (i.e., -y)
      sp.z = current_z_;

      trajectory_setpoint_pub_->publish(sp);
    }
  }

  void land()
  {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoLandingNode>());
  rclcpp::shutdown();
  return 0;
}