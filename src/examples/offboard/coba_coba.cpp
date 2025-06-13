#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

using std::placeholders::_1;

class ArucoLandingNode : public rclcpp::Node
{
public:
  ArucoLandingNode() : Node("aruco_landing_node")
  {
    using namespace std::chrono_literals;

    // Publishers
    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>("/fmu/in/trajectory_setpoint", 10);

    // Subscribers
    aruco_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/aruco_visualization", 10, std::bind(&ArucoLandingNode::aruco_callback, this, _1));

    // Timer
    timer_ = create_wall_timer(100ms, std::bind(&ArucoLandingNode::control_loop, this));

    // Init state
    setpoint_sent_ = false;
    landed_ = false;
    aruco_detected_ = false;

    // Arm and set to offboard
    arm();
    set_offboard_mode();
  }

private:
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr trajectory_setpoint_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool setpoint_sent_;
  bool landed_;
  bool aruco_detected_;
  geometry_msgs::msg::Pose aruco_pose_;

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
    msg.param1 = 1;  // custom mode
    msg.param2 = 6;  // PX4 OFFBOARD mode
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
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
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

  void control_loop()
  {
    send_offboard_control_mode();

    auto sp = px4_msgs::msg::VehicleLocalPositionSetpoint();
    sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    if (!setpoint_sent_) {
      // Initial takeoff to 5m
      sp.z = -5.0;
      trajectory_setpoint_pub_->publish(sp);
      setpoint_sent_ = true;
      return;
    }

    if (landed_) return;

    // If ArUco detected and it's centered under the drone
    if (aruco_detected_) {
      float x = aruco_pose_.position.x;
      float y = aruco_pose_.position.y;

      if (std::abs(x) < 0.03 && std::abs(y) < 0.03) {
        // It's directly below
        land();
        landed_ = true;
      }
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
