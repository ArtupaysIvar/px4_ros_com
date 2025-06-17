#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ArucoLandingNode : public rclcpp::Node
{
public:
    ArucoLandingNode() : Node("aruco_landing_node")
    {
        // Publishers
        offboard_control_mode_pub_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Subscribers
        aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/aruco_visualization", 10, std::bind(&ArucoLandingNode::aruco_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            std::bind(&ArucoLandingNode::odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&ArucoLandingNode::control_loop, this));

        // State
        setpoint_sent_ = false;
        landed_ = false;
        aruco_detected_ = false;
        current_z_ = 0.0f;
        current_yaw_ = 0.0f;
        RCLCPP_INFO(this->get_logger(), "Aruco landing node started.");
    }

private:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose aruco_pose_;
    bool aruco_detected_;
    bool setpoint_sent_;
    bool landed_;

    float current_z_;
    float current_yaw_;

    void aruco_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (!msg->poses.empty()) {
            aruco_pose_ = msg->poses[0];
            aruco_detected_ = true;
        } else {
            aruco_detected_ = false;
        }
    }

    void odom_callback(const VehicleOdometry::SharedPtr msg)
    {
        float q0 = msg->q[0], q1 = msg->q[1], q2 = msg->q[2], q3 = msg->q[3];
        current_yaw_ = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                                  1.0f - 2.0f * (q2 * q2 + q3 * q3));
        current_z_ = msg->position[2];
    }

    void control_loop()
    {
        publish_offboard_control_mode();

        if (!setpoint_sent_) {
            RCLCPP_INFO(this->get_logger(), "Sending initial takeoff setpoint...");
            publish_trajectory_setpoint(0.0f, 0.0f, -5.0f, current_yaw_);
            setpoint_sent_ = true;
            for (int i = 0; i < 10; ++i) publish_offboard_control_mode();
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);  // Offboard
            arm();
            return;
        }

        if (landed_) return;

        if (aruco_detected_) {
            float x = aruco_pose_.position.x;
            float y = aruco_pose_.position.y;

            if (std::abs(x) < 0.03 && std::abs(y) < 0.03) {
                RCLCPP_INFO(this->get_logger(), "ArUco centered. Initiating landing...");
                land();
                landed_ = true;
                return;
            } else {
                float descend_z = current_z_ - 0.1f;
                RCLCPP_INFO(this->get_logger(), "ArUco detected. Adjusting position x=%.2f y=%.2f", x, y);
                publish_trajectory_setpoint(x, y, descend_z, current_yaw_);
            }
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Searching for ArUco marker...");
            publish_trajectory_setpoint(0.0f, 0.0f, -5.0f, current_yaw_);
        }
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint(float x, float y, float z, float yaw)
    {
        TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = {x, y, z};
        msg.yaw = yaw;
        trajectory_setpoint_pub_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
    {
        VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(this->get_logger(), "Arm command sent.");
    }

    void land()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting ArUco landing node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoLandingNode>());
    rclcpp::shutdown();
    return 0;
}
