#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>


class traj_error : public rclcpp::Node {
public:
    traj_error();
    ~traj_error();

private:
    Eigen::Vector3f abs21;
    Eigen::Vector3f abs32;

    // position data
    float x1, y1, z1;
    float x2, y2, z2;
    float x3, y3, z3;

    bool received1 = false;
    bool received2 = false;
    bool received3 = false;

    std::ofstream csv_file_;
    rclcpp::Time mission_start_time_;

    // functions
    void odom_callback1(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg1);
    void odom_callback2(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg2);
    void odom_callback3(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg3);
    void error_calculation();

    // publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;

    // subscriber and timer
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_lead_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_fol2_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_fol3_sub;
};

traj_error::traj_error() : Node("traj_error_node")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                           .best_effort()
                           .durability_volatile();

    odom_lead_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/px4_1/fmu/out/vehicle_odometry",
        qos_profile,
        std::bind(&traj_error::odom_callback1, this, std::placeholders::_1));
    odom_fol2_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/px4_2/fmu/out/vehicle_odometry",
        qos_profile,
        std::bind(&traj_error::odom_callback2, this, std::placeholders::_1));
    odom_fol3_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/px4_3/fmu/out/vehicle_odometry",
        qos_profile,
        std::bind(&traj_error::odom_callback3, this, std::placeholders::_1));

    abs21 << 2.0f, - 3.464f, 0.0f; // abs position of drone 2 relative to drone 1
    abs32 << 4.0f, 0.0f, 0.0f; // abs position of drone 3 relative to drone 2

    mission_start_time_ = this->now();

    csv_file_.open("traj_error.csv", std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open traj_error.csv for writing.");
    } else {
        csv_file_ << "mission_time_s,error21,error32,"
                  << "x1,x2,x3,y1,y2,y3,z1,z2,z3\n";
        csv_file_ << std::fixed << std::setprecision(6);
        RCLCPP_INFO(this->get_logger(), "Writing trajectory data to traj_error.csv");
    }
}

traj_error::~traj_error()
{
    if (csv_file_.is_open()) {
        csv_file_.close();
    }
}

void traj_error::odom_callback1(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg1)
{
    x1 = odom_msg1->position[0];
    y1 = odom_msg1->position[1];
    z1 = odom_msg1->position[2];
    received1 = true;
    if (received1 && received2 && received3) {
        error_calculation();
    }
}

void traj_error::odom_callback2(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg2)
{
    x2 = odom_msg2->position[0];
    y2 = odom_msg2->position[1];
    z2 = odom_msg2->position[2];
    received2 = true;
    if (received1 && received2 && received3) {
        error_calculation();
    }
}

void traj_error::odom_callback3(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg3)
{
    x3 = odom_msg3->position[0];
    y3 = odom_msg3->position[1];
    z3 = odom_msg3->position[2];
    received3 = true;
    if (received1 && received2 && received3) {
        error_calculation();
    }
}

void traj_error::error_calculation()
{
    Eigen::Vector3f pos1(x1, y1, z1);
    Eigen::Vector3f pos2(x2, y2, z2);
    Eigen::Vector3f pos3(x3, y3, z3);

    Eigen::Vector3f rel21 = pos2 - pos1;
    Eigen::Vector3f rel32 = pos3 - pos2;

    Eigen::Vector3f error21 = rel21 - abs21;
    Eigen::Vector3f error32 = rel32 - abs32;

    float mag_error21 = error21.norm();
    float mag_error32 = error32.norm();
    const double mission_time_s = (this->now() - mission_start_time_).seconds();

    RCLCPP_INFO(this->get_logger(),
                "t=%.3fs | Trajectory error 2-1: %.3f, error 3-2: %.3f",
                mission_time_s,
                mag_error21,
                mag_error32);

    if (csv_file_.is_open()) {
        csv_file_ << mission_time_s << ","
                  << mag_error21 << ","
                  << mag_error32 << ","
                  << x1 << "," << x2 << "," << x3 << ","
                  << y1 << "," << y2 << "," << y3 << ","
                  << z1 << "," << z2 << "," << z3 << "\n";
    }

    // Reset flags for next calculation
    received1 = false;
    received2 = false;
    received3 = false;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<traj_error>());
    rclcpp::shutdown();
    return 0;
}
