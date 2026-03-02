    #include <rclcpp/rclcpp.hpp>
    #include <px4_msgs/msg/offboard_control_mode.hpp>
    #include <px4_msgs/msg/trajectory_setpoint.hpp>
    #include <px4_msgs/msg/vehicle_command.hpp>
    #include <px4_msgs/msg/vehicle_odometry.hpp>
    #include <px4_msgs/msg/vehicle_global_position.hpp>
    #include <sensor_msgs/msg/nav_sat_fix.hpp>
    #include <cmath>
    #include <Eigen/Dense>
    #include <Eigen/Geometry> 


    using namespace std::chrono_literals;

    class DistanceStepDrone3 : public rclcpp::Node
    {
    public:
        DistanceStepDrone3();
    private:
        void relative_setpoint();
        void trajectory_logic();
        Eigen::Vector3f gps_to_ned(
        double lat, double lon, double alt,
        double ref_lat, double ref_lon, double ref_alt);
        void origin_callback(const sensor_msgs::msg::NavSatFix::SharedPtr origin_msg);
        void gps_own_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr gps_msg);
        void gps_lead_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr gps_msg);
        void odom_own_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr own_odom_msg);
        void offboard_control_mode();
        void arm();
        void set_offboard_command();

        // Publishers and subscribers
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr odom_lead_sub_gps;
        rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr odom_own_sub_gps;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_own_sub;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub;
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

        float current_yaw{0.0};

        bool z_sync_phase{true};
        float z_tolerance{0.3f};

        double own_lat{0}, own_lon{0}, own_alt{0};
        double lead_lat{0}, lead_lon{0}, lead_alt{0};
        bool gps_own_received{false};
        bool gps_lead_received{false};

        double ref_lat{0}, ref_lon{0}, ref_alt{0};
        bool ref_set_{false};

        Eigen::Vector3f px4_setpoint_origin{Eigen::Vector3f::Zero()};
        Eigen::Vector3f px4_target{Eigen::Vector3f::Zero()};
    };


    DistanceStepDrone3::DistanceStepDrone3() : Node("distance_step_drone3")
        {
            offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/px4_3/fmu/in/offboard_control_mode", 10);
            trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/px4_3/fmu/in/trajectory_setpoint", 10);
            vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
                "/px4_3/fmu/in/vehicle_command", 10);

            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                .best_effort()
                .durability_volatile();

            odom_lead_sub_gps = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
                "/px4_2/fmu/out/vehicle_global_position", 
                qos_profile,
                std::bind(&DistanceStepDrone3::gps_lead_callback, this, std::placeholders::_1));
            
            odom_own_sub_gps = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
                "/px4_3/fmu/out/vehicle_global_position", 
                qos_profile,
                std::bind(&DistanceStepDrone3::gps_own_callback, this, std::placeholders::_1));

            odom_own_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
                "/px4_3/fmu/out/vehicle_odometry", 
                qos_profile,
                std::bind(&DistanceStepDrone3::odom_own_callback, this, std::placeholders::_1));
            
            auto origin_qos = rclcpp::QoS(1)
                        .reliable()
                        .transient_local();

            origin_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/swarm/global_origin", origin_qos,
                std::bind(&DistanceStepDrone3::origin_callback, this, std::placeholders::_1));

            // Timer to send setpoints periodically
            timer_ = create_wall_timer(100ms, std::bind(&DistanceStepDrone3::relative_setpoint, this));
            
            global_des_3d << 4.0f, 0.0f, 0.0f;
            lead_global_des_3d << 0.0f, 0.0f, 0.0f;
        }

    Eigen::Vector3f DistanceStepDrone3::gps_to_ned(
        double lat, double lon, double alt,
        double ref_lat, double ref_lon, double ref_alt)
    {
        const double R = 6371000.0; // earth radius meters
        float north = static_cast<float>((lat - ref_lat) * (M_PI/180.0) * R);
        float east  = static_cast<float>((lon - ref_lon) * (M_PI/180.0) * R * std::cos(ref_lat * M_PI/180.0));
        float down  = static_cast<float>(-(alt - ref_alt));
        return {north, east, down};
    }

    void DistanceStepDrone3::origin_callback(const sensor_msgs::msg::NavSatFix::SharedPtr origin_msg)
    {
        ref_lat = origin_msg -> latitude;
        ref_lon = origin_msg -> longitude;
        ref_alt = origin_msg -> altitude;

        RCLCPP_INFO(this->get_logger(),
        "Received swarm origin: lat=%.8f lon=%.8f alt=%.2f",
        ref_lat, ref_lon, ref_alt);

        ref_set_ = true;
    }

    void DistanceStepDrone3::gps_own_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr gps_msg)
    {
        if (!gps_msg->lat_lon_valid || !gps_msg->alt_valid)return;
        own_lat = gps_msg -> lat;
        own_lon = gps_msg -> lon;
        own_alt = gps_msg -> alt;
        
        if (!ref_set_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Own GPS received but NED reference not set yet, waiting for drone 1 GPS...");
            return;
        }
        global_pos_3d = gps_to_ned(own_lat, own_lon, own_alt,
                                    ref_lat, ref_lon, ref_alt);
        gps_own_received = true;

        RCLCPP_DEBUG(this->get_logger(),
            "Own NED: [%.3f, %.3f, %.3f]",
            global_pos_3d.x(), global_pos_3d.y(), global_pos_3d.z());
    }
    void DistanceStepDrone3::gps_lead_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr gps_msg)
    {
        if (!gps_msg->lat_lon_valid || !gps_msg->alt_valid)return;
        lead_lat = gps_msg -> lat;
        lead_lon = gps_msg -> lon;
        lead_alt = gps_msg -> alt;

        if (!ref_set_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Own GPS received but NED reference not set yet, waiting for drone 1 GPS...");
            return;
        }
        lead_global_pos_3d = gps_to_ned(lead_lat, lead_lon, lead_alt,
                                        ref_lat,  ref_lon,  ref_alt);
        gps_lead_received = true;

        RCLCPP_DEBUG(this->get_logger(),
            "Lead NED: [%.3f, %.3f, %.3f]",
            lead_global_pos_3d.x(), lead_global_pos_3d.y(), lead_global_pos_3d.z());
    }

        void DistanceStepDrone3::relative_setpoint(){
        
        if (setpoint_counter_ < 10) {
            if (!odom_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for vehicle_odometry (drone 3)...");
            return;
            }

        if (!gps_own_received || !gps_lead_received) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Waiting for GPS (own=%d lead=%d)...",
                    gps_own_received, gps_lead_received);
                return;
            }

        offboard_control_mode();
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000; 
        traj.position = {
            px4_setpoint_origin[0],
            px4_setpoint_origin[1],
            px4_setpoint_origin[2]};
        traj.yaw = current_yaw;
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
            RCLCPP_INFO(this->get_logger(), "Setting offboard mode (drone 3)...");
            set_offboard_command();
            state_ = OffboardState::OFFBOARD;
            return;
        }

        if (state_ == OffboardState::OFFBOARD) {
            RCLCPP_INFO(this->get_logger(), "Arming (drone 3)...");
            arm();
            state_ = OffboardState::ARMED;
            return;
        }

        // publish offboard_control_mode sama setpoint terus menerus
        offboard_control_mode();
        trajectory_logic();
        }

    void DistanceStepDrone3::offboard_control_mode() {
        // PUBLISHER_COUNT (offboard control)
        px4_msgs::msg::OffboardControlMode offboard_msg{};
        offboard_msg.position = true;
        offboard_msg.velocity = false;
        offboard_msg.acceleration = false;
        offboard_msg.attitude = false;
        offboard_msg.body_rate = false;
        offboard_control_mode_pub_->publish(offboard_msg);
        }
        
    void DistanceStepDrone3::arm()
        {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.param1 = 1.0; // arm
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.target_system = 4; // Drone 3
        cmd.target_component = 1;
        cmd.source_system = 2;
        cmd.source_component = 1;
        cmd.from_external = true;
        cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Arm command sent (drone 3)");
    }

    void DistanceStepDrone3::set_offboard_command() {
        // PUBLISHER_COUNT (vehicle command :vehicle mode)
        px4_msgs::msg::VehicleCommand vehicle_mode_msg{};
        vehicle_mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_mode_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        vehicle_mode_msg.param1 = 1.0;  // custom
        vehicle_mode_msg.param2 = 6.0;  // offboard
        vehicle_mode_msg.target_system = 4; // Drone 3
        vehicle_mode_msg.target_component = 1;
        vehicle_mode_msg.source_system = 1;
        vehicle_mode_msg.source_component = 1;
        vehicle_command_pub_->publish(vehicle_mode_msg);
        RCLCPP_INFO(this->get_logger(), "Offboard mode command sent (drone 3)");
    }
        // func count 1
    void DistanceStepDrone3::trajectory_logic(){
        if (!gps_own_received || !gps_lead_received) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for GPS positions (own=%d lead=%d)...",
                gps_own_received, gps_lead_received);
            return;
        }

        input_pos_3d = -(global_pos_3d - lead_global_pos_3d) + (global_des_3d - lead_global_des_3d);
        target_pos_3d = global_pos_3d + (input_pos_3d*step_gain);
        px4_target = px4_setpoint_origin + (input_pos_3d*step_gain);

        //bedain logic pas takeoff
        if(z_sync_phase){
            px4_msgs::msg::TrajectorySetpoint traj{};
            traj.timestamp = this->get_clock()->now().nanoseconds() / 1000; 
            traj.position = {
                px4_setpoint_origin[0],
                px4_setpoint_origin[1],
                px4_target[2]};
            traj.yaw = current_yaw;
            trajectory_setpoint_pub_->publish(traj); 
            Eigen::Vector3f actual_offset  = global_pos_3d - lead_global_pos_3d;
            Eigen::Vector3f desired_offset = global_des_3d - lead_global_des_3d;
            Eigen::Vector3f offset_error   = actual_offset - desired_offset;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "\n--- CONVERGENCE CHECK (GPS/NED) ---"
                "\n  own  pos (NED) : [%.3f, %.3f, %.3f]"
                "\n  lead pos (NED) : [%.3f, %.3f, %.3f]"
                "\n  actual  offset : [%.3f, %.3f, %.3f]"
                "\n  desired offset : [%.3f, %.3f, %.3f]"
                "\n  error          : [%.3f, %.3f, %.3f]"
                "\n  error magnitude: %.3f m",
                global_pos_3d.x(),      global_pos_3d.y(),      global_pos_3d.z(),
                lead_global_pos_3d.x(), lead_global_pos_3d.y(), lead_global_pos_3d.z(),
                actual_offset.x(),      actual_offset.y(),      actual_offset.z(),
                desired_offset.x(),     desired_offset.y(),     desired_offset.z(),
                offset_error.x(),       offset_error.y(),       offset_error.z(),
                offset_error.norm());
            
                float z_error = target_pos_3d[2] - global_pos_3d[2];
            if (std::abs(z_error) < z_tolerance)
            {
                z_sync_phase = false;
            }
            return; 
        }
        
        if(!z_sync_phase){
        Eigen::Vector3f actual_offset  = global_pos_3d-lead_global_pos_3d;
        Eigen::Vector3f desired_offset = global_des_3d-lead_global_des_3d;
        Eigen::Vector3f offset_error   = actual_offset-desired_offset;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "\n--- CONVERGENCE CHECK (GPS/NED) ---"
            "\n  own  pos (NED) : [%.3f, %.3f, %.3f]"
            "\n  lead pos (NED) : [%.3f, %.3f, %.3f]"
            "\n  actual  offset : [%.3f, %.3f, %.3f]"
            "\n  desired offset : [%.3f, %.3f, %.3f]"
            "\n  error          : [%.3f, %.3f, %.3f]"
            "\n  error magnitude: %.3f m",
            global_pos_3d.x(),      global_pos_3d.y(),      global_pos_3d.z(),
            lead_global_pos_3d.x(), lead_global_pos_3d.y(), lead_global_pos_3d.z(),
            actual_offset.x(),      actual_offset.y(),      actual_offset.z(),
            desired_offset.x(),     desired_offset.y(),     desired_offset.z(),
            offset_error.x(),       offset_error.y(),       offset_error.z(),
            offset_error.norm());

        px4_msgs::msg::TrajectorySetpoint traj{};
            traj.timestamp = this->get_clock()->now().nanoseconds() / 1000; 
            traj.position = {
                px4_target[0],
                px4_target[1],
                px4_target[2]};
            traj.yaw = current_yaw;
            trajectory_setpoint_pub_->publish(traj);    
        }
    }

    void DistanceStepDrone3::odom_own_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr own_odom_msg)
    {
        float qw = own_odom_msg->q[0];
        float qx = own_odom_msg->q[1];
        float qy = own_odom_msg->q[2];
        float qz = own_odom_msg->q[3];

        current_yaw = std::atan2(
            2.0f * (qw * qz + qx * qy),
            1.0f - 2.0f * (qy * qy + qz * qz)
        );
        px4_setpoint_origin << own_odom_msg->position[0], own_odom_msg->position[1], own_odom_msg->position[2];

        odom_received_ = true;
    }

    int main(int argc, char* argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<DistanceStepDrone3>());
        rclcpp::shutdown();
        return 0;
    }
