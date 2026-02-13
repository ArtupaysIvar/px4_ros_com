improvements:
1) understanding px4 arming sequence:
- pertama inisialisasi pake offboard_control_mode px4_msgs::msg::OffboardControlMode
- publish trajectory_setpoint pake current x,y,z sama yaw buat inisialisasi drone's position
- abis tuh offboard vehicle command
- abis tuh arm
- offboard_control_mode jalan terus + trajectory logic buat mission
2) bikin enum class as a state machine buat arming drone nya
3) cari yaw angle from quaternion
4) ubah setpoint jadi in terms of body frame
5) initialized positions at takeoff  
6) initialized positions for every waypoint being done

to do:
1) bikin timer to hold between waypoints