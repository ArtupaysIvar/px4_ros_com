improvements on leader:
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
7) bikin timer to hold between waypoints
8) implement dx/dt
9) drone 1 nunggu drone 3 untuk take off

improvements on follower:
1) fixed yaw nya

to do:
1) 
2) BUG: nggak ada hold in between waypoint 0 ke 1 (FIXED) 
3) BUG: initial position drone null karna subscriber kosong (FIXED)
4) benerin sistem logging nya (opsional)

