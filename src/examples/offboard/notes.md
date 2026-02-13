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
7) bikin timer to hold between waypoints

to do:
1) implement dx/dt
2) FIX BUG initial position null

RUN 1:
ravi@ArtupaysIvar:~/capstone-drone-project/ros2_ws$ ros2 run px4_ros_com offboard_control1
[INFO] [1770989216.257660208] [drone1_control_node]: GANTI INIT_POS | init = [0.008, 0.004, 0.019] | current = [0.008, 0.004, 0.019]
[INFO] [1770989217.256860862] [drone1_control_node]: Setting offboard mode...
[INFO] [1770989217.257099440] [drone1_control_node]: Offboard mode command sent
[INFO] [1770989217.357200707] [drone1_control_node]: Arming...
[INFO] [1770989217.357747026] [drone1_control_node]: Arm command sent
[INFO] [1770989227.157254254] [drone1_control_node]: Reached waypoint
[INFO] [1770989232.256467759] [drone1_control_node]: Hold complete at waypoint 0
[INFO] [1770989232.356346079] [drone1_control_node]: GANTI INIT_POS | init = [0.040, -0.014, -2.003] | current = [0.040, -0.014, -2.003]
[INFO] [1770989234.357227672] [drone1_control_node]: Reached waypoint
[INFO] [1770989239.458223805] [drone1_control_node]: Hold complete at waypoint 1
[INFO] [1770989239.556560045] [drone1_control_node]: GANTI INIT_POS | init = [-0.072, 2.001, -2.025] | current = [-0.072, 2.001, -2.025]
[INFO] [1770989241.656734430] [drone1_control_node]: Reached waypoint
[INFO] [1770989246.756860843] [drone1_control_node]: Hold complete at waypoint 2
[INFO] [1770989246.855807198] [drone1_control_node]: GANTI INIT_POS | init = [0.110, -1.011, -2.041] | current = [0.110, -1.011, -2.041]
[INFO] [1770989248.655903177] [drone1_control_node]: Reached waypoint
[INFO] [1770989253.755707481] [drone1_control_node]: Hold complete at waypoint 3
[INFO] [1770989253.856212151] [drone1_control_node]: GANTI INIT_POS | init = [0.013, -0.017, -2.055] | current = [0.013, -0.017, -2.055]
[INFO] [1770989258.655644425] [drone1_control_node]: Reached waypoint
[INFO] [1770989263.655999819] [drone1_control_node]: Hold complete at waypoint 4
[INFO] [1770989263.755222023] [drone1_control_node]: MISSION COMPLETE

ravi@ArtupaysIvar:~/capstone-drone-project/ros2_ws$ ros2 run px4_ros_com offboard_control1
[INFO] [1770989332.976004439] [drone1_control_node]: GANTI INIT_POS | init = [0.000, 0.000, 0.000] | current = [0.000, 0.000, 0.000]
[INFO] [1770989333.975931242] [drone1_control_node]: Setting offboard mode...
[INFO] [1770989333.976047133] [drone1_control_node]: Offboard mode command sent
[INFO] [1770989334.075891855] [drone1_control_node]: Arming...
[INFO] [1770989334.075975837] [drone1_control_node]: Arm command sent
[INFO] [1770989352.575388907] [drone1_control_node]: Reached waypoint
[INFO] [1770989357.675201491] [drone1_control_node]: Hold complete at waypoint 0
[INFO] [1770989357.775184096] [drone1_control_node]: GANTI INIT_POS | init = [-0.018, 0.061, -2.008] | current = [-0.018, 0.061, -2.008]
[INFO] [1770989359.775128605] [drone1_control_node]: Reached waypoint
[INFO] [1770989364.874984167] [drone1_control_node]: Hold complete at waypoint 1
[INFO] [1770989364.974910218] [drone1_control_node]: GANTI INIT_POS | init = [-0.122, 2.094, -2.016] | current = [-0.122, 2.094, -2.016]
[INFO] [1770989367.074967434] [drone1_control_node]: Reached waypoint
[INFO] [1770989372.174818359] [drone1_control_node]: Hold complete at waypoint 2
[INFO] [1770989372.274727816] [drone1_control_node]: GANTI INIT_POS | init = [0.054, -0.909, -2.015] | current = [0.054, -0.909, -2.015]
[INFO] [1770989374.074720674] [drone1_control_node]: Reached waypoint
[INFO] [1770989379.174589090] [drone1_control_node]: Hold complete at waypoint 3
[INFO] [1770989379.274596345] [drone1_control_node]: GANTI INIT_POS | init = [-0.001, 0.088, -2.013] | current = [-0.001, 0.088, -2.013]
[INFO] [1770989383.874341119] [drone1_control_node]: Reached waypoint
[INFO] [1770989388.974231287] [drone1_control_node]: Hold complete at waypoint 4
[INFO] [1770989389.074263819] [drone1_control_node]: MISSION COMPLETE

ravi@ArtupaysIvar:~/capstone-drone-project/ros2_ws$ ros2 run px4_ros_com offboard_control1
[INFO] [1770989436.679061479] [drone1_control_node]: GANTI INIT_POS | init = [-0.190, 10.079, -0.057] | current = [-0.190, 10.079, -0.057]
[INFO] [1770989437.678645870] [drone1_control_node]: Setting offboard mode...
[INFO] [1770989437.678827929] [drone1_control_node]: Offboard mode command sent
[INFO] [1770989437.778628189] [drone1_control_node]: Arming...
[INFO] [1770989437.778777546] [drone1_control_node]: Arm command sent
[INFO] [1770989447.278567681] [drone1_control_node]: Reached waypoint
[INFO] [1770989452.378339654] [drone1_control_node]: Hold complete at waypoint 0
[INFO] [1770989452.478367031] [drone1_control_node]: GANTI INIT_POS | init = [-0.209, 10.087, -2.050] | current = [-0.209, 10.087, -2.050]
[INFO] [1770989454.478245999] [drone1_control_node]: Reached waypoint
[INFO] [1770989459.578104286] [drone1_control_node]: Hold complete at waypoint 1
[INFO] [1770989459.678296394] [drone1_control_node]: GANTI INIT_POS | init = [-0.209, 12.124, -2.053] | current = [-0.209, 12.124, -2.053]
[INFO] [1770989461.778121759] [drone1_control_node]: Reached waypoint
[INFO] [1770989466.778541578] [drone1_control_node]: Hold complete at waypoint 2
[INFO] [1770989466.877899087] [drone1_control_node]: GANTI INIT_POS | init = [-0.140, 9.110, -2.061] | current = [-0.140, 9.110, -2.061]
[INFO] [1770989468.678318378] [drone1_control_node]: Reached waypoint
[INFO] [1770989473.777775923] [drone1_control_node]: Hold complete at waypoint 3
[INFO] [1770989473.877771380] [drone1_control_node]: GANTI INIT_POS | init = [-0.177, 10.090, -2.070] | current = [-0.177, 10.090, -2.070]
[INFO] [1770989477.877609129] [drone1_control_node]: Reached waypoint
[INFO] [1770989482.977922978] [drone1_control_node]: Hold complete at waypoint 4
[INFO] [1770989483.077611971] [drone1_control_node]: MISSION COMPLETE

RUN 2:
ravi@ArtupaysIvar:~/capstone-drone-project/ros2_ws$ ros2 run px4_ros_com offboard_control1
[INFO] [1770990703.316960848] [drone1_control_node]: GANTI INIT_POS | WP 0 | body_wp = [0.00, 0.00, -2.00] | init = [-0.066, 0.015, 0.086] | current = [-0.066, 0.015, 0.086]
[INFO] [1770990704.316161034] [drone1_control_node]: Setting offboard mode...
[INFO] [1770990704.316315579] [drone1_control_node]: Offboard mode command sent
[INFO] [1770990704.416163865] [drone1_control_node]: Arming...
[INFO] [1770990704.416797408] [drone1_control_node]: Arm command sent
[INFO] [1770990713.316611569] [drone1_control_node]: Reached waypoint
[INFO] [1770990718.416382149] [drone1_control_node]: Hold complete at waypoint 0
[INFO] [1770990718.516441429] [drone1_control_node]: GANTI INIT_POS | WP 1 | body_wp = [2.00, 0.00, 0.00] | init = [-0.072, 0.003, -1.948] | current = [-0.072, 0.003, -1.948]
[INFO] [1770990720.516545901] [drone1_control_node]: Reached waypoint
[INFO] [1770990725.616164430] [drone1_control_node]: Hold complete at waypoint 1
[INFO] [1770990725.716597024] [drone1_control_node]: GANTI INIT_POS | WP 2 | body_wp = [-3.00, 0.00, 0.00] | init = [-0.180, 2.010, -1.989] | current = [-0.180, 2.010, -1.989]
[INFO] [1770990727.916616859] [drone1_control_node]: Reached waypoint
[INFO] [1770990733.016135193] [drone1_control_node]: Hold complete at waypoint 2
[INFO] [1770990733.116155091] [drone1_control_node]: GANTI INIT_POS | WP 3 | body_wp = [1.00, 0.00, 0.00] | init = [-0.029, -0.971, -2.022] | current = [-0.029, -0.971, -2.022]
[INFO] [1770990734.916150495] [drone1_control_node]: Reached waypoint
[INFO] [1770990740.016094123] [drone1_control_node]: Hold complete at waypoint 3
[INFO] [1770990740.116091643] [drone1_control_node]: GANTI INIT_POS | WP 4 | body_wp = [5.00, 0.00, -2.00] | init = [-0.076, 0.027, -2.047] | current = [-0.076, 0.027, -2.047]
[INFO] [1770990743.516232101] [drone1_control_node]: Reached waypoint
[INFO] [1770990748.516479269] [drone1_control_node]: Hold complete at waypoint 4
[INFO] [1770990748.616073744] [drone1_control_node]: MISSION COMPLETE | final position = [-0.247, 4.978, -4.060]
^C[INFO] [1770990757.017956311] [rclcpp]: signal_handler(SIGINT/SIGTERM)

ravi@ArtupaysIvar:~/capstone-drone-project/ros2_ws$ ros2 run px4_ros_com offboard_control1
[INFO] [1770990776.650287979] [drone1_control_node]: GANTI INIT_POS | WP 0 | body_wp = [0.00, 0.00, -2.00] | init = [-0.197, 5.002, 0.017] | current = [-0.197, 5.002, 0.017]
[INFO] [1770990777.650084318] [drone1_control_node]: Setting offboard mode...
[INFO] [1770990777.650232197] [drone1_control_node]: Offboard mode command sent
[INFO] [1770990777.750093354] [drone1_control_node]: Arming...
[INFO] [1770990777.750272005] [drone1_control_node]: Arm command sent
[INFO] [1770990787.050203266] [drone1_control_node]: Reached waypoint
[INFO] [1770990792.150108960] [drone1_control_node]: Hold complete at waypoint 0
[INFO] [1770990792.250087367] [drone1_control_node]: GANTI INIT_POS | WP 1 | body_wp = [2.00, 0.00, 0.00] | init = [-0.186, 5.030, -2.004] | current = [-0.186, 5.030, -2.004]
[INFO] [1770990794.250113929] [drone1_control_node]: Reached waypoint
[INFO] [1770990799.350082812] [drone1_control_node]: Hold complete at waypoint 1
[INFO] [1770990799.450085227] [drone1_control_node]: GANTI INIT_POS | WP 2 | body_wp = [-3.00, 0.00, 0.00] | init = [-0.212, 7.049, -2.019] | current = [-0.212, 7.049, -2.019]
[INFO] [1770990801.550047843] [drone1_control_node]: Reached waypoint
[INFO] [1770990806.650028167] [drone1_control_node]: Hold complete at waypoint 2
[INFO] [1770990806.750019376] [drone1_control_node]: GANTI INIT_POS | WP 3 | body_wp = [1.00, 0.00, 0.00] | init = [-0.172, 4.044, -2.033] | current = [-0.172, 4.044, -2.033]
[INFO] [1770990808.650352377] [drone1_control_node]: Reached waypoint
[INFO] [1770990813.750034016] [drone1_control_node]: Hold complete at waypoint 3
[INFO] [1770990813.850140284] [drone1_control_node]: GANTI INIT_POS | WP 4 | body_wp = [5.00, 0.00, -2.00] | init = [-0.189, 5.045, -2.049] | current = [-0.189, 5.045, -2.049]
[INFO] [1770990818.149885410] [drone1_control_node]: Reached waypoint
[INFO] [1770990823.149947860] [drone1_control_node]: Hold complete at waypoint 4
[INFO] [1770990823.249927884] [drone1_control_node]: MISSION COMPLETE | final position = [-0.216, 9.990, -4.053]
^C[INFO] [1770990833.799998428] [rclcpp]: signal_handler(SIGINT/SIGTERM)

ravi@ArtupaysIvar:~/capstone-drone-project/ros2_ws$ ros2 run px4_ros_com offboard_control1
[INFO] [1770990856.928712566] [drone1_control_node]: GANTI INIT_POS | WP 0 | body_wp = [0.00, 0.00, -2.00] | init = [0.000, 0.000, 0.000] | current = [0.000, 0.000, 0.000]
[INFO] [1770990857.928364443] [drone1_control_node]: Setting offboard mode...
[INFO] [1770990857.928540361] [drone1_control_node]: Offboard mode command sent
[INFO] [1770990858.028384832] [drone1_control_node]: Arming...
[INFO] [1770990858.028720831] [drone1_control_node]: Arm command sent
[INFO] [1770990878.728451200] [drone1_control_node]: Reached waypoint
[INFO] [1770990883.828542885] [drone1_control_node]: Hold complete at waypoint 0
[INFO] [1770990883.928434839] [drone1_control_node]: GANTI INIT_POS | WP 1 | body_wp = [2.00, 0.00, 0.00] | init = [-0.015, 0.029, -2.016] | current = [-0.015, 0.029, -2.016]
[INFO] [1770990885.928364522] [drone1_control_node]: Reached waypoint
[INFO] [1770990891.028444161] [drone1_control_node]: Hold complete at waypoint 1
[INFO] [1770990891.128320371] [drone1_control_node]: GANTI INIT_POS | WP 2 | body_wp = [-3.00, 0.00, 0.00] | init = [-0.017, 2.023, -2.015] | current = [-0.017, 2.023, -2.015]
[INFO] [1770990893.328333037] [drone1_control_node]: Reached waypoint
[INFO] [1770990898.428272265] [drone1_control_node]: Hold complete at waypoint 2
[INFO] [1770990898.528177600] [drone1_control_node]: GANTI INIT_POS | WP 3 | body_wp = [1.00, 0.00, 0.00] | init = [0.046, -0.982, -2.023] | current = [0.046, -0.982, -2.023]
[INFO] [1770990900.428631857] [drone1_control_node]: Reached waypoint
[INFO] [1770990905.528251695] [drone1_control_node]: Hold complete at waypoint 3
[INFO] [1770990905.628397002] [drone1_control_node]: GANTI INIT_POS | WP 4 | body_wp = [5.00, 0.00, -2.00] | init = [0.021, 0.030, -2.037] | current = [0.021, 0.030, -2.037]
[INFO] [1770990910.528446581] [drone1_control_node]: Reached waypoint
[INFO] [1770990915.628347582] [drone1_control_node]: Hold complete at waypoint 4
[INFO] [1770990915.728217115] [drone1_control_node]: MISSION COMPLETE | final position = [-0.015, 4.981, -4.030]
^C[INFO] [1770990926.959071664] [rclcpp]: signal_handler(SIGINT/SIGTERM)

ravi@ArtupaysIvar:~/capstone-drone-project/ros2_ws$ ros2 run px4_ros_com offboard_control1
[INFO] [1770990949.533394750] [drone1_control_node]: GANTI INIT_POS | WP 0 | body_wp = [0.00, 0.00, -2.00] | init = [0.015, 5.001, -0.012] | current = [0.015, 5.001, -0.012]
[INFO] [1770990950.533279564] [drone1_control_node]: Setting offboard mode...
[INFO] [1770990950.533421294] [drone1_control_node]: Offboard mode command sent
[INFO] [1770990950.633309744] [drone1_control_node]: Arming...
[INFO] [1770990950.633399163] [drone1_control_node]: Arm command sent
[INFO] [1770990960.133335079] [drone1_control_node]: Reached waypoint
[INFO] [1770990965.233282687] [drone1_control_node]: Hold complete at waypoint 0
[INFO] [1770990965.333312774] [drone1_control_node]: GANTI INIT_POS | WP 1 | body_wp = [2.00, 0.00, 0.00] | init = [0.010, 4.999, -2.009] | current = [0.010, 4.999, -2.009]
[INFO] [1770990967.333326120] [drone1_control_node]: Reached waypoint
[INFO] [1770990972.433310376] [drone1_control_node]: Hold complete at waypoint 1
[INFO] [1770990972.533302345] [drone1_control_node]: GANTI INIT_POS | WP 2 | body_wp = [-3.00, 0.00, 0.00] | init = [0.039, 7.015, -2.013] | current = [0.039, 7.015, -2.013]
[INFO] [1770990974.633285578] [drone1_control_node]: Reached waypoint
[INFO] [1770990979.733249681] [drone1_control_node]: Hold complete at waypoint 2
[INFO] [1770990979.833256475] [drone1_control_node]: GANTI INIT_POS | WP 3 | body_wp = [1.00, 0.00, 0.00] | init = [0.045, 4.036, -2.007] | current = [0.045, 4.036, -2.007]
[INFO] [1770990981.633233154] [drone1_control_node]: Reached waypoint
[INFO] [1770990986.733255473] [drone1_control_node]: Hold complete at waypoint 3
[INFO] [1770990986.833201026] [drone1_control_node]: GANTI INIT_POS | WP 4 | body_wp = [5.00, 0.00, -2.00] | init = [0.057, 5.020, -2.010] | current = [0.057, 5.020, -2.010]
[INFO] [1770990991.833236400] [drone1_control_node]: Reached waypoint
[INFO] [1770990996.933216012] [drone1_control_node]: Hold complete at waypoint 4
[INFO] [1770990997.033216937] [drone1_control_node]: MISSION COMPLETE | final position = [0.163, 9.970, -4.009]
^C[INFO] [1770991005.104402303] [rclcpp]: signal_handler(SIGINT/SIGTERM)
