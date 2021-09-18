说明:

1. EKF需要用到的光流信息在 /ekf/opti_tf_odom 中，point.x、point.y对应的是x、y轴速度，point.z对应高度，详细看optical_flow源码;

2.EKF只需要运行光流包，使用/ekf/opti_tf_odom信息融合;

3.bag里有motion capture真值，是world frame，tag得到的结果是tag frame，EKF结果需要在world frame下显示，在rviz里下对比EKF位姿与mocap位姿;

4.bag里有一部分时间是看不到tag的，这时候需要用光流去估计，在有tag的时候，需要融合tag和光流，没有tag的时候，只融合光流，最后在rviz里下对比各个时刻下EKF位姿与mocap位姿。




附:

/* camera to imu */

  /* 之前的bag */
  // Tic_ << 1, 0, 0, 0.05, 0, -1, 0, 0.05, 0, 0, -1, 0, 0, 0, 0, 1;


  /* ekf_flow.bag */
  Tic_ << 0, -1, 0, -0.1, -1, 0, 0, 0.0, 0, 0, -1, -0.03, 0, 0, 0, 1;
  /* tag to world */
  Twt_ << 0, +1, 0, -0.55, +1, 0, 0, -0.3, 0, 0, -1, 0, 0, 0, 0, 1;