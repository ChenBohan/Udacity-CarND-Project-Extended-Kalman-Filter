# Udacity-CarND-Project-Extended-Kalman-Filter
Implementation of extended Kalman filter for sensor fusion with RADAR and LIDAR sensors in a self driving car.

### Logic

- Rececive measurement data
  - if Lidar
    ```cpp
    meas_package.sensor_type_ = MeasurementPackage::LASER;
    meas_package.raw_measurements_ = VectorXd(2);
    float px, py;
    iss >> px;
    iss >> py;
    meas_package.raw_measurements_ << px, py;
    ```
  - if Radar
    ```cpp
    meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = VectorXd(3);
    float ro, theta, ro_dot;
    iss >> ro;
    iss >> theta;
    iss >> ro_dot;
    meas_package.raw_measurements_ << ro,theta, ro_dot;
    ```
- Call Kalman filter
  - Initialization
    - EKF init with LASER data
      - Initialize state
      ```cpp
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);   // x
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);   // y
      // We don't know velocities from the first measurement of the LIDAR, so, we use zeros
      ```
    - EKF init with RADAR data
      - Convert radar from polar to cartesian coordinates
      - Initialize state
      ```cpp
      ekf_.x_(0) = rho     * cos(phi);    // x
      ekf_.x_(1) = rho     * sin(phi);    // y 
      ekf_.x_(2) = rho_dot * cos(phi);    // vx
      ekf_.x_(3) = rho_dot * sin(phi);    // vy
      ```
  - Prediction
    - State transition matrix update
      ```
      ekf_.F_ = MatrixXd(4, 4);
      ekf_.F_ << 1, 0, dt, 0,
                 0, 1, 0, dt,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
      ```
    - Process covariance matrix update
      ```
      ekf_.Q_ = MatrixXd(4, 4);
      ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
                 0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
                 dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
                 0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;
      ```
    - Predict the state
      ```cpp
      x_ = F_ * x_ ;
      MatrixXd Ft = F_.transpose();
      P_ = F_ * P_ * Ft + Q_;
      ```
  - Update
    - if 
