# Udacity-CarND-Project-Extended-Kalman-Filter
Implementation of extended Kalman filter for sensor fusion with RADAR and LIDAR sensors in a self driving car.

The project's [GitHub repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) (included within the workspace) contains all of the files that you will need. 

## Logic

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
    - Lidar updates
      ```cpp
      H_laser_ << 1, 0, 0, 0,
                  0, 1, 0, 0;
      R_laser_ << 0.0225, 0,
                  0, 0.0225;
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
      ```
        - Update KF
          - Calculate y
            ```cpp
            VectorXd y = z - H_ * x_;
            ```
          - Update With Y
            - Kalman gain
              ```cpp
              MatrixXd Ht_ = H_.transpose();
              MatrixXd K = P_ * Ht_ * (H_ * P_ * Ht_ + R_).inverse();
              ```
            - New estimate
              ```cpp
              x_ = x_ + (K * y);
              int x_size = x_.size();
              MatrixXd I = MatrixXd::Identity(x_size, x_size);
              P_ = (I - K * H_) * P_;
              ```
    - Radar updates
      ```cpp
      R_radar_ << 0.09, 0, 0,
                  0, 0.0009, 0,
                  0, 0, 0.09;
      ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.R_ = R_radar_;
  	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
      ```
        - Calculate Jacobian
          ```cpp
          float c1 = px*px+py*py;
          float c2 = sqrt(c1);
          float c3 = (c1*c2);
          Hj << (px/c2), (py/c2), 0, 0,
                 -(py/c1), (px/c1), 0, 0,
                 py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
          ```
        - UpdateEKF
          - Recalculate x object state to rho, theta, rho_dot coordinates
            ```cpp
              double rho = sqrt(px * px + py * py);
              double theta = atan2(py, px);
              double rho_dot = (px * vx + py * vy) / rho;
              VectorXd h = VectorXd(3);
              h << rho, theta, rho_dot;
              VectorXd y = z - h;
              UpdateWithY(y);
            ```
          - Update With Y
            - Kalman gain
              ```cpp
              MatrixXd Ht_ = H_.transpose();
              MatrixXd K = P_ * Ht_ * (H_ * P_ * Ht_ + R_).inverse();
              ```
            - New estimate
              ```cpp
              x_ = x_ + (K * y);
              int x_size = x_.size();
              MatrixXd I = MatrixXd::Identity(x_size, x_size);
              P_ = (I - K * H_) * P_;
              ``` 
