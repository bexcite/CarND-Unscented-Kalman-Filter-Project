#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");
Eigen::IOFormat fmtm(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "", "");

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initialize sizes & lambda
  n_x_ = 5;
  n_aug_ = n_x_ + 2;
  lambda_ = 3 - n_aug_;

  // Generate weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  double w = 0.5 / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
    weights_(i) = w;
  }

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // initialize Predicted Sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.7;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // Initialize Radar noise matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0,                         0,
              0,                     std_radphi_ * std_radphi_, 0,
              0,                     0,                         std_radrd_ * std_radrd_;

  // Initialize Lidar noise matrix
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_ * std_laspx_, 0,
              0,                       std_laspy_;

  is_initialized_ = false;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  // Initialize on first measurement
  if (!is_initialized_) {

    x_.setZero();

    float v_var = 1000;
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(0);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      float rho_dot = meas_package.raw_measurements_(2);
      x_(0) = rho * cos(phi);
      x_(1) = rho * sin(phi);
      x_(2) = rho_dot;
      v_var = 0.5;
    }

    P_ << 0.5,     0,   0,       0,     0,
            0,   0.5,   0,       0,     0,
            0,     0,   v_var,   0,     0,
            0,     0,   0,     100,     0,
            0,     0,   0,       0,   100;

    std::cout << "First Measurement Initialized:" << std::endl;
    std::cout << "x_ = " << x_.format(fmt) << std::endl;
    std::cout << "P_ = " << P_ << std::endl;

    time_us_ = meas_package.timestamp_;

    is_initialized_ = true;
    return;

  }

  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  std::cout << "delta_t = " << delta_t << " secs" << std::endl;

  Prediction(delta_t);

  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  }

  std::cout << "P_ = " << std::endl << P_.format(fmtm) << std::endl;
  std::cout << "x_ = " << x_.format(fmt) << std::endl;

//  std::cout << "<<<<<<< end update <<< " << std::endl;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */


  // Augment state vector
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);



  // Augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_ + 1) = 0;

//  std::cout << "x_aug = " << x_aug.format(fmt) << std::endl;


  // Augment covariance matrix
  P_aug.setZero();
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

//  std::cout << "P_aug = " << std::endl << P_aug.format(fmtm) << std::endl;

  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_aug_ * 2 +1);


  MatrixXd A = P_aug.llt().matrixL();
  A = sqrt(lambda_ + n_aug_) * A;

//  std::cout << "A = " << std::endl << A.format(fmtm) << std::endl;

  Xsig_aug.col(0) = x_aug;

  for (int c = 0; c < n_aug_; ++c) {
    Xsig_aug.col(c + 1) = x_aug + A.col(c);
    Xsig_aug.col(n_aug_ + c + 1) = x_aug - A.col(c);
  }



//  std::cout << "Xsig_aug(0-7) = " << std::endl
//            << Xsig_aug.block(0, 1, n_aug_, n_aug_).format(fmtm) << std::endl;


  // Normalize angles
  while (Xsig_aug(3) < -M_PI) Xsig_aug(3) += M_PI;
  while (Xsig_aug(3) > M_PI) Xsig_aug(3) -= M_PI;


  // Sigma Point Prediction
  double delta_t2 = delta_t * delta_t;

  // Predicted state
  VectorXd x(n_x_);
  x.setZero();

  for (int c = 0; c < 2 * n_aug_ + 1; ++c) {
    VectorXd sp = Xsig_aug.col(c);
    double v = sp[2];
    double psi = sp[3];
    double psi_dot = sp[4];
    double std_a = sp[5];
    double std_yawdd = sp[6];


    // Predict Sigma Point

    VectorXd s_pred(n_x_);
    s_pred.setZero();

    if (abs(psi_dot) < 1e-05) {
      s_pred(0) = sp(0) + v * cos(psi) * delta_t;
      s_pred(1) = sp(1) + v * sin(psi) * delta_t;
    } else {
      s_pred(0) = sp(0) + v / psi_dot * ( sin(psi + psi_dot * delta_t) - sin(psi) );
      s_pred(1) = sp(1) + v / psi_dot * ( -cos(psi + psi_dot * delta_t) + cos(psi) );
    }

    // Add noise component
    s_pred(0) += 0.5 * delta_t2 * cos(psi) * std_a;
    s_pred(1) += 0.5 * delta_t2 * sin(psi) * std_a;

    s_pred(2) = sp(2) + delta_t * std_a;
    s_pred(3) = sp(3) + delta_t * psi_dot + 0.5 * delta_t2 * std_yawdd;
    s_pred(4) = sp(4) + delta_t * std_yawdd;

    // Normalize angles
    while (s_pred(3) < -M_PI) s_pred(3) += M_PI;
    while (s_pred(3) > M_PI) s_pred(3) -= M_PI;

    // << End - Predict Sigma Point

//    std::cout << "s_pred = " << std::endl << s_pred << std::endl;

    Xsig_pred_.col(c) = s_pred;

    // Predict Mean
    x += weights_(c) * s_pred;

  }

  // Normalize angles
  while (x(3) < -M_PI) x(3) += M_PI;
  while (x(3) > M_PI) x(3) -= M_PI;

//  std::cout << "Xsig_pred = " << std::endl << Xsig_pred_.format(fmtm) << std::endl;

//  std::cout << "x = " << x.format(fmt) << std::endl;


  // Predict Covariance
  MatrixXd P(n_x_, n_x_);
  P.setZero();

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd diffVec = Xsig_pred_.col(i) - x;

    // Normalize angles
    while (diffVec(3) < -M_PI) diffVec(3) += M_PI;
    while (diffVec(3) > M_PI) diffVec(3) -= M_PI;

    P += weights_(i) * (diffVec * diffVec.transpose());
  }

//  std::cout << "P_predicted = " << std::endl << P.format(fmtm) << std::endl;

  // TODO: - make calculations directly on x_ and P_
  x_ = x;
  P_ = P;

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  std::cout << "---- UpdateLidar ---- " << std::endl;

  int n_z = 2;

  // Measurement vector
  VectorXd z(n_z);
  z = meas_package.raw_measurements_;

  // Predicted Measurement sigma points
  MatrixXd Zsig_pred(n_z, 2 * n_aug_ + 1);

  // Predicted Measurement mean
  VectorXd Z_pred(n_z);
  Z_pred.setZero();

  // Predict Sigma points measurements
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd sp = Xsig_pred_.col(i);
    double px = sp(0);
    double py = sp(1);

    Zsig_pred.col(i) << px, py;

    Z_pred += weights_(i) * Zsig_pred.col(i);
  }

//  std::cout << "Zsig = " << std::endl << Zsig_pred.format(fmtm) << std::endl;
  std::cout << "Z_pred = " << Z_pred.format(fmt) << std::endl;

  // Predicted measurement covariance
  MatrixXd S(n_z, n_z);

  S.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd diffVec = Zsig_pred.col(i) - Z_pred;

    S += weights_(i) * (diffVec * diffVec.transpose());
  }

  // Add measurement noise
  S += R_lidar_;

//  std::cout << "S = " << std::endl << S.format(fmtm) << std::endl;

  // calculate cross correlation matrix
  MatrixXd Tc(n_x_, n_z);
  Tc.setZero();

  for (int i = 0; i < 2* n_aug_ + 1; ++i) {
    VectorXd diffX = Xsig_pred_.col(i) - x_;
    VectorXd diffZ = Zsig_pred.col(i) - Z_pred;
    Tc += weights_(i) * diffX * diffZ.transpose();
  }

//  std::cout << "Tc = " << std::endl << Tc.format(fmtm) << std::endl;

  // Calculate Kalman gain
  MatrixXd Si = S.inverse();
  MatrixXd K = Tc * Si;

  std::cout << "K = " << std::endl << K.format(fmtm) << std::endl;

  // Update state and covariance
  x_ += K * (z - Z_pred);
  P_ -= K * S * K.transpose();

  std::cout << " <<<<< UpdateLidar ENDS ---- " << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  std::cout << "---- UpdateRadar ---- " << std::endl;

  int n_z = 3;

  // Measurement vector
  VectorXd z(n_z);
  z = meas_package.raw_measurements_;

  // Normalize angles
  while (z(1) < -M_PI) z(1) += M_PI;
  while (z(1) > M_PI) z(1) -= M_PI;

  // Measurement sigma points
  MatrixXd Zsig_pred(n_z, 2 * n_aug_ + 1);

  // Measurement mean
  VectorXd Z_pred(n_z);
  Z_pred.setZero();

  // Predict Sigma points measurements
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd sp = Xsig_pred_.col(i);
    double px = sp(0);
    double py = sp(1);
    double v = sp(2);
    double psi = sp(3);

    double rho = sqrt(px * px + py * py);
    double phi = atan2(py, px);
    double rho_dot = (px * cos(psi) * v + py * sin(psi) * v);
    if (rho < 1e-03) {
      rho_dot = rho_dot / 1e-03;
    } else {
      rho_dot = rho_dot / rho;
    }

    Zsig_pred.col(i) << rho, phi, rho_dot;

    Z_pred += weights_(i) * Zsig_pred.col(i);
  }

  // Normalize angles
  while (Z_pred(1) < -M_PI) Z_pred(1) += M_PI;
  while (Z_pred(1) > M_PI) Z_pred(1) -= M_PI;


//  std::cout << "Zsig = " << std::endl << Zsig_pred.format(fmtm) << std::endl;
  std::cout << "Z_pred = " << Z_pred.format(fmt) << std::endl;

  // Predicted measurement covariance
  MatrixXd S(n_z, n_z);

  S.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd diffVec = Zsig_pred.col(i) - Z_pred;

    // Normalize angles
    while (diffVec(1) < -M_PI) diffVec(1) += M_PI;
    while (diffVec(1) > M_PI) diffVec(1) -= M_PI;

    S += weights_(i) * (diffVec * diffVec.transpose());
  }

  // Add measurement noise
  S += R_radar_;

//  std::cout << "S = " << std::endl << S.format(fmtm) << std::endl;

  // calculate cross correlation matrix
  MatrixXd Tc(n_x_, n_z);
  Tc.setZero();

  for (int i = 0; i < 2* n_aug_ + 1; ++i) {

    VectorXd diffX = Xsig_pred_.col(i) - x_;
    // Normalize angles
    while (diffX(3) < -M_PI) diffX(3) += M_PI;
    while (diffX(3) > M_PI) diffX(3) -= M_PI;

    VectorXd diffZ = Zsig_pred.col(i) - Z_pred;
    // Normalize angles
    while (diffZ(1) < -M_PI) diffZ(1) += M_PI;
    while (diffZ(1) > M_PI) diffZ(1) -= M_PI;

    Tc += weights_(i) * diffX * diffZ.transpose();
  }

//  std::cout << "Tc = " << std::endl << Tc.format(fmtm) << std::endl;

  // Calculate Kalman gain
  MatrixXd Si = S.inverse();
  MatrixXd K = Tc * Si;

  std::cout << "K = " << std::endl << K.format(fmtm) << std::endl;

  // Update state and covariance
  x_ += K * (z - Z_pred);

  // Normalize angles
  while (x_(3) < -M_PI) x_(3) += M_PI;
  while (x_(3) > M_PI) x_(3) -= M_PI;

  P_ -= K * S * K.transpose();

  std::cout << " <<<<< UpdateRadar ENDS ---- " << std::endl;


}
