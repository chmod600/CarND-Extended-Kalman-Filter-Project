#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

// Same function for laser & radar
void KalmanFilter::Predict() {
  /**
     TODO:
     * predict the state
     */

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

// This is for Lidar
void KalmanFilter::Update(const VectorXd &z) {
  /**
     TODO:
     * update the state by using Kalman Filter equations
     */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  // New state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
     TODO:
     * update the state by using Extended Kalman Filter equations
     */

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px * px + py * py);
  float theta = atan2(py, px);
  float rho_dot = (px * vx + py * vy) / rho;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;

  VectorXd y = z - z_pred;

  // Normalize the angle, y(1)
  double width = 2 * M_PI;
  double offsetValue = y(1) + M_PI;
  y(1) = (offsetValue - (floor(offsetValue / width) * width)) - M_PI;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  // New state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}
