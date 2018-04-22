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

  VectorXd u = VectorXd(2);
  u << 0, 0;
}


// Same function for laser & radar
void KalmanFilter::Predict() {
  /**
     TODO:
     * predict the state
     */

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
     TODO:
     * update the state by using Kalman Filter equations
     */
  MatrixXd I = MatrixXd::Identity(2, 2);
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  /*
   * KF Prediction step
   */
  x_ = F_ * x_ + u;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

  //new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
     TODO:
     * update the state by using Extended Kalman Filter equations
     */
}
