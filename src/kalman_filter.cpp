#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    x_ = F_*x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd y = z - H_*x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(4, 4);
    P_ = (I - K * H_) * P_;
    
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    float px = x_[0];
    float py = x_[1];
    float vx = x_[2];
    float vy = x_[3];
    
    VectorXd h = VectorXd(3);
    h << sqrt(px*px + py*py),
        atan2(py, px),
        (px*vx+py*vy)/sqrt(px*px + py*py);
    
    VectorXd y = z - h;
    y[1] = atan2(sin(y[1]), cos(y[1]));
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(4, 4);
    P_ = (I - K * H_) * P_;
    
}
