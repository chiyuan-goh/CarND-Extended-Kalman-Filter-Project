#include "kalman_filter.h"
#include "tools.h"

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

void KalmanFilter::Predict() {
    x_ = F_ * x_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd y = z - (H_ * x_);
    MatrixXd S = H_* P_  * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    MatrixXd I = MatrixXd::Identity(K.rows(), H_.cols());

    x_ += K * y;
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    //MatrixXd jacob = Tools::CalculateJacobian(x_);
    VectorXd hx(3);
    double x = x_(0), y = x_(1), vx = x_(2), vy = x_(3);
    double term = sqrt(x * x + y * y);
    hx << term, atan2(y, x), (x * vx  + y * vy) / term;
    VectorXd Y = z - hx;

    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    MatrixXd I = MatrixXd::Identity(K.rows(), H_.cols());

    x_ += K * Y;
    P_ = (I - K * H_) * P_;
}
