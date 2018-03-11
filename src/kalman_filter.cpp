#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


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
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateUsingY(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size); // @suppress("Invalid arguments")
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateUsingY(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho, theta, rho_dot;
  theta = atan2(py,px);
  if(px != 0 && py != 0){
      rho_dot = (px*vx+py*vy)/sqrt(pow(px,2)+pow(py,2));
  }
  rho  = sqrt(pow(px,2)+pow(py,2));

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;

  VectorXd y = z - z_pred;

  //normalize theta to be between -pi and pi
  double twoPi = 2 * M_PI;
  double corr = (y[1] + M_PI) /twoPi;
  corr = y[1] - twoPi * floor(corr);
  y[1] = corr;

  UpdateUsingY(y);
 }
