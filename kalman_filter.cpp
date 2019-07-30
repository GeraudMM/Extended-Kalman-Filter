#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define PI 3.1416

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
   x_ = F_ * x_ ;
   MatrixXd Ft = F_.transpose();
   P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   VectorXd y = z - H_ * x_;
   MatrixXd Ht = H_.transpose();
   MatrixXd S = H_ * P_ * Ht + R_;
   MatrixXd Si = S.inverse();
   MatrixXd K =  P_ * Ht * Si;

   // new state
   x_ = x_ + (K * y);
   MatrixXd I = MatrixXd::Identity( x_.size(),  x_.size());
   P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   if (fabs(x_(0)) < 0.0001 and fabs(x_(1)) < 0.0001){
    	return;
   }
   float rho = sqrt( x_(0) * x_(0) + x_(1) * x_(1) );
   float phi = atan2( x_(1), x_(0) );
   float rho_dot = ( x_(0) * x_(2) + x_(1) * x_(3) ) / rho;
   VectorXd h_x = VectorXd(3);
   h_x << rho, phi, rho_dot;

   VectorXd y = z - h_x;
   //y[1] = (y[1] + PI)%(2*PI) -PI;
   while (y[1]< -PI){
     y[1] += 2*PI;
   }
   while (y[1]> PI){
     y[1] -=2*PI;
   }
   MatrixXd Ht = H_.transpose();
   MatrixXd S = H_ * P_ * Ht + R_;
   MatrixXd Si = S.inverse();
   MatrixXd K =  P_ * Ht * Si;

   // new state
   x_ = x_ + (K * y);
   MatrixXd I = MatrixXd::Identity( x_.size(),  x_.size());
   P_ = (I - K * H_) * P_;
}
