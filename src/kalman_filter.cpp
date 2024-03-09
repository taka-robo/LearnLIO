#include <kalman_filter.hpp>

KalmanFilter::KalmanFilter() {

}
void KalmanFilter::initialize(const Quatd& q) {
	F_ << 0, 0, 0, 0, 0, 0, 0, 0;
}
void KalmanFilter::filter(const double dt, const double w1, const double w2, const double w3) {
	predict(dt, w1, w2, w3);
	update();
}
void KalmanFilter::predict(const double dt, const double w1, const double w2, const double w3) {
	F_ = calcF(dt, w1, w2, w3);
	X_hat_ = F_ * X_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}
void KalmanFilter::update() {
	Eigen::MatrixXd h = calcH();
	Eigen::MatrixXd g = P_ * C_ * (C_.transpose() * P_ * C_ + R_).inverse();
	X_ = X_hat_ + g * (y - h);
	P_ = P_ - g * C_.transpose() * P_;
}
Eigen::MatrixXd KalmanFilter::calcF(const double dt, const double w1, const double w2, const double w3) {
	Eigen::MatrixXd F(4, 4);
	F << 1, 0.5 * dt * w3, -0.5 * dt * w2, 0.5 * dt * w1, //
		-0.5 * dt * w3, 1, 0.5 * dt * w1, 0.5 * dt * w2,//
		0.5 * dt * w2, -0.5 * dt * w1, 1, 0.5 * dt * w3,//
		-0.5 * dt * w1, -0.5 * dt * w2, -0.5 * dt * w3, 1;
	return F;
}
Eigen::MatrixXd KalmanFilter::calcH() {
	Eigen::MatrixXd H(4, 4);
	return H;
}
