#include <kalman_filter.hpp>

KalmanFilter::KalmanFilter() {

}
void KalmanFilter::initialize(const Quatd& q) {
	F_ << 0, 0, 0, 0, 0, 0, 0, 0;
}
void KalmanFilter::predict(const Quatd& U) {
	X_hat_ = X_ * U;
	P_ = F_ * P_ * F_.transpose() + Q_;
}
void KalmanFilter::update() {

}