#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
// #pragma once
using Quatd = Eigen::Quaterniond;
class KalmanFilter {
public:
	KalmanFilter();
	// ~KalmanFilter();
	void initialize(const Quatd& q);
	void filter(const double dt, const double w1, const double w2, const double w3);
	void predict(const double dt, const double w1, const double w2, const double w3);
	void update();
private:
	Eigen::MatrixXd calcF(const double dt, const double w1, const double w2, const double w3);
	Eigen::MatrixXd calcH();
	Quatd X_;//状態ベクトル(姿勢)
	Quatd X_hat_;//状態ベクトルの推定値(姿勢)
	Quatd sigma_;
	// Eigen::VectorXd state_; // 状態ベクトル
	Eigen::MatrixXd F_; // 誤差共分散行列
	Eigen::MatrixXd P_; // 誤差共分散行列
	Eigen::MatrixXd Q_; // 誤差共分散行列
	Eigen::MatrixXd R_; // 誤差共分散行列
	Eigen::MatrixXd C_; // 誤差共分散行列
	Eigen::MatrixXd processNoise_; // プロセスノイズ
	Eigen::MatrixXd measurementNoise_; // 観測ノイズ

	// // シグマポイントの生成
	// void generateSigmaPoints();

	// // シグマポイントの伝播
	// void propagateSigmaPoints();

	// // 状態と誤差共分散の再計算
	// void updateStateAndCovariance();
};
#endif