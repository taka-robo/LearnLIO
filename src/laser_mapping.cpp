#include <ros/ros.h>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kalman_filter.hpp>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

// void lidarCallBack(const livox_ros_driver::CustomMsg::ConstPtr& msg) {

// }
std::unique_ptr<KalmanFilter> kf;
void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg) {
	// msg->header.
	const double omega_x = msg->angular_velocity.x;
	const double omega_y = msg->angular_velocity.y;
	const double omega_z = msg->angular_velocity.z;
	const double dt = 0.01;
	kf->filter(dt, omega_x, omega_y, omega_z);
}


int main(int argc, char** argv)
{
	// ROSノードの初期化、"laserMapping"という名前でノードを登録
	ros::init(argc, argv, "laser_mapping");
	ros::NodeHandle nh;
	// ROSのサブスクライバーとパブリッシャーの初期化
	// LiDARとIMUからのデータをサブスクライブし、結果をパブリッシュする
	// ros::Subscriber subLidarCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, lidarCallBack);
	ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, imuCallBack);
	kf.reset(new KalmanFilter());
	//初期姿勢
	Eigen::Quaterniond init_pose(1, 0, 0, 0);
	kf->initialize(init_pose);
	// ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
// 	("/Laser_map", 100000);
// ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>
// 	("/Odometry", 100000);
// ros::Publisher pubPath = nh.advertise<nav_msgs::Path>
// 	("/path", 100000);
	while (!ros::ok()) {
		//特徴抽出モジュール
		//状態推定モジュール(IMUとLidar特徴量を利用)
	}
}