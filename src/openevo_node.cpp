#include "openevo/evo.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Imu.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <cmath>

#include <fstream>
#include <chrono>

namespace{

openevo::EVO evo;
image_geometry::PinholeCameraModel model;

tf::TransformBroadcaster *tf_bcaster;
ros::Publisher odom_pub;

cv::Mat R_cam_imu;
cv::Mat imu_bias(cv::Mat::zeros(3, 1, CV_64F));
int imu_bias_samples;

auto time_start = std::chrono::high_resolution_clock::now();
std::ofstream time_log;

template<class T>
void rvec_to_quat(cv::Mat &rvec, T &quat) {
	float angle;
	cv::Mat axis;
	angle = cv::norm(rvec);
	axis = rvec / angle;
	quat.w = std::cos(angle / 2);
	quat.x = axis.at<double>(0, 0) * std::sin(angle / 2);
	quat.y = axis.at<double>(1, 0) * std::sin(angle / 2);
	quat.z = axis.at<double>(2, 0) * std::sin(angle / 2);
}

template<class T>
void vec_to_xyz(cv::Mat &vec, T &xyz) {
	xyz.x = vec.at<double>(0, 0);
	xyz.y = vec.at<double>(1, 0);
	xyz.z = vec.at<double>(2, 0);
}

template<class T>
void xyz_to_vec(const T &xyz, cv::Mat &vec) {
	vec.at<double>(0, 0) = xyz.x;
	vec.at<double>(1, 0) = xyz.y;
	vec.at<double>(2, 0) = xyz.z;
}

void on_imu(const sensor_msgs::Imu &imu) {
	cv::Mat rates(3, 1, CV_64F);
	xyz_to_vec(imu.angular_velocity, rates);
	if(imu_bias_samples < 100) {
		imu_bias += rates / 100.0;
		++imu_bias_samples;
		return;
	}
	rates -= imu_bias;
	rates = R_cam_imu * rates;
	evo.updateIMU(rates, imu.header.stamp.toSec());
}

void on_image(
		const sensor_msgs::ImageConstPtr &color,
		const sensor_msgs::ImageConstPtr &depth,
		const sensor_msgs::CameraInfoPtr &caminfo) {
  auto time_on_image = std::chrono::high_resolution_clock::now();
	// Update camera information
	model.fromCameraInfo(caminfo);
	// Convert images to openCV
	cv_bridge::CvImageConstPtr colorbr = cv_bridge::toCvShare(color, "mono8");
	cv_bridge::CvImageConstPtr depthbr = cv_bridge::toCvShare(depth);
	// Update EVO
	cv::Mat intr;
	static_cast<cv::Mat>(model.projectionMatrix()).copyTo(intr); // Note: intrinsic matrix is 0 on SLAMDunk...
	intr = intr(cv::Range(0, 3), cv::Range(0, 3));
	auto time_before_evo = std::chrono::high_resolution_clock::now();
	evo.updateImageDepth(colorbr->image, depthbr->image, intr,
			caminfo->header.stamp.toSec());
	auto time_after_evo = std::chrono::high_resolution_clock::now();
	// Broadcast pose
	cv::Mat rvec, tvec;
	bool evo_valid;
	evo_valid = evo.getPose(rvec, tvec);
	if(evo_valid) { // Only broadcast pose and odom if the estimate is valid
		geometry_msgs::TransformStamped tf_msg;
		tf_msg.header.stamp = caminfo->header.stamp;
		tf_msg.header.frame_id = "odom";
		tf_msg.child_frame_id = "evo";
		vec_to_xyz(tvec, tf_msg.transform.translation);
		rvec_to_quat(rvec, tf_msg.transform.rotation);
		tf_bcaster->sendTransform(tf_msg); // tf_bcaster should always exist
		// Broadcast odom
		nav_msgs::Odometry odom_msg;
		odom_msg.header.stamp = caminfo->header.stamp;
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "evo";
		vec_to_xyz(tvec, odom_msg.pose.pose.position);
		rvec_to_quat(rvec, odom_msg.pose.pose.orientation);
		cv::Mat vel, rates;
		evo.getRates(vel, rates);
		vec_to_xyz(vel, odom_msg.twist.twist.linear);
		vec_to_xyz(rates, odom_msg.twist.twist.angular);
		odom_pub.publish(odom_msg);
	}
	auto time_msg_sent = std::chrono::high_resolution_clock::now();

	// Log profiling
	time_log << std::chrono::duration_cast<std::chrono::microseconds>(time_on_image - time_start).count()
	    << std::chrono::duration_cast<std::chrono::microseconds>(time_before_evo - time_start).count()
	    << std::chrono::duration_cast<std::chrono::microseconds>(time_after_evo - time_start).count()
	    << std::chrono::duration_cast<std::chrono::microseconds>(time_msg_sent - time_start).count()
	    << evo_valid << std::endl;

	// !!!! Could this cause delays???
	char key = cv::waitKey(1);
	if(key == 27 || key == 'q') {
		ros::shutdown();
	}
}

void configure_evo(ros::NodeHandle &nh) {
	int target_keypts;
	if(nh.getParam("target_keypts", target_keypts)) {
		evo.setTargetKeypts(target_keypts);
		ROS_INFO("Target keypoints: %d", target_keypts);
	}

	int min_keypts;
	if(nh.getParam("min_keypts", min_keypts)) {
		evo.setMinKeypts(min_keypts);
		ROS_INFO("Min keypoints: %d", min_keypts);
	}

	double keyframe_thres;
	if(nh.getParam("keyframe_thres", keyframe_thres)) {
		evo.setKeyframeThres(keyframe_thres);
		ROS_INFO("Keyframe threshold: %f", keyframe_thres);
	}

	double near_clip;
	if(nh.getParam("near_clip", near_clip)) {
		evo.setNearClip(near_clip);
		ROS_INFO("Near clip: %f", near_clip);
	}

	double far_clip;
	if(nh.getParam("far_clip", far_clip)) {
    evo.setFarClip(far_clip);
    ROS_INFO("Far clip: %f", far_clip);
  }

	int grid_rows;
	if(nh.getParam("grid_rows", grid_rows)) {
		evo.setGridRows(grid_rows);
		ROS_INFO("Grid rows: %d", grid_rows);
	}

	int grid_cols;
	if(nh.getParam("grid_cols", grid_cols)) {
		evo.setGridCols(grid_cols);
		ROS_INFO("Grid cols: %d", grid_cols);
	}
}

} // namespace

int main(int argc, char **argv) {
	// Initialize ROS
	ros::init(argc, argv, "openevo");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	// Configure evo
	configure_evo(nh_private);

	// TODO Load imu to cam transform
	double R_cam_imu_data[] = {	 0.0, 1.0,  0.0,
								 0.0,  0.0,  -1.0,
								-1.0,  0.0,  0.0 }; // XXX Fix, incorrect!
	R_cam_imu = cv::Mat(3, 3, CV_64F, R_cam_imu_data);

	// Advertise tf and odom
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	tf_bcaster = new tf::TransformBroadcaster();

	// Subscribe to IMU
	ros::Subscriber imu_sub = nh.subscribe("/imu", 100, &on_imu);

	// Subscribe to image topics
	image_transport::ImageTransport it(nh);
	image_transport::SubscriberFilter sub_left(it, "image", 3);
	image_transport::SubscriberFilter sub_depth(it, "depth_image", 3);
	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_caminfo(nh, "camera_info", 3);
	message_filters::TimeSynchronizer
			<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
			sync(sub_left, sub_depth, sub_caminfo, 10);
	sync.registerCallback(&on_image);

	// Open log file for profiling
	time_log.open("~/openevo_time.csv");
	time_log << "on_image,before_evo,after_evo,msg_sent,evo_valid," << std::endl;

	ros::spin();

	time_log.close();

	return 0;
}
