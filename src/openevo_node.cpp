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

namespace{

openevo::EVO evo;
image_geometry::PinholeCameraModel model;

tf::TransformBroadcaster *tf_bcaster;
ros::Publisher odom_pub;

cv::Mat R_cam_imu;

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
	rates = R_cam_imu * rates;
	evo.updateIMU(rates, ros::Time::now().toSec());
}

void on_image(
		const sensor_msgs::ImageConstPtr &color,
		const sensor_msgs::ImageConstPtr &depth,
		const sensor_msgs::CameraInfoPtr &caminfo) {
	// Update camera information
	model.fromCameraInfo(caminfo);
	// Convert images to openCV
	cv_bridge::CvImageConstPtr colorbr = cv_bridge::toCvShare(color, "mono8");
	cv_bridge::CvImageConstPtr depthbr = cv_bridge::toCvShare(depth);
	// Update EVO
	cv::Mat intr;
	static_cast<cv::Mat>(model.projectionMatrix()).copyTo(intr); // Note: intrinsic matrix is 0 on SLAMDunk...
	intr = intr(cv::Range(0, 3), cv::Range(0, 3));
	evo.updateImageDepth(colorbr->image, depthbr->image, intr,
			caminfo->header.stamp.toSec());
	// Broadcast pose
	cv::Mat rvec, tvec;
	evo.getPose(rvec, tvec);
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


	char key = cv::waitKey(1);
	if(key == 27 || key == 'q') {
		ros::shutdown();
	}
}

} // namespace

int main(int argc, char **argv) {
	// Initialize ROS
	ros::init(argc, argv, "percevite");
	ros::NodeHandle nh;

	// TODO Load imu to cam transform
	double R_cam_imu_data[] = {	 0.0, -1.0,  0.0,
								 0.0,  0.0,  1.0,
								-1.0,  0.0,  0.0 };
	R_cam_imu = cv::Mat(3, 3, CV_64F, R_cam_imu_data);

	// Advertise tf and odom
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	tf_bcaster = new tf::TransformBroadcaster();

	// Subscribe to IMU
	ros::Subscriber imu_sub = nh.subscribe("/imu", 100, &on_imu);

	// Subscribe to image topics
	image_transport::ImageTransport it(nh);
	image_transport::SubscriberFilter sub_left(it, "/left_rgb_rect/image_rect_color", 3);
	image_transport::SubscriberFilter sub_depth(it, "/depth_map/image", 3);
	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_caminfo(nh, "/left_rgb_rect/camera_info", 3);
	message_filters::TimeSynchronizer
			<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
			sync(sub_left, sub_depth, sub_caminfo, 10);
	sync.registerCallback(&on_image);

	ros::spin();

	return 0;
}
