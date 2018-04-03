#include "openevo/evo.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>

#include <iostream>

namespace{

openevo::EVO evo;
image_geometry::PinholeCameraModel model;


void on_image(
		const sensor_msgs::ImageConstPtr &color,
		const sensor_msgs::ImageConstPtr &depth,
		const sensor_msgs::CameraInfoPtr &caminfo) {
	// Update camera information
	model.fromCameraInfo(caminfo);
	// Convert images to openCV
	cv_bridge::CvImageConstPtr colorbr = cv_bridge::toCvShare(color, "bgr8");
	cv_bridge::CvImageConstPtr depthbr = cv_bridge::toCvShare(depth);
	// Test code
	cv::Mat intr;
	static_cast<cv::Mat>(model.projectionMatrix()).copyTo(intr); // Note: intrinsic matrix is 0 on SLAMDunk...
	intr = intr(cv::Range(0, 3), cv::Range(0, 3));
	evo.updateImageDepth(colorbr->image, depthbr->image, intr);

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
