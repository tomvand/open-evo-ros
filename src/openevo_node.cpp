#include "openevo/evo.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

namespace{

openevo::EVO evo;

void on_image(
		const sensor_msgs::ImageConstPtr &color,
		const sensor_msgs::ImageConstPtr &depth) {
	// Convert images to openCV
	cv_bridge::CvImageConstPtr colorbr = cv_bridge::toCvShare(color, "bgr8");
	cv_bridge::CvImageConstPtr depthbr = cv_bridge::toCvShare(depth);
	// Test code
	evo.updateImageDepth(colorbr->image, depthbr->image);
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
	image_transport::SubscriberFilter subf_left(it, "/left_rgb_rect/image_rect_color", 3);
	image_transport::SubscriberFilter subf_depth(it, "/depth_map/image", 3);
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(
				subf_left, subf_depth, 10);
	sync.registerCallback(boost::bind(&on_image, _1, _2));

	ros::spin();

	return 0;
}
