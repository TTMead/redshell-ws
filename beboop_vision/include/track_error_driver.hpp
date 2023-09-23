#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cv_bridge/cv_bridge.h>

class TrackErrorDriver : public rclcpp::Node
{
	public:
		TrackErrorDriver();

		const int VIDEO_CAMERA_ID = 0;
	
	private:
		/**
		 * Runs the core vision aquisition and processing loop. Is not
		 * used when running in SITL
		*/
		int run();

		/**
		 * Performs analysis on the current frame stored in _image_frame
		*/
		void analyse_frame();

		/**
		 * Is called from /front_camera messages received while in SITL.
		*/
		void front_camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);

		cv::Point get_centroid(cv::Mat &mask);

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _front_camera_sub;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _error_publisher;

		cv::VideoCapture _video_capture;
		cv::Mat _image_frame;
};