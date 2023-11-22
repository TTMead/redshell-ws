#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <string>

class VisionDriver : public rclcpp::Node
{
    public:
        VisionDriver(const std::string &node_name);

    protected:

		/**
		 * Performs analysis on the an image_frame that was captured
		*/
		virtual void analyse_frame(cv::Mat image_frame) = 0;

        cv::Point get_centroid(cv::Mat &mask);

		cv::VideoCapture _video_capture;

    private:
		/**
		 * Runs the core vision aquisition and processing loop. Is not
		 * used when running in SITL
		*/
		int run();

		/**
		 * Is called from /front_camera messages received while in SITL.
		*/
		void front_camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _front_camera_sub;

};