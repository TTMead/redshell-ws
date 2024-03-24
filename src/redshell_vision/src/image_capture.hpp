#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <string>

class ImageCapture : public rclcpp::Node
{
    public:
        ImageCapture();

    private:
		/**
		 * Is called from /front_camera messages received while in SITL.
		*/
		void front_camera_capture(const sensor_msgs::msg::Image::SharedPtr msg);
		void physical_camera_capture();
		void save_camera_image(cv::Mat image);

        const int VIDEO_CAMERA_ID = 0;
		cv::VideoCapture _video_capture;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _front_camera_sub;
};