#include "vision_driver.hpp"


VisionDriver::VisionDriver(const std::string &node_name) : Node(node_name)
{
	this->declare_parameter("camera_topic", "/front_camera");
	this->declare_parameter("camera_index", 0);
    this->declare_parameter("is_sitl", false);

	if (this->get_parameter("is_sitl").as_bool()) {
		// In SITL receive image feed from the ROS topic
		_front_camera_sub = this->create_subscription<sensor_msgs::msg::Image>(
			this->get_parameter("camera_topic").as_string(), 10, 
			std::bind(&VisionDriver::front_camera_callback, this, std::placeholders::_1)
		);
	} else {
		// If not SITL receive image feed from webcam
		_video_capture = cv::VideoCapture(this->get_parameter("camera_index").as_int(), cv::CAP_V4L2);
		if (!_video_capture.isOpened()) {
			RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
		}

		RCLCPP_INFO(this->get_logger(), "Backend: %s", std::string(_video_capture.getBackendName()).c_str());
		_run_thread = std::thread(&VisionDriver::run, this);
	}
}


void
VisionDriver::front_camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGBA8);

	cv::Mat image_frame = cv_image_ptr->image;
	cv::cvtColor(image_frame, image_frame, cv::COLOR_RGB2BGR);

	analyse_frame(image_frame);
}


cv::Point
VisionDriver::get_centroid(cv::Mat &mask)
{
	cv::Moments m = cv::moments(mask, true);
	cv::Point centroid(m.m10/m.m00, m.m01/m.m00);

	// Checks to fix funkiness from empty masks
	if (centroid.x < 0) centroid.x = 0;
	if (centroid.y < 0) centroid.y = 0;

	return centroid;
}


void
VisionDriver::run()
{
	bool running = true;
	while (running)
	{
		cv::Mat image_frame;
		_video_capture >> image_frame;

		if (image_frame.empty()) {
			RCLCPP_ERROR(this->get_logger(), "Could not extract image frame from video capture. Exiting");
			rclcpp::shutdown();
			running = false;
		}

		analyse_frame(image_frame);
	}
}

