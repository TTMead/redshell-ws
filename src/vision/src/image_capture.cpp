#include "image_capture.hpp"


ImageCapture::ImageCapture() : Node("image_capture")
{
    this->declare_parameter("is_sitl", false);
	if (this->get_parameter("is_sitl").as_bool()) {
		// In SITL receive image feed from the ROS topic
		_front_camera_sub = this->create_subscription<sensor_msgs::msg::Image>(
			"/front_camera", 10,
			std::bind(&ImageCapture::front_camera_capture, this, std::placeholders::_1)
		);
	} else {
		// If not SITL receive image feed from webcam
		_video_capture = cv::VideoCapture(VIDEO_CAMERA_ID, cv::CAP_V4L2);
		if (!_video_capture.isOpened()) {
			RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
		}

		RCLCPP_INFO(this->get_logger(), "Backend: %s", std::string(_video_capture.getBackendName()).c_str());

        physical_camera_capture();
	}
}


void
ImageCapture::front_camera_capture(const sensor_msgs::msg::Image::SharedPtr msg)
{
	cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGBA8);

	cv::Mat image_frame = cv_image_ptr->image;
	cv::cvtColor(image_frame, image_frame, cv::COLOR_RGB2BGR);

	save_camera_image(image_frame);
}


void
ImageCapture::physical_camera_capture()
{
    cv::Mat image_frame;
	_video_capture >> image_frame;

	if (image_frame.empty()) {
		RCLCPP_ERROR(this->get_logger(), "Could not extract image frame from video capture");
		// rclcpp::shutdown();
	}

    save_camera_image(image_frame);
}


void
ImageCapture::save_camera_image(cv::Mat image)
{
    cv::imwrite("image_capture.jpg", image);
    // rclcpp::shutdown();
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImageCapture>());
	return 0;
}