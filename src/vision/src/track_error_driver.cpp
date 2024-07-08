#include "track_error_driver.hpp"


TrackErrorDriver::TrackErrorDriver() : VisionDriver("track_error_driver")
{
	_error_publisher = this->create_publisher<std_msgs::msg::Float32>("track_error", 10);
}

void
TrackErrorDriver::analyse_frame(cv::Mat image_frame)
{
	// cv::imshow("Image", image_frame);

	// Convert the frame to hsv
	cv::cvtColor(image_frame, image_frame, cv::COLOR_BGR2HSV);

	// Threshold the frame by colour
	cv::Mat yellow_frame, blue_frame;
	cv::inRange(image_frame, cv::Scalar(25, 30, 0), cv::Scalar(35, 255, 255), yellow_frame);
	cv::inRange(image_frame, cv::Scalar(90, 50, 190), cv::Scalar(130, 250, 250), blue_frame);

	// cv::imshow("Yellow Threshold", yellow_frame);
	// cv::imshow("Blue Threshold", blue_frame);
	// cv::waitKey(1);

	// Apply median blur to each frame
	cv::medianBlur(yellow_frame, yellow_frame, 7);
	cv::medianBlur(blue_frame, blue_frame, 7);

	// Calculate standard errors
	float standard_left_error = 0.4;
	float standard_right_error = -0.4;

	// Masked image must be big enough to be valid
	static constexpr float cutoff_percentage = 0.0000001;
	bool blue_ok = (cv::mean(blue_frame)[0] / blue_frame.total()) > cutoff_percentage;
	bool yellow_ok = (cv::mean(yellow_frame)[0] / yellow_frame.total()) > cutoff_percentage;

	std_msgs::msg::Float32 error_msg;
	if (blue_ok && yellow_ok) {
		// If blue and yellow are good masks, then use the mean centroid as error
		float track_middle_px = static_cast<float>(get_centroid(blue_frame).x + get_centroid(yellow_frame).x)/2.0;	// Middle of the track in px
		float error_px = track_middle_px - (image_frame.cols/2.0);	// The error of the track in px
		error_msg.data = error_px/static_cast<float>(image_frame.cols);		// Normalise the px error to [-1, 1]
	}
	else
	{
		// If only one of the masks are okay, use a preset error
		if (blue_ok)
		{
			error_msg.data = standard_left_error;
		}
		else if (yellow_ok)
		{
			error_msg.data = standard_right_error;
		}
		else
		{
			// If neither of the masks are okay
			error_msg.data = std::numeric_limits<float>::quiet_NaN();
		}

	}
	_error_publisher->publish(error_msg);
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TrackErrorDriver>());
	return 0;
}