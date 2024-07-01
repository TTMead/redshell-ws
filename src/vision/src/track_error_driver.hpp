#include "vision_driver.hpp"
#include "std_msgs/msg/float32.hpp"

class TrackErrorDriver : public VisionDriver
{
	public:
		TrackErrorDriver();

	private:
		void analyse_frame(cv::Mat image_frame) override;

		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _error_publisher;
};