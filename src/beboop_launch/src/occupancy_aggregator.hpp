#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"

class OccupancyAggregator : public rclcpp::Node
{
	typedef rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr field_sub;

	public:
		OccupancyAggregator();

	private:
		void potential_field_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
		void wheel_encoder_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
		void update();
		void initialise_occupancy_grid_msg();
		void publish();

		nav_msgs::msg::OccupancyGrid _aggregated_occupancy_grid;
		rclcpp::TimerBase::SharedPtr _timer;

		std::vector<field_sub> _potential_field_subs;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _encoder_sub;
		rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _aggregate_grid_pub;
};