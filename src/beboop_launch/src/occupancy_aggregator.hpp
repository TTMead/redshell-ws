#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

class OccupancyAggregator : public rclcpp::Node
{
	public:
		OccupancyAggregator();

	private:
		nav_msgs::msg::OccupancyGrid _aggregated_occupancy_grid;

		rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _potential_field_sub;
		rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _aggregate_grid_pub;
};