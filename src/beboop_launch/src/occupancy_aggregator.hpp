#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class OccupancyAggregator : public rclcpp::Node
{
	public:
		OccupancyAggregator();

	private:
		void potential_field_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
		void update();
		void initialise_occupancy_grid_msg();
		void publish();

		nav_msgs::msg::OccupancyGrid _aggregated_occupancy_grid;
		rclcpp::TimerBase::SharedPtr _timer;

		rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _potential_field_sub;
		rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _aggregate_grid_pub;
};