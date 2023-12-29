#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class OccupancyAggregator : public rclcpp::Node
{
	typedef rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr field_sub;

	public:
		OccupancyAggregator();

	private:
		void potential_field_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
		void update();
		void initialise_occupancy_grid_msg();
		void publish();

		nav_msgs::msg::OccupancyGrid _aggregated_occupancy_grid;
		rclcpp::TimerBase::SharedPtr _timer;

		std::vector<field_sub> _potential_field_subs;
		rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _aggregate_grid_pub;
};