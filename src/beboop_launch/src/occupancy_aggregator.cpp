#include "occupancy_aggregator.hpp"
#include "occupancy_grid_helpers.h"

using namespace std::chrono_literals;

OccupancyAggregator::OccupancyAggregator() : Node("occupancy_aggregator_node")
{
    _potential_field_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
		"/potential_field", 10, 
		std::bind(&OccupancyAggregator::potential_field_callback, this, std::placeholders::_1)
	);

	_aggregate_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("potential_field_combined", 10);

	_timer = this->create_wall_timer(
		100ms, std::bind(&OccupancyAggregator::update, this)
	);

	initialise_occupancy_grid_msg();
}

void
OccupancyAggregator::potential_field_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
	add_costmaps(_aggregated_occupancy_grid, *msg);
}


void
OccupancyAggregator::update()
{
	// fade(_aggregated_occupancy_grid, 1);
	publish();
}

void
OccupancyAggregator::initialise_occupancy_grid_msg()
{
    _aggregated_occupancy_grid.info.resolution = COSTMAP_RESOLUTION;
    _aggregated_occupancy_grid.info.width = COSTMAP_WIDTH;
    _aggregated_occupancy_grid.info.height = COSTMAP_HEIGHT;
    _aggregated_occupancy_grid.info.origin.position.x = 0;
    _aggregated_occupancy_grid.info.origin.position.y = 0;
    _aggregated_occupancy_grid.info.origin.position.z = 0;
    _aggregated_occupancy_grid.info.origin.orientation.x = 0;
    _aggregated_occupancy_grid.info.origin.orientation.y = 0;
    _aggregated_occupancy_grid.info.origin.orientation.z = 0;
    _aggregated_occupancy_grid.info.origin.orientation.w = 1;

    for (uint32_t row_index = 0; row_index < COSTMAP_HEIGHT; row_index++)
    {
        for (uint32_t column_index = 0; column_index < COSTMAP_WIDTH; column_index++)
        {
            _aggregated_occupancy_grid.data.push_back(0);
        }
    }
}


void
OccupancyAggregator::publish()
{
	_aggregated_occupancy_grid.header.stamp = rclcpp::Node::now();
    _aggregated_occupancy_grid.header.frame_id = "map";

    _aggregate_grid_pub->publish(_aggregated_occupancy_grid);
}



int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OccupancyAggregator>());
	rclcpp::shutdown();
	return 0;
}