#include "occupancy_aggregator.hpp"
#include "occupancy_grid_helpers.h"

using namespace std::chrono_literals;

OccupancyAggregator::OccupancyAggregator() : Node("occupancy_aggregator_node")
{
    this->declare_parameter("field_topics", std::vector<std::string>({"/front_field"}));

    std::vector field_topics = this->get_parameter("field_topics").as_string_array();
    for (std::string field_topic : field_topics)
    {
        field_sub new_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            field_topic, 10, 
            std::bind(&OccupancyAggregator::potential_field_callback, this, std::placeholders::_1)
        );

        _potential_field_subs.push_back(new_sub);
    }

	initialise_occupancy_grid_msg();
	_aggregate_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("potential_field_combined", 10);


    _encoder_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/wheel_encoders", 10, 
        std::bind(&OccupancyAggregator::wheel_encoder_callback, this, std::placeholders::_1)
    );

	_timer = this->create_wall_timer(
		250ms, std::bind(&OccupancyAggregator::update, this)
	);
}

void
OccupancyAggregator::potential_field_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
	combine_costmaps(_aggregated_occupancy_grid, *msg);
}

void
OccupancyAggregator::wheel_encoder_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    static rclcpp::Time previous_time = rclcpp::Node::now();
    static double forward_distance_remainder = 0.0;

    rclcpp::Time current_time = rclcpp::Node::now();
    rclcpp::Duration dt = current_time - previous_time;
    previous_time = rclcpp::Node::now();

    double foward_distance_m = msg->linear.x * dt.seconds();
    double forward_distance_cells = (foward_distance_m / COSTMAP_RESOLUTION) + forward_distance_remainder;
    int32_t x_translation = static_cast<int32_t>(std::round(forward_distance_cells));
    forward_distance_remainder = forward_distance_cells - x_translation;
    translate(_aggregated_occupancy_grid, x_translation, 0);
}


void
OccupancyAggregator::update()
{
	publish();
	// fade(_aggregated_occupancy_grid, 40);
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