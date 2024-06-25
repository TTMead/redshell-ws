#include "aggregator.h"

#include <tf2/exceptions.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

Aggregator::Aggregator() : Node("aggregator_node")
{
    this->declare_parameter("field_topics", std::vector<std::string>({"/front_field"}));
    this->declare_parameter("aggregate_frame_id", "map");

	initialise_occupancy_grid_msg();

    _tf_buffer.reset(new tf2_ros::Buffer(this->get_clock()));
    _tf_listener.reset(new tf2_ros::TransformListener(*_tf_buffer));

    // Create subscription to all field topics
    std::vector field_topics = this->get_parameter("field_topics").as_string_array();
        using namespace std::chrono_literals;
    for (std::string field_topic : field_topics)
    {
        field_sub new_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            field_topic, 10, 
            std::bind(&Aggregator::potential_field_callback, this, std::placeholders::_1)
        );

        _potential_field_subs.push_back(new_sub);
    }

	_aggregate_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("potential_field_combined", 10);

	_publish_timer = this->create_wall_timer(
		250ms, std::bind(&Aggregator::publish_costmap, this)
	);

	_filter_timer = this->create_wall_timer(
		500ms, std::bind(&Aggregator::filter_costmap, this)
	);

    using namespace std::placeholders;
    _reset_aggregate_grid_service = this->create_service<occupancy_grid_aggregator_srv::srv::ResetAggregateGrid>("reset_aggregate_grid", std::bind(&Aggregator::reset_grid_service_callback, this, _1, _2));
}

void
Aggregator::publish_costmap()
{
	_aggregated_occupancy_grid.header.stamp = rclcpp::Node::now();
    _aggregate_grid_pub->publish(_aggregated_occupancy_grid);
}

void
Aggregator::filter_costmap()
{
	fade(_aggregated_occupancy_grid, -1);
}

void
Aggregator::potential_field_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
	combine_costmaps(_aggregated_occupancy_grid, *msg);
}

void
Aggregator::reset_grid_service_callback(const std::shared_ptr<occupancy_grid_aggregator_srv::srv::ResetAggregateGrid::Request> request, 
            std::shared_ptr<occupancy_grid_aggregator_srv::srv::ResetAggregateGrid::Response> response)
{
    // Using std::ignore as request/response are defined as empty
    std::ignore = request;
    std::ignore = response;

    RCLCPP_INFO(this->get_logger(), "Resetting occupancy grid");
    
    _aggregated_occupancy_grid.data.clear();
    for (uint32_t row_index = 0; row_index < _aggregated_occupancy_grid.info.height; row_index++)
    {
        for (uint32_t column_index = 0; column_index < _aggregated_occupancy_grid.info.width; column_index++)
        {
            _aggregated_occupancy_grid.data.push_back(0);
        }
    }
}

void
Aggregator::combine_costmaps(nav_msgs::msg::OccupancyGrid& grid, const nav_msgs::msg::OccupancyGrid& new_grid)
{
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = _tf_buffer->lookupTransform(grid.header.frame_id, new_grid.header.frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
    }

    // Iterate through every tile in new_grid
    for (int64_t tile_index = 0; tile_index < (new_grid.info.width * new_grid.info.height); tile_index++)
    {
        // If this tile has a small value, skip it
        static constexpr int8_t copy_threshold = 5;
        if (std::abs(new_grid.data[tile_index]) < copy_threshold)
        {
            continue;
        }

        // Get position of tile
        const geometry_msgs::msg::PointStamped tile_location = get_point_from_index(new_grid, tile_index);

        // Transform tile position to the grid frame
        geometry_msgs::msg::PointStamped  grid_tile_location; 
        grid_tile_location.header.frame_id = grid.header.frame_id;
        tf2::doTransform(tile_location, grid_tile_location, transform);
        
        // Add transformed tile to grid
        add_point_to_grid(grid, grid_tile_location.point, new_grid.data[tile_index]);
    }
}

void
Aggregator::initialise_occupancy_grid_msg()
{
    static constexpr float costmap_resolution_m_per_cell = 0.05;
    static constexpr uint32_t costmap_width_cells = 1000;
    static constexpr uint32_t costmap_height_cells = 1000;

    _aggregated_occupancy_grid.header.frame_id = "map";
    _aggregated_occupancy_grid.info.resolution = costmap_resolution_m_per_cell;
    _aggregated_occupancy_grid.info.width = costmap_width_cells;
    _aggregated_occupancy_grid.info.height = costmap_height_cells;
    _aggregated_occupancy_grid.info.origin.position.x = -(static_cast<double>(costmap_width_cells)/2.0) * costmap_resolution_m_per_cell;
    _aggregated_occupancy_grid.info.origin.position.y = -(static_cast<double>(costmap_height_cells)/2.0) * costmap_resolution_m_per_cell;
    _aggregated_occupancy_grid.info.origin.position.z = 0;
    _aggregated_occupancy_grid.info.origin.orientation.x = 0;
    _aggregated_occupancy_grid.info.origin.orientation.y = 0;
    _aggregated_occupancy_grid.info.origin.orientation.z = 0;
    _aggregated_occupancy_grid.info.origin.orientation.w = 1;

    for (uint32_t row_index = 0; row_index < costmap_height_cells; row_index++)
    {
        for (uint32_t column_index = 0; column_index < costmap_width_cells; column_index++)
        {
            _aggregated_occupancy_grid.data.push_back(0);
        }
    }
}
