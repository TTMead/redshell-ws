#include "aggregator.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("aggregator_node");
    Aggregator aggregator(node);

    node->declare_parameter("field_topics", std::vector<std::string>({"/front_field"}));
    node->declare_parameter("aggregate_frame_id", "map");

    // Create a subscription to all field topics
    std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> potential_field_subs;
    const std::vector field_topics = node->get_parameter("field_topics").as_string_array();
    for (const std::string& field_topic : field_topics)
    {
        auto new_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            field_topic, 10, 
            std::bind(&Aggregator::potential_field_callback, &aggregator, std::placeholders::_1)
        );

        potential_field_subs.push_back(new_sub);
    }

    // Regularly publish the aggregate grid
    auto aggregate_grid_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("potential_field_combined", 10);
	auto grid_pub_timer = node->create_wall_timer(std::chrono::milliseconds(250), [&](){
        aggregate_grid_pub->publish(aggregator.get_occupancy_grid());
    });

    // Regularly filter the aggregate grid
	auto filter_timer = node->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Aggregator::filter_costmap, &aggregator));

    // Initialise the grid resetting service
    auto reset_aggregate_grid_service = node->create_service<occupancy_grid_aggregator_srv::srv::ResetAggregateGrid>("reset_aggregate_grid",
        std::bind(&Aggregator::reset_grid_service_callback, &aggregator, std::placeholders::_1, std::placeholders::_2));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}