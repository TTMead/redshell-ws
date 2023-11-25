#include <math.h>
#include <algorithm>

#include "potential_field_driver.hpp"

typedef nav_msgs::msg::OccupancyGrid Grid;

PotentialFieldDriver::PotentialFieldDriver() : VisionDriver("track_error_driver")
{
    initialise_occupancy_grid_msg();

    _potential_field_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("potential_field", 10);

    this->declare_parameter("y_pixel_to_distance_a", 269.0);
    this->declare_parameter("y_pixel_to_distance_b", -300.64);
    this->declare_parameter("x_pixel_to_bearing_a", 0.0850541);
    this->declare_parameter("x_pixel_to_bearing_b", -52.4552);
}

void
PotentialFieldDriver::initialise_occupancy_grid_msg()
{
    _occupancy_grid.info.resolution = COSTMAP_RESOLUTION;
    _occupancy_grid.info.width = COSTMAP_WIDTH;
    _occupancy_grid.info.height = COSTMAP_HEIGHT;
    _occupancy_grid.info.origin.position.x = 0;
    _occupancy_grid.info.origin.position.y = 0;
    _occupancy_grid.info.origin.position.z = 0;
    _occupancy_grid.info.origin.orientation.x = 0;
    _occupancy_grid.info.origin.orientation.y = 0;
    _occupancy_grid.info.origin.orientation.z = 0;
    _occupancy_grid.info.origin.orientation.w = 1;

    for (uint32_t row_index = 0; row_index < COSTMAP_HEIGHT; row_index++)
    {
        for (uint32_t column_index = 0; column_index < COSTMAP_WIDTH; column_index++)
        {
            set_occupancy_grid_tile(row_index, column_index, 0);
        }
    }
}


void
PotentialFieldDriver::analyse_frame(cv::Mat image_frame)
{
	// Convert the frame to hsv
	cv::cvtColor(image_frame, image_frame, cv::COLOR_BGR2HSV);

	// Threshold the frame by colour
	cv::Mat yellow_frame, blue_frame;
	cv::inRange(image_frame, 
                cv::Scalar(YELLOW_THRESHOLD_H_LOW, YELLOW_THRESHOLD_S_LOW, YELLOW_THRESHOLD_V_LOW),
                cv::Scalar(YELLOW_THRESHOLD_H_HIGH, YELLOW_THRESHOLD_S_HIGH, YELLOW_THRESHOLD_V_HIGH), 
                yellow_frame);
	cv::inRange(image_frame, 
                cv::Scalar(BLUE_THRESHOLD_H_LOW, BLUE_THRESHOLD_S_LOW, BLUE_THRESHOLD_V_LOW),
                cv::Scalar(BLUE_THRESHOLD_H_HIGH, BLUE_THRESHOLD_S_HIGH, BLUE_THRESHOLD_V_HIGH), 
                blue_frame);

    // Combine the image frames and apply post processing
    cv::Mat track_frame = yellow_frame + blue_frame;
	cv::medianBlur(track_frame, track_frame, 7);

    if (this->get_parameter("is_sitl").as_bool())
    {
        cv::imshow("Combined Threshold", track_frame);
        cv::waitKey(1);
    }

    add_bin_image_to_occupancy(track_frame);

    publish();
}


void
PotentialFieldDriver::set_occupancy_grid_tile(uint32_t row_index, uint32_t column_index, int8_t value)
{
    if (row_index >= COSTMAP_HEIGHT)
    {
        RCLCPP_ERROR(rclcpp::get_logger("occupancy_grid_helper"), "Attempt to access out of bounds row. Requested row: %d. Max height: %d.", row_index, COSTMAP_HEIGHT);
        return;
    }

    if (column_index >= COSTMAP_WIDTH)
    {
        RCLCPP_ERROR(rclcpp::get_logger("occupancy_grid_helper"), "Attempt to access out of bounds column. Requested column: %d. Max width: %d.", column_index, COSTMAP_WIDTH);
        return;
    }

    _occupancy_grid.data[column_index + (row_index*COSTMAP_WIDTH)] = value;
}


void
PotentialFieldDriver::add_bin_image_to_occupancy(cv::Mat binary_image)
{
    std::vector<cv::Point> locations;
    cv::findNonZero(binary_image, locations);

    for (cv::Point point_px : locations)
    {
        double x_m, y_m;
        pixels_to_m(point_px.x, point_px.y, x_m, y_m);

        uint32_t row_index = std::round(x_m / COSTMAP_RESOLUTION);
        uint32_t column_index = std::round(y_m / COSTMAP_RESOLUTION) + (COSTMAP_WIDTH/2);

        // Ignore readings that go outside our defined costmap range
        if (row_index >= COSTMAP_HEIGHT || column_index >= COSTMAP_WIDTH)
        {
            continue;
        }

        static constexpr int8_t track_value = 80;
        set_occupancy_grid_tile(row_index, column_index, track_value);
    }

    // Uncomment for data collection
    // if (locations.size() != 0)
    // {
    //     cv::Point mid = locations[locations.size()/2];

    //     RCLCPP_INFO(this->get_logger(), "mid x: [%d]", mid.x);
    //     RCLCPP_INFO(this->get_logger(), "mid y: [%d]", mid.y);
    // }
}

void
PotentialFieldDriver::pixels_to_m(double x_px, double y_px, double &x_m, double &y_m)
{
    double y_pixel_to_distance_a = this->get_parameter("y_pixel_to_distance_a").as_double();
    double y_pixel_to_distance_b = this->get_parameter("y_pixel_to_distance_b").as_double();
    double x_pixel_to_bearing_a = this->get_parameter("x_pixel_to_bearing_a").as_double();
    double x_pixel_to_bearing_b = this->get_parameter("x_pixel_to_bearing_b").as_double();

    // Convert pixel coordinates into real world distance and bearing
    // using empirical model.
    double forward_distance_m = y_pixel_to_distance_a / (static_cast<double>(y_px) + y_pixel_to_distance_b);
    double bearing_deg = (x_pixel_to_bearing_a * static_cast<double>(x_px)) + x_pixel_to_bearing_b;

    // Using FLU frame
    x_m = forward_distance_m;
    y_m = forward_distance_m * std::tan(bearing_deg * M_PI / 180.0);
}


uint32_t
PotentialFieldDriver::scale(uint32_t value, uint32_t old_min, uint32_t old_max, uint32_t new_min, uint32_t new_max)
{
    return round(((static_cast<double>(value - old_min) / static_cast<double>(old_max - old_min)) * (new_max - new_min)) + new_min);
}

void
PotentialFieldDriver::publish()
{
    _occupancy_grid.header.stamp = rclcpp::Node::now();
    _occupancy_grid.header.frame_id = "map";

    _potential_field_publisher->publish(_occupancy_grid);
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PotentialFieldDriver>());
	return 0;
}