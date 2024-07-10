#include <math.h>
#include <algorithm>

#include "potential_field_driver.hpp"

typedef nav_msgs::msg::OccupancyGrid Grid;

PotentialFieldDriver::PotentialFieldDriver() : VisionDriver("track_error_driver")
{
    initialise_occupancy_grid_msg();

    this->declare_parameter("y_pixel_to_distance_a", 138.0);
    this->declare_parameter("y_pixel_to_distance_b", -137.0);
    this->declare_parameter("x_pixel_to_bearing_a", 0.11);
    this->declare_parameter("x_pixel_to_bearing_b", -32.0);
    this->declare_parameter("frame_id", "base_link");
    this->declare_parameter("field_topic", "/front_field");
    this->declare_parameter("camera_pub_topic", "/front_cam");

    _potential_field_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(this->get_parameter("field_topic").as_string(), 10);
    _physical_camera_publisher = this->create_publisher<sensor_msgs::msg::Image>(this->get_parameter("camera_pub_topic").as_string(), 10);
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
            _occupancy_grid.data.push_back(0);
        }
    }
}

void
PotentialFieldDriver::clear_occupancy_grid_msg()
{
    std::fill(_occupancy_grid.data.begin(), _occupancy_grid.data.end(), 0);
}


void
PotentialFieldDriver::analyse_frame(cv::Mat image_frame)
{
    std_msgs::msg::Header header;
    header.frame_id = "front_cam";
    header.stamp = rclcpp::Node::now();

	// Convert the frame to hsv
	cv::cvtColor(image_frame, image_frame, cv::COLOR_BGR2HSV);

	// Threshold the frame by colour
	cv::Mat yellow_frame, blue_frame, purple_frame, red_frame;
	cv::inRange(image_frame, 
                cv::Scalar(YELLOW_THRESHOLD_H_LOW, YELLOW_THRESHOLD_S_LOW, YELLOW_THRESHOLD_V_LOW),
                cv::Scalar(YELLOW_THRESHOLD_H_HIGH, YELLOW_THRESHOLD_S_HIGH, YELLOW_THRESHOLD_V_HIGH), 
                yellow_frame);
	cv::inRange(image_frame, 
                cv::Scalar(BLUE_THRESHOLD_H_LOW, BLUE_THRESHOLD_S_LOW, BLUE_THRESHOLD_V_LOW),
                cv::Scalar(BLUE_THRESHOLD_H_HIGH, BLUE_THRESHOLD_S_HIGH, BLUE_THRESHOLD_V_HIGH), 
                blue_frame);
    cv::inRange(image_frame, 
                cv::Scalar(PURPLE_THRESHOLD_H_LOW, PURPLE_THRESHOLD_S_LOW, PURPLE_THRESHOLD_V_LOW),
                cv::Scalar(PURPLE_THRESHOLD_H_HIGH, PURPLE_THRESHOLD_S_HIGH, PURPLE_THRESHOLD_V_HIGH), 
                purple_frame);
    cv::inRange(image_frame, 
                cv::Scalar(RED_THRESHOLD_H_LOW, RED_THRESHOLD_S_LOW, RED_THRESHOLD_V_LOW),
                cv::Scalar(RED_THRESHOLD_H_HIGH, RED_THRESHOLD_S_HIGH, RED_THRESHOLD_V_HIGH), 
                red_frame);

    // Combine the image frames and apply post processing
    cv::Mat track_frame = yellow_frame + blue_frame + purple_frame + red_frame;
	cv::medianBlur(track_frame, track_frame, 7);

    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, track_frame);
    img_bridge.toImageMsg(_image);


    if (false) // this->get_parameter("is_sitl").as_bool()
    {
        cv::imshow(this->get_parameter("camera_topic").as_string(), track_frame);
        cv::waitKey(1);
    }

    clear_occupancy_grid_msg();
    add_bin_image_to_occupancy(track_frame);

    publish();
}


void
PotentialFieldDriver::set_occupancy_grid_tile(uint32_t row_index, uint32_t column_index, int8_t value)
{
    if (row_index >= COSTMAP_HEIGHT)
    {
        RCLCPP_ERROR(this->get_logger(), "Attempt to access out of bounds row. Requested row: %d. Max height: %d.", row_index, COSTMAP_HEIGHT);
        return;
    }

    if (column_index >= COSTMAP_WIDTH)
    {
        RCLCPP_ERROR(this->get_logger(), "Attempt to access out of bounds column. Requested column: %d. Max width: %d.", column_index, COSTMAP_WIDTH);
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
    //     RCLCPP_INFO(this->get_logger(), "Centroid [x, y] : [%d, %d]", mid.x, mid.y);
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
    y_m = forward_distance_m * std::tan(-bearing_deg * M_PI / 180.0);
}

uint32_t
PotentialFieldDriver::scale(uint32_t value, uint32_t old_min, uint32_t old_max, uint32_t new_min, uint32_t new_max)
{
    return round(((static_cast<double>(value - old_min) / static_cast<double>(old_max - old_min)) * (new_max - new_min)) + new_min);
}

void
PotentialFieldDriver::publish()
{
    _occupancy_grid.info.map_load_time = rclcpp::Node::now();
    _occupancy_grid.header.stamp = rclcpp::Node::now();
    _occupancy_grid.header.frame_id = this->get_parameter("frame_id").as_string();
    _potential_field_publisher->publish(_occupancy_grid);
    _physical_camera_publisher->publish(_image);
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PotentialFieldDriver>());
	return 0;
}