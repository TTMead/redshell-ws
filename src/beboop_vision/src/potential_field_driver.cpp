#include "potential_field_driver.hpp"
#include <math.h>

PotentialFieldDriver::PotentialFieldDriver() : VisionDriver("track_error_driver")
{
    _initialise_costmap();

    _potential_field_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("potential_field", 10);

    this->declare_parameter("y_pixel_to_distance_a", 269.0);
    this->declare_parameter("y_pixel_to_distance_b", -300.64);
    this->declare_parameter("x_pixel_to_bearing_a", 0.0850541);
    this->declare_parameter("x_pixel_to_bearing_b", -52.4552);
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

    clear();
    add_bin_image_to_occupancy(track_frame);

    publish();
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

        uint8_t new_value = get_tile(row_index, column_index) + 80;
        set_tile(row_index, column_index, new_value);
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
    nav_msgs::msg::OccupancyGrid msg;

    msg.header.stamp = rclcpp::Node::now();
    msg.header.frame_id = "map";
    msg.info.resolution = COSTMAP_RESOLUTION;
    msg.info.width = COSTMAP_WIDTH;
    msg.info.height = COSTMAP_HEIGHT;
    msg.info.origin.position.x = 0;
    msg.info.origin.position.y = 0;
    msg.info.origin.position.z = 0;
    msg.info.origin.orientation.x = 0;
    msg.info.origin.orientation.y = 0;
    msg.info.origin.orientation.z = 0;
    msg.info.origin.orientation.w = 1;

    msg.data.clear();
    msg.data.insert(msg.data.end(), &costmap[0], &costmap[COSTMAP_LENGTH]);

    _potential_field_publisher->publish(msg);
}


void
PotentialFieldDriver::set_tile(uint32_t row_index, uint32_t column_index, int8_t value)
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

    costmap[column_index + (row_index*COSTMAP_WIDTH)] = value;
}

int8_t
PotentialFieldDriver::get_tile(uint32_t row_index, uint32_t column_index)
{
    if (row_index >= COSTMAP_HEIGHT)
    {
        RCLCPP_ERROR(this->get_logger(), "Attempt to access out of bounds row. Requested row: %d. Max height: %d.", row_index, COSTMAP_HEIGHT);
        return 0;
    }

    if (column_index >= COSTMAP_WIDTH)
    {
        RCLCPP_ERROR(this->get_logger(), "Attempt to access out of bounds column. Requested column: %d. Max width: %d.", column_index, COSTMAP_WIDTH);
        return 0;
    }

    return costmap[row_index + (column_index*COSTMAP_WIDTH)];
}


void
PotentialFieldDriver::_initialise_costmap()
{
    for (uint32_t row_index = 0; row_index < COSTMAP_HEIGHT; row_index++)
    {
        for (uint32_t column_index = 0; column_index < COSTMAP_WIDTH; column_index++)
        {
            set_tile(row_index, column_index, 0);
        }
    }
}

void
PotentialFieldDriver::translate(int32_t x_translation, int32_t y_translation)
{
    if ((abs(x_translation) > COSTMAP_WIDTH/2) || (abs(y_translation) > COSTMAP_HEIGHT/2))
    {
        RCLCPP_ERROR(this->get_logger(), "Translation magnitude too high [%d, %d]", x_translation, y_translation);
        return;
    }

    for (uint32_t row_index = 0; row_index < COSTMAP_HEIGHT; row_index++)
    {
        for (uint32_t column_index = 0; column_index < COSTMAP_WIDTH; column_index++)
        {
            int64_t old_row_index = row_index + x_translation;
            int64_t old_column_index = column_index + y_translation;

            if ((old_row_index > COSTMAP_HEIGHT)||
                (old_row_index < 0) ||
                (old_column_index > COSTMAP_WIDTH) ||
                (old_column_index < 0))
            {
                set_tile(row_index, column_index, 0);
            }
            else
            {
                set_tile(row_index, column_index, get_tile(old_row_index, old_column_index));
            }
        }
    }
}


void
PotentialFieldDriver::clear()
{
    for (uint32_t i = 0; i < COSTMAP_LENGTH; i++)
    {
        costmap[i] = 0;
    }
}


void
PotentialFieldDriver::fade(uint8_t fade_magnitude)
{
    for (uint32_t row_index = 0; row_index < COSTMAP_HEIGHT; row_index++)
    {
        for (uint32_t column_index = 0; column_index < COSTMAP_WIDTH; column_index++)
        {
            int8_t old_val = get_tile(row_index, column_index);
            set_tile(row_index, column_index, signbit(old_val) * std::max(old_val - fade_magnitude, 0));
        }
    }
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PotentialFieldDriver>());
	return 0;
}