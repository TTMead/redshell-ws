#include "potential_field_driver.hpp"

PotentialFieldDriver::PotentialFieldDriver() : VisionDriver("track_error_driver")
{
    _initialise_costmap();

    _potential_field_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("potential_field", 10);
}

void
PotentialFieldDriver::analyse_frame(cv::Mat image_frame)
{
	// Convert the frame to hsv
	cv::cvtColor(image_frame, image_frame, cv::COLOR_BGR2HSV);

	// Threshold the frame by colour
	cv::Mat yellow_frame, blue_frame;
	cv::inRange(image_frame, cv::Scalar(25, 30, 0), cv::Scalar(35, 255, 255), yellow_frame);
	cv::inRange(image_frame, cv::Scalar(90, 50, 190), cv::Scalar(130, 250, 250), blue_frame);

	// Apply median blur to each frame
	cv::medianBlur(yellow_frame, yellow_frame, 7);
	cv::medianBlur(blue_frame, blue_frame, 7);

    clear();
    add_bin_image_to_occupancy(yellow_frame);
    add_bin_image_to_occupancy(blue_frame);
    
    set_tile(0, 0, 5);


    publish();
}

void
PotentialFieldDriver::add_bin_image_to_occupancy(cv::Mat binary_image)
{
    std::vector<cv::Point> locations;
    cv::findNonZero(binary_image, locations);

    RCLCPP_INFO(this->get_logger(), "num pix: [%d]", locations.size());

    for (cv::Point point : locations)
    {
        uint32_t row_index = scale(point.y, 0, binary_image.rows, 0, COSTMAP_HEIGHT-1);
        uint32_t column_index = scale(point.x, 0, binary_image.cols, 0, COSTMAP_WIDTH-1);

        uint8_t new_value = get_tile(row_index, column_index) + 80;
        set_tile(row_index, column_index, new_value);
    }
}

uint32_t
PotentialFieldDriver::scale(uint32_t value, uint32_t old_min, uint32_t old_max, uint32_t new_min, uint32_t new_max)
{
    return round(((static_cast<float>(value - old_min) / static_cast<float>(old_max - old_min)) * (new_max - new_min)) + new_min);
}

void
PotentialFieldDriver::publish()
{
    nav_msgs::msg::OccupancyGrid msg;

    msg.header.stamp = rclcpp::Node::now();
    msg.header.frame_id = "map";
    float costmap_resolution = 0.05;
    msg.info.resolution = costmap_resolution; // [m/cell]
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