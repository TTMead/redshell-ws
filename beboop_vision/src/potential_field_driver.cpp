#include "potential_field_driver.hpp"

PotentialFieldDriver::PotentialFieldDriver() : VisionDriver("track_error_driver")
{
    _initialise_costmap();

    _potential_field_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("potential_field", 10);
}

void
PotentialFieldDriver::analyse_frame(cv::Mat image_frame)
{
    // ToDo: Implement frame converison to costmap
    (void)image_frame;
}

void
PotentialFieldDriver::publish()
{
    nav_msgs::msg::OccupancyGrid msg;

    msg.header.stamp = rclcpp::Node::now();
    msg.header.frame_id = "base_link";
    msg.info.resolution = 0.1; // [m/cell]
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
    costmap[column_index + (row_index*COSTMAP_WIDTH)] = value;
}

int8_t
PotentialFieldDriver::get_tile(uint32_t row_index, uint32_t column_index)
{
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