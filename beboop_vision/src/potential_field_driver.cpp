#include "potential_field_driver.hpp"

PotentialFieldDriver::PotentialFieldDriver() : VisionDriver("track_error_driver")
{
    _initialise_costmap();

    _potential_field_publisher = this->create_publisher<std_msgs::msg::UInt8MultiArray>("potential_field", 10);
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
    std_msgs::msg::UInt8MultiArray msg;

    msg.data.clear();
    msg.data.insert(msg.data.end(), &costmap[0], &costmap[COSTMAP_LENGTH]);
    msg.layout.data_offset = 0;
    msg.layout.dim[0].label = "height";
    msg.layout.dim[0].size = COSTMAP_HEIGHT;
    msg.layout.dim[0].stride = COSTMAP_HEIGHT * COSTMAP_WIDTH;
    msg.layout.dim[1].label = "width";
    msg.layout.dim[1].size = COSTMAP_WIDTH;
    msg.layout.dim[1].stride = COSTMAP_WIDTH;

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