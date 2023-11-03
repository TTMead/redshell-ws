#include "potential_field_driver.hpp"

PotentialFieldDriver::PotentialFieldDriver() : VisionDriver("track_error_driver")
{
    _initialise_costmap();
}

void
PotentialFieldDriver::analyse_frame(cv::Mat image_frame)
{
    // ToDo: Implement frame converison to costmap
    (void)image_frame;
}

void
PotentialFieldDriver::_initialise_costmap()
{
    for (uint32_t row_index = 0; row_index < COSTMAP_WIDTH; row_index++)
    {
        for (uint32_t column_index = 0; column_index < COSTMAP_HEIGHT; column_index++)
        {
            costmap[row_index][column_index] = 0;
        }
    }
}


void
PotentialFieldDriver::add_point(uint32_t x_position, uint32_t y_position, int8_t value)
{
    costmap[x_position][y_position] = value;
}

void
PotentialFieldDriver::translate(int32_t x_translation, int32_t y_translation)
{
    if ((abs(x_translation) > COSTMAP_WIDTH/2) || (abs(y_translation) > COSTMAP_HEIGHT/2))
    {
        RCLCPP_ERROR(this->get_logger(), "Translation magnitude too high [%d, %d]", x_translation, y_translation);
        return;
    }

    for (uint32_t row_index = 0; row_index < COSTMAP_WIDTH; row_index++)
    {
        for (uint32_t column_index = 0; column_index < COSTMAP_HEIGHT; column_index++)
        {
            int64_t old_row_index = row_index + x_translation;
            int64_t old_column_index = column_index + y_translation;

            if ((old_row_index > COSTMAP_WIDTH)||
                (old_row_index < 0) ||
                (old_column_index > COSTMAP_HEIGHT) ||
                (old_column_index < 0))
            {
                costmap[row_index][column_index] = 0;
            }
            else
            {
                costmap[row_index][column_index] = costmap[old_row_index][old_column_index];
            }
        }
    }
}


void
PotentialFieldDriver::fade(uint8_t fade_magnitude)
{
    for (uint32_t row_index = 0; row_index < COSTMAP_WIDTH; row_index++)
    {
        for (uint32_t column_index = 0; column_index < COSTMAP_HEIGHT; column_index++)
        {
            int8_t old_val = costmap[row_index][column_index];

            costmap[row_index][column_index] = signbit(old_val) * std::max(old_val - fade_magnitude, 0);
        }
    }
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PotentialFieldDriver>());
	return 0;
}