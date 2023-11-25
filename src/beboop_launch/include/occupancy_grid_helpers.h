#ifndef OCCUPANCY_GRID_HELPERS_H
#define OCCUPANCY_GRID_HELPERS_H

#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

static constexpr uint32_t COSTMAP_WIDTH = 160;
static constexpr uint32_t COSTMAP_HEIGHT = 100;
static constexpr uint64_t COSTMAP_LENGTH = COSTMAP_WIDTH * COSTMAP_HEIGHT;
static constexpr float COSTMAP_RESOLUTION = 0.05;   // [m/cell]

typedef nav_msgs::msg::OccupancyGrid Grid;



void set_tile(Grid &grid, uint32_t row_index, uint32_t column_index, int8_t value)
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

    grid.data[column_index + (row_index*COSTMAP_WIDTH)] = value;
}


int8_t get_tile(Grid &grid, uint32_t row_index, uint32_t column_index)
{
    if (row_index >= COSTMAP_HEIGHT)
    {
        RCLCPP_ERROR(rclcpp::get_logger("occupancy_grid_helper"), "Attempt to access out of bounds row. Requested row: %d. Max height: %d.", row_index, COSTMAP_HEIGHT);
        return 0;
    }

    if (column_index >= COSTMAP_WIDTH)
    {
        RCLCPP_ERROR(rclcpp::get_logger("occupancy_grid_helper"), "Attempt to access out of bounds column. Requested column: %d. Max width: %d.", column_index, COSTMAP_WIDTH);
        return 0;
    }

    return grid.data[row_index + (column_index*COSTMAP_WIDTH)];
}


void initialise_occupancy_values(Grid &grid)
{
    for (uint32_t row_index = 0; row_index < COSTMAP_HEIGHT; row_index++)
    {
        for (uint32_t column_index = 0; column_index < COSTMAP_WIDTH; column_index++)
        {
            set_tile(grid, row_index, column_index, 0);
        }
    }
}


void translate(Grid &grid, int32_t x_translation, int32_t y_translation)
{
    if ((abs(x_translation) > COSTMAP_WIDTH/2) || (abs(y_translation) > COSTMAP_HEIGHT/2))
    {
        RCLCPP_ERROR(rclcpp::get_logger("occupancy_grid_helper"), "Translation magnitude too high [%d, %d]", x_translation, y_translation);
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
                set_tile(grid, row_index, column_index, 0);
            }
            else
            {
                set_tile(grid, row_index, column_index, get_tile(grid, old_row_index, old_column_index));
            }
        }
    }
}


void clear(Grid &grid)
{
    for (uint32_t i = 0; i < COSTMAP_LENGTH; i++)
    {
        grid.data[i] = 0;
    }
}


void add_costmaps(Grid &grid_a, Grid grid_b)
{
    for (uint32_t row_index = 0; row_index < COSTMAP_HEIGHT; row_index++)
    {
        for (uint32_t column_index = 0; column_index < COSTMAP_WIDTH; column_index++)
        {
            int8_t value = static_cast<std::clamp(
                static_cast<int16_t>(get_tile(grid_a, row_index, column_index))
              + static_cast<int16_t>get_tile(grid_b, row_index, column_index), INT8_MIN, INT8_MAX);
            
            set_tile(grid_a, row_index, column_index, value);
        }
    }
}


void fade(Grid &grid, uint8_t fade_magnitude)
{
    for (uint32_t row_index = 0; row_index < COSTMAP_HEIGHT; row_index++)
    {
        for (uint32_t column_index = 0; column_index < COSTMAP_WIDTH; column_index++)
        {
            int8_t old_val = get_tile(grid, row_index, column_index);
            set_tile(grid, row_index, column_index, signbit(old_val) * std::max(old_val - fade_magnitude, 0));
        }
    }
}


#endif /* OCCUPANCY_GRID_HELPERS_H */