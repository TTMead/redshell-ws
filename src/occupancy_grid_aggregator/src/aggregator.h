#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class Aggregator : public rclcpp::Node
{
	typedef rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr field_sub;

	public:
		Aggregator();

	private:
        /**
         * @brief 
         */
		void filter_costmap();

        /**
         * @brief Publishes the aggregated costmap.
         */
		void publish_costmap();

        /**
         * @brief Initialises the internal aggregate occupancy_grid.
         */
		void initialise_occupancy_grid_msg();

        /**
         * @brief Callback on the potential field subscriptions to combine.
         */
		void potential_field_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        /**
         * @brief Merges the information from a new occupancy_grid to an existing occupancy_grid.
         * 
         * @param grid The occupancy_grid to by altered.
         * @param new_grid The occupancy_grid to be merged.
         */
        void combine_costmaps(nav_msgs::msg::OccupancyGrid& grid, const nav_msgs::msg::OccupancyGrid& new_grid);

        /**
         * @brief Returns the world location of a given tile within an occupancy grid.
         * 
         * @param grid The occupancy grid to extract the tile from.
         * @param index The index of the tile to extract.
         * 
         * @return The Point location of the tile in the grid's frame_id
         */
        static inline geometry_msgs::msg::PointStamped get_point_from_index(const nav_msgs::msg::OccupancyGrid& grid, const int index)
        {
            const uint32_t col = index % grid.info.width;
            const uint32_t row = std::floor(index / grid.info.width);

            geometry_msgs::msg::PointStamped tile_location{};
            tile_location.point.x = grid.info.origin.position.x + (static_cast<float>(row) * grid.info.resolution);
            tile_location.point.y = grid.info.origin.position.y + (static_cast<float>(col - (grid.info.width/2)) * grid.info.resolution);
            tile_location.header.frame_id = grid.header.frame_id;

            return tile_location;
        }

        /**
         * @brief Adds a value from a given world point to an occupancy grid tile. If the world point exists outside
         * the bounds of the occupancy grid, nothing is done.
         * 
         * @param grid The occupancy grid to add to.
         * @param point The point to map to a tile.
         * @param value The value to fuse into the grid at the location of point.
         */
        static inline void add_point_to_grid(nav_msgs::msg::OccupancyGrid& grid, const geometry_msgs::msg::Point& point, const int8_t value)
        {
            const double x_rel = point.x - grid.info.origin.position.x;
            const double y_rel = point.y - grid.info.origin.position.y;

            if ((x_rel < 0) || (y_rel < 0))
            {
                return;
            }

            const uint32_t col = std::floor(x_rel / grid.info.resolution);
            const uint32_t row = std::floor(y_rel / grid.info.resolution);

            if ((col >= grid.info.width) || (row >= grid.info.height))
            {
                return;
            }

            const uint64_t index = col + (row * grid.info.width);
            grid.data[index] = value;
        }

        static inline void median_filter(nav_msgs::msg::OccupancyGrid& grid)
        {
            nav_msgs::msg::OccupancyGrid new_grid = grid;

            // For each cell
            for (int32_t col = 1; col < static_cast<int32_t>(grid.info.width - 1); col++)
            {
                for (int32_t row = 1; row < static_cast<int32_t>(grid.info.height - 1); row++)
                {
                    // Apply a 3x3 median filter kernel
                    std::vector<int8_t> kernel;
                    for (int8_t x = -1; x < 2; x++)
                    {
                        for (int8_t y = -1; y < 2; y++)
                        {
                            const uint64_t kernel_index = (col + x) + ((row + y) * grid.info.width);
                            kernel.push_back(grid.data[kernel_index]);
                        }
                    }
                    const uint64_t cell_index = col + (row * grid.info.width);
                    new_grid.data[cell_index] = kernel[4];
                }
            }

            grid = new_grid;
        }

        static inline void fade(nav_msgs::msg::OccupancyGrid& grid, int8_t magnitude)
        {
            for (int32_t col = 1; col < static_cast<int32_t>(grid.info.width - 1); col++)
            {
                for (int32_t row = 1; row < static_cast<int32_t>(grid.info.height - 1); row++)
                {
                    const uint64_t cell_index = col + (row * grid.info.width);
                    int8_t faded_value = std::max(grid.data[cell_index] + magnitude, 0);
                    grid.data[cell_index] = faded_value;
                }
            }
        }

		nav_msgs::msg::OccupancyGrid _aggregated_occupancy_grid;
		rclcpp::TimerBase::SharedPtr _publish_timer;
		rclcpp::TimerBase::SharedPtr _filter_timer;

        std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

		std::vector<field_sub> _potential_field_subs;
		rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _aggregate_grid_pub;
};