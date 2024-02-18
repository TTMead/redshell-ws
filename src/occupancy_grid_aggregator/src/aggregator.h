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
		void update();
		void publish();
		void initialise_occupancy_grid_msg();
		void potential_field_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
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
            tile_location.point.x = grid.info.origin.position.x + (row * grid.info.resolution);
            tile_location.point.y = grid.info.origin.position.y + (col * grid.info.resolution);
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

		nav_msgs::msg::OccupancyGrid _aggregated_occupancy_grid;
		rclcpp::TimerBase::SharedPtr _timer;

        std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

		std::vector<field_sub> _potential_field_subs;
		rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _aggregate_grid_pub;
};