#include "vision_driver.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class PotentialFieldDriver : public VisionDriver
{
    public:
        PotentialFieldDriver();

        void translate(int32_t x_translation, int32_t y_translation);
        void fade(uint8_t fade_magnitude);
        void publish();
        void add_bin_image_to_occupancy(cv::Mat binary_image);
        uint32_t scale(uint32_t value, uint32_t old_min, uint32_t old_max, uint32_t new_min, uint32_t new_max);

        void set_tile(uint32_t row_index, uint32_t column_index, int8_t value);
        int8_t get_tile(uint32_t row_index, uint32_t column_index);
        void clear();

    private:
        void analyse_frame(cv::Mat image_frame) override;

        void _initialise_costmap();
        
        static constexpr uint32_t COSTMAP_WIDTH = 100;
        static constexpr uint32_t COSTMAP_HEIGHT = 100;
        static constexpr uint64_t COSTMAP_LENGTH = COSTMAP_WIDTH * COSTMAP_HEIGHT;

        int8_t costmap[COSTMAP_LENGTH];

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _potential_field_publisher;
};