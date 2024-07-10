#include "vision_driver.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"

class PotentialFieldDriver : public VisionDriver
{
    public:
        PotentialFieldDriver();

    private:
        void analyse_frame(cv::Mat image_frame) override;
        void initialise_occupancy_grid_msg();
        void clear_occupancy_grid_msg();
        void pixels_to_m(double x_px, double y_px, double &x_m, double &y_m); // Converts a point in pixels to a point in meters using a calibrated empirical model
        void publish();
        void add_bin_image_to_occupancy(cv::Mat binary_image);
        void set_occupancy_grid_tile(uint32_t row_index, uint32_t column_index, int8_t value);
        uint32_t scale(uint32_t value, uint32_t old_min, uint32_t old_max, uint32_t new_min, uint32_t new_max);

        static constexpr uint8_t RED_THRESHOLD_H_LOW = 0;
        static constexpr uint8_t RED_THRESHOLD_H_HIGH = 170;
        static constexpr uint8_t RED_THRESHOLD_S_LOW = 150;
        static constexpr uint8_t RED_THRESHOLD_S_HIGH = 255;
        static constexpr uint8_t RED_THRESHOLD_V_LOW = 190;
        static constexpr uint8_t RED_THRESHOLD_V_HIGH = 255;

        static constexpr uint8_t BLUE_THRESHOLD_H_LOW = 105;
        static constexpr uint8_t BLUE_THRESHOLD_H_HIGH = 115;
        static constexpr uint8_t BLUE_THRESHOLD_S_LOW = 45;
        static constexpr uint8_t BLUE_THRESHOLD_S_HIGH = 179;
        static constexpr uint8_t BLUE_THRESHOLD_V_LOW = 175;
        static constexpr uint8_t BLUE_THRESHOLD_V_HIGH = 255;

        static constexpr uint8_t YELLOW_THRESHOLD_H_LOW = 25;
        static constexpr uint8_t YELLOW_THRESHOLD_H_HIGH = 35;
        static constexpr uint8_t YELLOW_THRESHOLD_S_LOW = 40;
        static constexpr uint8_t YELLOW_THRESHOLD_S_HIGH = 70;
        static constexpr uint8_t YELLOW_THRESHOLD_V_LOW = 190;
        static constexpr uint8_t YELLOW_THRESHOLD_V_HIGH = 245;
 
        static constexpr uint8_t PURPLE_THRESHOLD_H_LOW = 135;
        static constexpr uint8_t PURPLE_THRESHOLD_H_HIGH = 165;
        static constexpr uint8_t PURPLE_THRESHOLD_S_LOW = 50;
        static constexpr uint8_t PURPLE_THRESHOLD_S_HIGH = 140;
        static constexpr uint8_t PURPLE_THRESHOLD_V_LOW = 50;
        static constexpr uint8_t PURPLE_THRESHOLD_V_HIGH = 225;

        static constexpr uint32_t COSTMAP_WIDTH = 160;
        static constexpr uint32_t COSTMAP_HEIGHT = 160;
        static constexpr uint64_t COSTMAP_LENGTH = COSTMAP_WIDTH * COSTMAP_HEIGHT;
        static constexpr float COSTMAP_RESOLUTION = 0.05;   // [m/cell]

        nav_msgs::msg::OccupancyGrid _occupancy_grid;
        sensor_msgs::msg::Image _image;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _potential_field_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _physical_camera_publisher;
};