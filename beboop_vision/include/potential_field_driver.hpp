#include "vision_driver.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

class PotentialFieldDriver : public VisionDriver
{
    public:
        PotentialFieldDriver();

        void translate(int32_t x_translation, int32_t y_translation);
        void fade(uint8_t fade_magnitude);
        void publish();

        void set_tile(uint32_t row_index, uint32_t column_index, int8_t value);
        int8_t get_tile(uint32_t row_index, uint32_t column_index);

    private:
        void analyse_frame(cv::Mat image_frame) override;

        void _initialise_costmap();
        
        static constexpr uint32_t COSTMAP_WIDTH = 2000;
        static constexpr uint32_t COSTMAP_HEIGHT = 2000;
        static constexpr uint64_t COSTMAP_LENGTH = COSTMAP_WIDTH * COSTMAP_HEIGHT;

        int8_t costmap[COSTMAP_LENGTH];

        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr _potential_field_publisher;
};