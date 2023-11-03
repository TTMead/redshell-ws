#include "vision_driver.hpp"

class PotentialFieldDriver : public VisionDriver
{
    public:
        PotentialFieldDriver();

        void add_point(uint32_t x_position, uint32_t y_position, int8_t value);
        void translate(int32_t x_translation, int32_t y_translation);
        void fade(uint8_t fade_magnitude);

    private:
        void analyse_frame(cv::Mat image_frame) override;

        void _initialise_costmap();
        
        static constexpr uint32_t COSTMAP_WIDTH = 2000;
        static constexpr uint32_t COSTMAP_HEIGHT = 2000;

        int8_t costmap[COSTMAP_WIDTH][COSTMAP_HEIGHT];
};