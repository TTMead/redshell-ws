#include "ekf_supervisor.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EkfSupervisor>());
    rclcpp::shutdown();
    return 0;
}
