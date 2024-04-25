#include "rrt/rrt.h"

int main(int argc, char **argv)
{   
    try{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRT>());
    rclcpp::shutdown();
    }
    catch(const std::exception& e){
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    return 0;
}