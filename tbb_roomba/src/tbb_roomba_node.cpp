#include "rclcpp/rclcpp.hpp"

class TbbRoombaNode : public rclcpp::Node {
public:
    TbbRoombaNode() : Node("tbb_roomba_node") {
        RCLCPP_INFO(this->get_logger(), "Tbb Roomba Node has been started.");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TbbRoombaNode>());
    rclcpp::shutdown();
    return 0;
}