#include "rclcpp/rclcpp.hpp"
#include "lopsy_interfaces/msg/tendon_forces.hpp"

using TendonForces = lopsy_interfaces::msg::TendonForces;

class TendonForcePublisherNode : public rclcpp::Node
{
public:
    TendonForcePublisherNode() : Node("tendon_force_publisher"){
        force_publisher_ = this -> create_publisher<TendonForces>("tendon_forces", 10);
    }

private:
    rclcpp::Publisher<TendonForces>::SharedPtr force_publisher_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TendonForcePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}