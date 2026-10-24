#include "rclcpp/rclcpp.hpp"
#include "lopsy_interfaces/msg/rod_tensions.hpp"

using RodTensions = lopsy_interfaces::msg::RodTensions;

class RodTensionPublisherNode : public rclcpp::Node
{
public:
    RodTensionPublisherNode() : Node("tendon_force_publisher"){
        force_publisher_ = this -> create_publisher<RodTensions>("tendon_forces", 10);
        timer_ = this -> create_wall_timer(period_,std::bind(&RodTensionPublisherNode::publishInputForces, this));
    }

    void publishInputForces(){
        RodTensions msg;
        
        msg.left_tension = 20.0;
        msg.right_tension = 0.0;

        force_publisher_ -> publish(msg);
    }

private:
    rclcpp::Publisher<RodTensions>::SharedPtr force_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    const std::chrono::milliseconds period_{1000};
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RodTensionPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}