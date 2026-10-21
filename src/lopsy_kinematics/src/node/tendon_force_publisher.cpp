#include "rclcpp/rclcpp.hpp"
#include "lopsy_interfaces/msg/tendon_forces.hpp"

using TendonForces = lopsy_interfaces::msg::TendonForces;

class TendonForcePublisherNode : public rclcpp::Node
{
public:
    TendonForcePublisherNode() : Node("tendon_force_publisher"){
        force_publisher_ = this -> create_publisher<TendonForces>("tendon_forces", 10);
        timer_ = this -> create_wall_timer(period_,std::bind(&TendonForcePublisherNode::publishInputForces, this));
    }

    void publishInputForces(){
        TendonForces msg;
        msg.tendong_forces = std::vector<double>(20,0.0);
        force_publisher_ -> publish(msg);
    }

private:
    rclcpp::Publisher<TendonForces>::SharedPtr force_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    const std::chrono::milliseconds period_{1000};
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TendonForcePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}