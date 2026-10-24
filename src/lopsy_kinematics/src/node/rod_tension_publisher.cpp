#include "rclcpp/rclcpp.hpp"
#include "lopsy_interfaces/msg/rod_tensions.hpp"

using RodTensions = lopsy_interfaces::msg::RodTensions;

class RodTensionPublisherNode : public rclcpp::Node
{
public:
    RodTensionPublisherNode() : Node("rod_tension_publisher"){
        declareTensionParam("left_tension", 20.0);
        declareTensionParam("right_tension", 0.0);
        force_publisher_ = this -> create_publisher<RodTensions>("rod_tensions", 10);
        timer_ = this -> create_wall_timer(period_,std::bind(&RodTensionPublisherNode::publishInputForces, this));
    }

private:
    rclcpp::Publisher<RodTensions>::SharedPtr force_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    const std::chrono::milliseconds period_{50};

    void declareTensionParam(std::string param_name, double default_value){
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = "Tendon Tension [N]";

        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = 50.0;
        range.step = 0.1;
        desc.floating_point_range.push_back(range);

        declare_parameter(param_name, default_value, desc);
    }

    void publishInputForces(){
        RodTensions msg;
        msg.left_tension = get_parameter("left_tension").as_double();
        msg.right_tension = get_parameter("right_tension").as_double();
        force_publisher_ -> publish(msg);
    }

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RodTensionPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}