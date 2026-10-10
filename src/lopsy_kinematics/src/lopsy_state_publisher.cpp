#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lopsy_kinematics/tendon_kinematics.hpp"
#include "lopsy_interfaces/msg/tendon_forces.hpp"

using TendonForces = lopsy_interfaces::msg::TendonForces;
using JointState = sensor_msgs::msg::JointState;
using namespace std::placeholders;

class LopsyStatePublisherNode : public rclcpp::Node
{
public:
    LopsyStatePublisherNode() : Node("lopsy_state_publisher"){
        joint_state_publisher_ = this -> create_publisher<JointState>("joint_states", 10);
        input_force_subscriber_ = this -> create_subscription<TendonForces>("tendon_forces", 10, std::bind(&LopsyStatePublisherNode::publishJointStates, this, _1));
    }

    void publishJointStates([[maybe_unused]] const TendonForces& forces){

    }

private:
    rclcpp::Publisher<JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<TendonForces>::SharedPtr input_force_subscriber_;



};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LopsyStatePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

