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

    void publishJointStates(const TendonForces& tendon_forces){
        JointState msg;
        const std::vector<double> forces = tendon_forces.tendong_forces;

        joint_positions_ = tendon_kinematics::calculate_joint_angles(forces);

        msg.header.stamp = this -> now();
        msg.name = joint_names_;
        msg.position = joint_positions_;

        joint_state_publisher_ -> publish(msg);

    }

private:
    rclcpp::Publisher<JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<TendonForces>::SharedPtr input_force_subscriber_;

    const std::vector<std::string> joint_names_ = {
    "j1_roll_l",  "j1_roll_r",  "j2_roll_l",  "j2_roll_r",  "j3_roll_l",  "j3_roll_r",
    "j4_roll_l",  "j4_roll_r",  "j5_roll_l",  "j5_roll_r",  "j6_roll_l",  "j6_roll_r",
    "j7_roll_l",  "j7_roll_r",  "j8_roll_l",  "j8_roll_r",  "j9_roll_l",  "j9_roll_r",
    "j10_roll_l", "j10_roll_r", "j11_roll_l", "j11_roll_r", "j12_roll_l", "j12_roll_r",
    "j13_roll_l", "j13_roll_r", "j14_roll_l", "j14_roll_r", "j15_roll_l", "j15_roll_r"
    };
    std::vector<double> joint_positions_ = std::vector<double>(joint_names_.size(), 0.0);
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LopsyStatePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

