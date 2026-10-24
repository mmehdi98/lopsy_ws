#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lopsy_kinematics/joint_state_mapping.hpp"
#include "lopsy_kinematics/model.hpp"
#include "lopsy_interfaces/msg/rod_tensions.hpp"

using JointState = sensor_msgs::msg::JointState;
using namespace std::placeholders;

namespace lopsy_kinematics
{

class LopsyStatePublisherNode : public rclcpp::Node
{
public:
    LopsyStatePublisherNode() : Node("lopsy_state_publisher"){
        params_ = declare_robot_params();
        solver_settings_ = declare_solver_settings();
        model_ = std::make_shared<ContinuumRobotModel>(params_, solver_settings_);
        joint_names_ = urdf_joint_names(model_->params().n);
        joint_positions_ = std::vector<double>(2*model_->params().n,0.0);

        joint_state_publisher_ = this -> create_publisher<JointState>("joint_states", 10);
        input_force_subscriber_ = this -> create_subscription<lopsy_interfaces::msg::RodTensions>("rod_tensions", 10, std::bind(&LopsyStatePublisherNode::publishJointStates, this, _1));
    }

    void publishJointStates(const lopsy_interfaces::msg::RodTensions& rod_tensions){
        JointState msg;
        const RodTensions F0 {.left=rod_tensions.left_tension, .right=rod_tensions.right_tension};

        auto rob_state = model_ -> solve(F0);
        joint_positions_ = urdf_joint_positions(rob_state.theta);
        
        // for (size_t i = 1; i < joint_positions_.size(); i += 2) {
        //     joint_positions_[i] = -0.1;
        // }

        msg.header.stamp = this -> now();
        msg.name = joint_names_;
        msg.position = joint_positions_;

        joint_state_publisher_ -> publish(msg);

    }

private:
    rclcpp::Publisher<JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<lopsy_interfaces::msg::RodTensions>::SharedPtr input_force_subscriber_;
    std::shared_ptr<lopsy_kinematics::ContinuumRobotModel> model_;
    RobotParams params_;
    SolverSettings solver_settings_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;

    // const std::vector<std::string> joint_names_ = {
    // "j1_roll_l",  "j1_roll_r",  "j2_roll_l",  "j2_roll_r",  "j3_roll_l",  "j3_roll_r",
    // "j4_roll_l",  "j4_roll_r",  "j5_roll_l",  "j5_roll_r",  "j6_roll_l",  "j6_roll_r",
    // "j7_roll_l",  "j7_roll_r",  "j8_roll_l",  "j8_roll_r",  "j9_roll_l",  "j9_roll_r",
    // "j10_roll_l", "j10_roll_r", "j11_roll_l", "j11_roll_r", "j12_roll_l", "j12_roll_r",
    // "j13_roll_l", "j13_roll_r", "j14_roll_l", "j14_roll_r", "j15_roll_l", "j15_roll_r"
    // };

    RobotParams declare_robot_params()
    {
        const RobotParams d;  // defaults
        RobotParams p;
        p.n = declare_parameter<int64_t>("n_links", d.n);

        p.L = declare_parameter("geometry.L", d.L);
        p.D = declare_parameter("geometry.D", d.D);
        p.A1 = declare_parameter("geometry.A1", d.A1);
        p.A2 = declare_parameter("geometry.A2", d.A2);
        p.R1 = declare_parameter("geometry.R1", d.R1);
        p.R2 = declare_parameter("geometry.R2", d.R2);

        p.r = declare_parameter("rods.r", d.r);
        p.E = declare_parameter("rods.E", d.E);
        p.mu = declare_parameter("rods.mu", d.mu);
        p.lambda0 = declare_parameter("rods.lambda0", d.lambda0);

        p.G = declare_parameter("gravity.G", d.G);
        p.x_cg = declare_parameter("gravity.x_cg", d.x_cg);
        p.y_cg = declare_parameter("gravity.y_cg", d.y_cg);
        p.psi0 = declare_parameter("gravity.psi0", d.psi0);
        return p;
    }

    SolverSettings declare_solver_settings()
    {
        const SolverSettings d;
        SolverSettings s;
        s.blend_width = declare_parameter("solver.blend_width", d.blend_width);
        s.abs_smoothing = declare_parameter("solver.abs_smoothing", d.abs_smoothing);
        s.bracket_margin = declare_parameter("solver.bracket_margin", d.bracket_margin);
        s.stop_stiffness = declare_parameter("solver.stop_stiffness", d.stop_stiffness);
        s.angle_tolerance = declare_parameter("solver.angle_tolerance", d.angle_tolerance);
        s.max_iterations = declare_parameter<int64_t>("solver.max_iterations", d.max_iterations);
        return s;
    }
    
};

} // namespace lopsy_kinematics

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lopsy_kinematics::LopsyStatePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

