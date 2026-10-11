#include "lopsy_kinematics/tendon_kinematics.hpp"

std::vector<double> tendon_kinematics::calculate_joint_angles([[maybe_unused]] const std::vector<double>& forces){
    std::vector<double> joint_positions = std::vector<double>(30,0.0);

    for (size_t i = 1; i < joint_positions.size(); i += 2) {
    joint_positions[i] = -0.1;
    }

    return joint_positions;
}