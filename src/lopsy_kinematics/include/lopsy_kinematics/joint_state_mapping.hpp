#pragma once

// Translation between the model's n joint angles and the 2n revolute joints
// of lopsy.urdf.xacro. Kept free of ROS so it can be unit-tested.
//
// Each rolling joint i is two revolute joints in the URDF, one per arc:
//   j{i}_roll_l  in [0, alpha]  carries theta_i when theta_i >= 0
//   j{i}_roll_r  in [-beta, 0]  carries theta_i when theta_i <  0
// The prismatic j{i}_slide_l / j{i}_slide_r are <mimic> joints;
// robot_state_publisher fills them in, so they are not published.

#include <string>
#include <vector>

namespace lopsy_kinematics
{

/// "j1_roll_l", "j1_roll_r", "j2_roll_l", ..., "j{n}_roll_r"  (size 2n).
/// Build once at start-up, not per message.
std::vector<std::string> urdf_joint_names(int n);

/// Positions in the same order as urdf_joint_names():
///   [max(theta_1, 0), min(theta_1, 0), max(theta_2, 0), ...]  (size 2n).
std::vector<double> urdf_joint_positions(const std::vector<double>& theta);

}  // namespace lopsy_kinematics
