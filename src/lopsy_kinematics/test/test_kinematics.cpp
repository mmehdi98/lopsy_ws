// Layer 4: forward kinematics (C15) and the URDF joint-state mapping.

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "lopsy_kinematics/joint_state_mapping.hpp"
#include "lopsy_kinematics/link_geometry.hpp"
#include "lopsy_kinematics/mechanics.hpp"

using namespace lopsy_kinematics;

namespace
{
// Angles well outside the +-0.2 deg arc blend, so R is exactly R_l or R_r.
const std::vector<double> kTestAngles = {-0.35, -0.12, -0.03, 0.03, 0.12, 0.35};
}  // namespace

TEST(JointStateMapping, NamesMatchTheUrdf)
{
  const auto names = urdf_joint_names(15);
  ASSERT_EQ(names.size(), 30u);
  EXPECT_EQ(names.front(), "j1_roll_l");
  EXPECT_EQ(names[1], "j1_roll_r");
  EXPECT_EQ(names[2], "j2_roll_l");
  EXPECT_EQ(names.back(), "j15_roll_r");
}

TEST(JointStateMapping, PositiveAndNegativeAnglesGoToTheirArc)
{
  const auto pos = urdf_joint_positions({0.1, -0.05, 0.0});
  ASSERT_EQ(pos.size(), 6u);
  EXPECT_DOUBLE_EQ(pos[0], 0.1);    // j1_roll_l
  EXPECT_DOUBLE_EQ(pos[1], 0.0);    // j1_roll_r
  EXPECT_DOUBLE_EQ(pos[2], 0.0);    // j2_roll_l
  EXPECT_DOUBLE_EQ(pos[3], -0.05);  // j2_roll_r
  EXPECT_DOUBLE_EQ(pos[4], 0.0);
  EXPECT_DOUBLE_EQ(pos[5], 0.0);
}