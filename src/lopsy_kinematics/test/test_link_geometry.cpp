// Layer 1: per-link geometry (C1)-(C3) and the odd/even alternation (§3.2).
// Left and right are physical: odd links have A1, R1 on the left.

#include <cmath>

#include <gtest/gtest.h>

#include "lopsy_kinematics/link_geometry.hpp"

using namespace lopsy_kinematics;

namespace
{
constexpr double kRadToDeg = 180.0 / kPi;
}

TEST(LinkGeometry, BuildsOneLinkPerJoint)
{
  const auto links = build_links(default_params());
  EXPECT_EQ(links.size(), 15u);
}

TEST(LinkGeometry, OddLinkMatchesTheoryDoc)
{
  const auto links = build_links(default_params());
  ASSERT_EQ(links.size(), 15u);
  const Link& odd = links[0];  // link 1

  EXPECT_DOUBLE_EQ(odd.A_l, 3.0e-3);
  EXPECT_DOUBLE_EQ(odd.R_l, 6.7e-3);
  EXPECT_NEAR(odd.alpha * kRadToDeg, 26.60, 0.005);  // theory §3.3
  EXPECT_NEAR(odd.beta * kRadToDeg, 26.08, 0.005);
  EXPECT_NEAR(odd.Gamma, 1.0e-3, 1e-12);
  EXPECT_NEAR(odd.L_tl, 14.2908e-3, 1e-7);
  EXPECT_DOUBLE_EQ(odd.sigma, 1.0);
}

TEST(LinkGeometry, EvenLinkIsTheSwap)
{
  const auto links = build_links(default_params());
  ASSERT_EQ(links.size(), 15u);
  const Link& odd = links[0];
  const Link& even = links[1];  // link 2

  EXPECT_DOUBLE_EQ(even.A_l, odd.A_r);
  EXPECT_DOUBLE_EQ(even.R_l, odd.R_r);
  EXPECT_DOUBLE_EQ(even.alpha, odd.beta);
  EXPECT_DOUBLE_EQ(even.beta, odd.alpha);
  EXPECT_NEAR(even.Gamma, -1.0e-3, 1e-12);
  EXPECT_DOUBLE_EQ(even.sigma, -1.0);
}

TEST(LinkGeometry, AlternatesAlongTheChain)
{
  const auto links = build_links(default_params());
  ASSERT_EQ(links.size(), 15u);
  for (std::size_t k = 0; k < links.size(); ++k)
  {
    const double expected_sigma = (k % 2 == 0) ? 1.0 : -1.0;  // k even = odd link i
    EXPECT_DOUBLE_EQ(links[k].sigma, expected_sigma) << "link index k = " << k;
  }
}
