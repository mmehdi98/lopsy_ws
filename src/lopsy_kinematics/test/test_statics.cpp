// Layer 3: the joint balance (C16) with both rods, the capstan law (C6) and
// the on-demand contact forces (C11)/(C13).
//
// These are properties any correct solution must have. For exact numbers,
// compare a left-rod sweep with the Python cut statics as well.

#include <cmath>
#include <numeric>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "lopsy_kinematics/link_geometry.hpp"
#include "lopsy_kinematics/statics.hpp"

using namespace lopsy_kinematics;

namespace
{

class Statics : public ::testing::Test
{
protected:
  RobotParams params;  // recommended set
  SolverSettings settings;
  std::vector<Link> links = build_links(params);

  void SetUp() override { ASSERT_EQ(links.size(), 15u) << "implement build_links() first"; }

  State pull_left(double F) const { return solve(links, RodTensions{F, 0.0}, params, settings); }
  State pull_right(double F) const { return solve(links, RodTensions{0.0, F}, params, settings); }
};

const std::vector<double> kForces = {3.0, 10.0, 20.0, 47.0};  // N

double sum(const std::vector<double>& v)
{
  return std::accumulate(v.begin(), v.end(), 0.0);
}

}  // namespace

// ---- Left rod (the tested configuration) -------------------------------------

TEST_F(Statics, ZeroTensionGivesTheStraightRobot)
{
  const State s = pull_left(0.0);
  ASSERT_EQ(s.theta.size(), links.size());
  for (double th : s.theta)
  {
    EXPECT_DOUBLE_EQ(th, 0.0);
  }
  EXPECT_TRUE(s.converged);
}

TEST_F(Statics, NegativeTensionIsTreatedAsSlack)
{
  const State s = solve(links, RodTensions{-5.0, 0.0}, params, settings);
  ASSERT_EQ(s.theta.size(), links.size());
  for (double th : s.theta)
  {
    EXPECT_DOUBLE_EQ(th, 0.0);
  }
}

TEST_F(Statics, OutputSizes)
{
  const State s = pull_left(20.0);
  EXPECT_EQ(s.theta.size(), links.size());
  EXPECT_EQ(s.tension.size(), links.size() + 1);
  EXPECT_TRUE(s.converged);
}

TEST_F(Statics, EveryJointSatisfiesTheBalance)
{
  for (double F : kForces)
  {
    for (const State& s : {pull_left(F), pull_right(F)})
    {
      ASSERT_EQ(s.theta.size(), links.size());
      ASSERT_EQ(s.tension.size(), links.size() + 1);
      for (std::size_t k = 0; k < links.size(); ++k)
      {
        const double g = joint_residual(links[k], s.theta[k], s.tension[k], params, settings).value;
        EXPECT_NEAR(g, 0.0, 1e-10) << "F = " << F << ", joint k = " << k;  // [N m]
      }
    }
  }
}

TEST_F(Statics, AnglesStayInsideTheJointRange)
{
  for (double F : kForces)
  {
    for (const State& s : {pull_left(F), pull_right(F)})
    {
      ASSERT_EQ(s.theta.size(), links.size());
      for (std::size_t k = 0; k < links.size(); ++k)
      {
        EXPECT_GT(s.theta[k], -links[k].beta);
        EXPECT_LT(s.theta[k], links[k].alpha);
      }
    }
  }
}

TEST_F(Statics, LeftRodBendsEveryJointLeft)
{
  // The left-rod term T (A_l - R sin theta) is positive at theta = 0, so every
  // joint, even ones included, bends left: no buckling (§10.2).
  const State s = pull_left(20.0);
  ASSERT_EQ(s.theta.size(), links.size());
  for (std::size_t k = 0; k < links.size(); ++k)
  {
    EXPECT_GT(s.theta[k], 0.0) << "joint k = " << k;
  }
}

TEST_F(Statics, LeftRodBendsOddJointsMoreThanEvenJoints)
{
  // Odd links have the long lever A1 on the left.
  const State s = pull_left(47.0);
  ASSERT_EQ(s.theta.size(), links.size());
  double odd = 0.0, even = 0.0;
  for (std::size_t k = 0; k < s.theta.size(); ++k)
  {
    (k % 2 == 0 ? odd : even) += s.theta[k];  // k even = odd link i
  }
  EXPECT_GT(odd / 8.0, even / 7.0);
}

TEST_F(Statics, TensionDecreasesAndFollowsTheCapstanLaw)
{
  for (double F : kForces)
  {
    const State s = pull_left(F);
    ASSERT_EQ(s.tension.size(), links.size() + 1);
    EXPECT_DOUBLE_EQ(s.tension.front().left, F);
    for (std::size_t i = 1; i < s.tension.size(); ++i)
    {
      EXPECT_LT(s.tension[i].left, s.tension[i - 1].left);
      EXPECT_DOUBLE_EQ(s.tension[i].right, 0.0) << "the slack rod stays slack";
    }
    double wrap = 0.0;
    for (double th : s.theta)
    {
      wrap += std::abs(th);
    }
    EXPECT_NEAR(s.tension.back().left, F * std::exp(-params.mu * wrap), 1e-3 * F);
  }
}

TEST_F(Statics, BendingGrowsWithTension)
{
  double previous = 0.0;
  for (double F : kForces)
  {
    const double total = sum(pull_left(F).theta);
    EXPECT_GT(total, previous) << "F = " << F;
    previous = total;
  }
}

TEST_F(Statics, ScalingTensionAndStiffnessTogetherLeavesTheAnglesUnchanged)
{
  // With G = 0, (F0, E) -> 2 (F0, E) gives the same angles (§10.2).
  RobotParams stiff = params;
  stiff.E *= 2.0;
  const State a = solve(links, RodTensions{20.0, 0.0}, params, settings);
  const State b = solve(links, RodTensions{40.0, 0.0}, stiff, settings);
  ASSERT_EQ(a.theta.size(), b.theta.size());
  ASSERT_FALSE(a.theta.empty());
  for (std::size_t k = 0; k < a.theta.size(); ++k)
  {
    EXPECT_NEAR(a.theta[k], b.theta[k], 1e-10);
  }
}

// ---- Right rod and both rods ---------------------------------------------------

TEST_F(Statics, RightRodBendsEveryJointRight)
{
  const State s = pull_right(20.0);
  ASSERT_EQ(s.theta.size(), links.size());
  for (std::size_t k = 0; k < links.size(); ++k)
  {
    EXPECT_LT(s.theta[k], 0.0) << "joint k = " << k;
  }
}

TEST_F(Statics, RightRodOnThisRobotIsTheLeftRodOnItsMirrorImage)
{
  // Mirroring the robot puts A1, R1 on the RIGHT of the odd links, which is
  // the same as swapping A1<->A2 and R1<->R2. Pulling the right rod of this
  // robot must give exactly the negated angles of pulling the left rod of the
  // mirrored one. This checks the right-rod term against the tested left one.
  RobotParams mirrored = params;
  std::swap(mirrored.A1, mirrored.A2);
  std::swap(mirrored.R1, mirrored.R2);
  const auto mirrored_links = build_links(mirrored);

  for (double F : kForces)
  {
    const State right = pull_right(F);
    const State left_of_mirror = solve(mirrored_links, RodTensions{F, 0.0}, mirrored, settings);
    ASSERT_EQ(right.theta.size(), left_of_mirror.theta.size());
    ASSERT_FALSE(right.theta.empty());
    for (std::size_t k = 0; k < right.theta.size(); ++k)
    {
      EXPECT_NEAR(right.theta[k], -left_of_mirror.theta[k], 1e-10) << "F = " << F << ", joint k = " << k;
    }
  }
}

TEST_F(Statics, PassiveTensionInTheRightRodReducesTheFirstJoint)
{
  // §10.2 extension: a tensioned passive rod resists the bend.
  const State slack = solve(links, RodTensions{20.0, 0.0}, params, settings);
  const State taut = solve(links, RodTensions{20.0, 2.0}, params, settings);
  ASSERT_FALSE(slack.theta.empty());
  ASSERT_FALSE(taut.theta.empty());
  EXPECT_LT(taut.theta[0], slack.theta[0]);
}

TEST_F(Statics, NoTensionCanPullAJointPastItsLimit)
{
  // At theta = alpha the rod's lever arm A_l - R sin(alpha) is exactly zero,
  // so even an absurd tension leaves every joint inside its range.
  for (const State& s : {pull_left(1.0e4), pull_right(1.0e4)})
  {
    EXPECT_TRUE(s.converged);
    ASSERT_EQ(s.theta.size(), links.size());
    for (std::size_t k = 0; k < links.size(); ++k)
    {
      EXPECT_GT(s.theta[k], -links[k].beta) << "joint k = " << k;
      EXPECT_LT(s.theta[k], links[k].alpha) << "joint k = " << k;
    }
  }
}

TEST_F(Statics, NewtonNeedsOnlyAFewIterations)
{
  const JointSolution sol = solve_joint(links[0], RodTensions{20.0, 0.0}, params, settings);
  EXPECT_TRUE(sol.converged);
  EXPECT_LE(sol.iterations, 8);
}

// ---- Contact forces (computed on demand, not part of the State) ------------------

TEST_F(Statics, LastContactForceCarriesTheTipTension)
{
  // (C13) with G = 0: F^R_n cos theta_n = T^L_{n-1} + T^R_{n-1}.
  for (const State& s : {pull_left(20.0), pull_right(20.0)})
  {
    ASSERT_EQ(s.theta.size(), links.size());
    ASSERT_EQ(s.tension.size(), links.size() + 1);
    const std::vector<double> FR = contact_forces(links, s.theta, s.tension, params);
    ASSERT_EQ(FR.size(), links.size());
    const std::size_t n = links.size();
    const double tip_tension = s.tension[n - 1].left + s.tension[n - 1].right;
    EXPECT_NEAR(FR[n - 1] * std::cos(s.theta[n - 1]), tip_tension, 1e-10);
  }
}

TEST_F(Statics, ContactForcesBalanceEachLink)
{
  // (C11) with G = 0: F^R_i cos theta_i - F^R_{i+1} - f_i = 0.
  for (const State& s : {pull_left(20.0), pull_right(20.0)})
  {
    ASSERT_EQ(s.theta.size(), links.size());
    ASSERT_EQ(s.tension.size(), links.size() + 1);
    const std::vector<double> FR = contact_forces(links, s.theta, s.tension, params);
    ASSERT_EQ(FR.size(), links.size());
    for (std::size_t k = 0; k + 1 < links.size(); ++k)
    {
      const double f = (s.tension[k].left - s.tension[k + 1].left) + (s.tension[k].right - s.tension[k + 1].right);
      EXPECT_NEAR(FR[k] * std::cos(s.theta[k]) - FR[k + 1] - f, 0.0, 1e-10);
    }
  }
}