// Layer 2: one-joint mechanics (C4), (C8)-(C9), (C17), (C19).
//
// The "Derivative" tests compare each analytic derivative with a central
// difference. They only matter once you move from bisection to Newton.

#include <cmath>
#include <functional>
#include <vector>

#include <gtest/gtest.h>

#include "lopsy_kinematics/link_geometry.hpp"
#include "lopsy_kinematics/mechanics.hpp"

using namespace lopsy_kinematics;

namespace
{

// Central difference of f at x.
double central_difference(const std::function<double(double)>& f, double x, double h = 1e-6)
{
  return (f(x + h) - f(x - h)) / (2.0 * h);
}

// Angles away from 0 and from the joint limits, where both branches are smooth.
const std::vector<double> kTestAngles = {-0.30, -0.10, -0.02, 0.02, 0.10, 0.30};

Link odd_link()
{
  return build_links(default_params()).at(0);
}

Link even_link()
{
  return build_links(default_params()).at(1);
}

}  // namespace

TEST(Mechanics, BlendWeight)
{
  const double w = deg2rad(0.2);
  EXPECT_NEAR(blend_weight(0.0, w).value, 0.5, 1e-15);
  EXPECT_NEAR(blend_weight(10.0 * w, w).value, 1.0, 1e-8);
  EXPECT_NEAR(blend_weight(-10.0 * w, w).value, 0.0, 1e-8);
}

TEST(Mechanics, SmoothAbsIsAbsAwayFromZero)
{
  const double eps = 1e-4;
  for (double x : {-0.3, -0.05, 0.05, 0.3})
  {
    EXPECT_NEAR(smooth_abs(x, eps).value, std::abs(x), 2.0 * eps) << "x = " << x;
    EXPECT_DOUBLE_EQ(smooth_abs(x, eps).value, smooth_abs(-x, eps).value) << "must be even";
  }
}

TEST(Mechanics, ContactRadiusPicksTheRollingArc)
{
  const SolverSettings s;
  const Link k = odd_link();
  EXPECT_NEAR(contact_radius(k, +0.1, s).value, k.R_l, 1e-12);
  EXPECT_NEAR(contact_radius(k, -0.1, s).value, k.R_r, 1e-12);
}

TEST(Mechanics, GapHeightsAtZero)
{
  const Link k = odd_link();
  const GapHeights g = gap_heights_exact(k, 0.0);
  EXPECT_NEAR(g.closing.value, 0.70917e-3, 1e-8);  // R_l (1 - cos alpha), theory §5.4
  EXPECT_NEAR(g.opening.value, 0.23157e-3, 1e-8);  // R_r (1 - cos beta)
}

TEST(Mechanics, ClosingGapVanishesAtTheJointLimits)
{
  const Link k = odd_link();
  EXPECT_NEAR(gap_heights_exact(k, k.alpha).closing.value, 0.0, 1e-12);
  EXPECT_NEAR(gap_heights_exact(k, -k.beta).closing.value, 0.0, 1e-12);
}

TEST(Mechanics, ArcFactor)
{
  EXPECT_NEAR(arc_factor(0.0).value, 1.0, 1e-15);
  EXPECT_NEAR(arc_factor(0.5).value, 0.5 / std::sin(0.5), 1e-14);
  EXPECT_NEAR(arc_factor(-0.5).value, arc_factor(0.5).value, 1e-14);
  // Continuous across the switch to the series near 0.
  EXPECT_NEAR(arc_factor(0.99e-4).value, arc_factor(1.01e-4).value, 1e-9);
}

TEST(Mechanics, ElasticMomentIsZeroAtZeroAndOpposesTheBend)
{
  const RobotParams p;
  const SolverSettings s;
  for (const Link& k : {odd_link(), even_link()})
  {
    EXPECT_NEAR(elastic_moment(k, 0.0, p, s).value, 0.0, 1e-15);
    EXPECT_GT(elastic_moment(k, +0.1, p, s).value, 0.0);
    EXPECT_LT(elastic_moment(k, -0.1, p, s).value, 0.0);
  }
}

TEST(Mechanics, ElasticStiffnessAtZero)
{
  // dM/dtheta at 0 = E I (1/s_c + 1/s_o) with s = h(0) + 2 lambda0.
  // 1 % tolerance: the blended gap heights average the two branches at 0.
  const RobotParams p;
  const SolverSettings s;
  const Link k = odd_link();
  const double EI = p.E * second_moment_of_area(p.r);
  const double s_c = k.R_l * (1.0 - std::cos(k.alpha)) + 2.0 * p.lambda0;
  const double s_o = k.R_r * (1.0 - std::cos(k.beta)) + 2.0 * p.lambda0;
  const double expected = EI * (1.0 / s_c + 1.0 / s_o);
  EXPECT_NEAR(elastic_moment(k, 0.0, p, s).derivative, expected, 0.01 * expected);
}

TEST(Mechanics, JointStopOnlyActsPastTheLimits)
{
  const SolverSettings s;
  const Link k = odd_link();
  EXPECT_DOUBLE_EQ(joint_stop_moment(k, 0.5 * k.alpha, s).value, 0.0);
  EXPECT_DOUBLE_EQ(joint_stop_moment(k, -0.5 * k.beta, s).value, 0.0);
  EXPECT_NEAR(joint_stop_moment(k, k.alpha + 0.01, s).value, s.stop_stiffness * 0.01, 1e-12);
  EXPECT_NEAR(joint_stop_moment(k, -k.beta - 0.01, s).value, -s.stop_stiffness * 0.01, 1e-12);
}

TEST(Mechanics, DerivativeOfContactRadius)
{
  const SolverSettings s;
  const Link k = odd_link();
  for (double th : {-0.002, -0.001, 0.0, 0.001, 0.002})  // inside the blend, where R' != 0
  {
    const double fd = central_difference([&](double x) { return contact_radius(k, x, s).value; }, th, 1e-7);
    EXPECT_NEAR(contact_radius(k, th, s).derivative, fd, 1e-6 * std::abs(fd) + 1e-10) << "theta = " << th;
  }
}

TEST(Mechanics, DerivativeOfGapHeights)
{
  for (const Link& k : {odd_link(), even_link()})
  {
    for (double th : kTestAngles)
    {
      const double fd_c = central_difference([&](double x) { return gap_heights_exact(k, x).closing.value; }, th);
      const double fd_o = central_difference([&](double x) { return gap_heights_exact(k, x).opening.value; }, th);
      const GapHeights g = gap_heights_exact(k, th);
      EXPECT_NEAR(g.closing.derivative, fd_c, 1e-8) << "theta = " << th;
      EXPECT_NEAR(g.opening.derivative, fd_o, 1e-8) << "theta = " << th;
    }
  }
}

TEST(Mechanics, DerivativeOfElasticMoment)
{
  const RobotParams p;
  const SolverSettings s;
  for (const Link& k : {odd_link(), even_link()})
  {
    std::vector<double> angles = kTestAngles;
    angles.push_back(k.alpha + 0.05);  // inside the stop
    for (double th : angles)
    {
      const double fd = central_difference([&](double x) { return elastic_moment(k, x, p, s).value; }, th);
      const double an = elastic_moment(k, th, p, s).derivative;
      EXPECT_NEAR(an, fd, 1e-6 * std::abs(fd)) << "theta = " << th;
    }
  }
}
