#include "lopsy_kinematics/statics.hpp"

#include <algorithm>
#include <cmath>

namespace lopsy_kinematics
{

namespace
{

/// Sum of both rods' tensions. Used where only the total axial load matters.
double total(const RodTensions& t)
{
  return t.left + t.right;
}

}  // namespace

ValueAndDerivative joint_residual(const Link& k, double theta, const RodTensions& T_below,
                                  const RobotParams& params, const SolverSettings& settings)
{
  const ValueAndDerivative R = contact_radius(k, theta, settings);
  const ValueAndDerivative M = elastic_moment(k, theta, params, settings);
  const double s = std::sin(theta);
  const double c = std::cos(theta);

  // Lever arms of the two rods about the rolling contact point O_i.
  // Left rod: hole at +A_l, the contact moves left by R sin(theta).
  // Right rod: hole at -A_r, so its arm is A_r + R sin(theta).
  const double arm_left = k.A_l - R.value * s;
  const double arm_right = k.A_r + R.value * s;

  // d(R sin theta)/dtheta, shared by both arms (with opposite signs).
  const double d_Rs = R.derivative * s + R.value * c;

  ValueAndDerivative g;
  g.value = M.value - T_below.left * arm_left + T_below.right * arm_right;
  g.derivative = M.derivative + (T_below.left + T_below.right) * d_Rs;  // (C20) diagonal, > 0
  return g;
}

JointSolution solve_joint(const Link& k, const RodTensions& T_below, const RobotParams& params,
                          const SolverSettings& settings)
{
  JointSolution result;

  // Bracket: g < 0 at lo, g > 0 at hi, for ANY tensions. At theta = alpha the
  // left rod's arm A_l - R_l sin(alpha) is exactly 0 (sin alpha = A_l / R_l),
  // and beyond it the arm turns negative, so the left rod can never pull a
  // joint past its limit. The same holds for the right rod at -beta.
  double lo = -k.beta - settings.bracket_margin;
  double hi = k.alpha + settings.bracket_margin;

  const double g_lo = joint_residual(k, lo, T_below, params, settings).value;
  const double g_hi = joint_residual(k, hi, T_below, params, settings).value;
  if (!(g_lo < 0.0 && g_hi > 0.0))
  {
    // Cannot happen with physical inputs; catches NaN or nonsensical
    // parameters instead of returning a meaningless angle.
    result.theta = 0.0;
    return result;  // converged = false
  }

  // Start from the linearisation of g at theta = 0:
  //   g(theta) ~ g(0) + g'(0) theta,  g(0) = -T^L A_l + T^R A_r
  const ValueAndDerivative g0 = joint_residual(k, 0.0, T_below, params, settings);
  double theta = -g0.value / g0.derivative;
  if (!(theta > lo && theta < hi))
  {
    theta = 0.5 * (lo + hi);
  }

  for (result.iterations = 1; result.iterations <= settings.max_iterations; ++result.iterations)
  {
    const ValueAndDerivative g = joint_residual(k, theta, T_below, params, settings);
    if (g.value == 0.0)
    {
      result.converged = true;
      break;
    }

    // g is increasing, so the root is below theta if g > 0 and above it if g < 0.
    if (g.value > 0.0)
    {
      hi = theta;
    }
    else
    {
      lo = theta;
    }

    // Newton step; fall back to bisection if it would leave the bracket.
    double next = theta - g.value / g.derivative;
    if (!(next > lo && next < hi))
    {
      next = 0.5 * (lo + hi);
    }

    const bool small_step = std::abs(next - theta) < settings.angle_tolerance;
    theta = next;
    if (small_step)
    {
      result.converged = true;
      break;
    }
  }

  result.theta = theta;
  return result;
}

RodTensions tension_after(const RodTensions& T_below, double theta, const RobotParams& params,
                          const SolverSettings& settings)
{
  // (C6): capstan loss across one joint, wrap angle |theta|. Both rods wrap
  // the same angle, so they lose the same fraction.
  const double factor = std::exp(-params.mu * smooth_abs(theta, settings.abs_smoothing).value);
  return {T_below.left * factor, T_below.right * factor};
}

std::vector<double> contact_forces(const std::vector<Link>& links, const std::vector<double>& theta,
                                   const std::vector<RodTensions>& tension, const RobotParams& params)
{
  const std::size_t n = links.size();
  std::vector<double> FR(n, 0.0);
  if (n == 0 || theta.size() != n || tension.size() != n + 1)
  {
    return FR;  // inconsistent input: nothing sensible to compute
  }

  // (C14): absolute angle of each link's axis from the upward vertical.
  std::vector<double> psi(n);
  double accumulated = params.psi0;
  for (std::size_t k = 0; k < n; ++k)
  {
    accumulated += theta[k];
    psi[k] = accumulated;
  }

  // (C13), last link: the rods' tension below it, T_{n-1}, all goes into the contact.
  const std::size_t last = n - 1;
  FR[last] = (total(tension[last]) + params.G * std::cos(psi[last])) / std::cos(theta[last]);

  // (C11), links n-1 down to 1. tension[k] is the tension BELOW link k+1 and
  // tension[k+1] the tension above it, so their difference is the friction f
  // the rods leave on that link (C7).
  for (std::size_t k = last; k-- > 0;)
  {
    const double friction = total(tension[k]) - total(tension[k + 1]);
    FR[k] = (FR[k + 1] + friction + params.G * std::cos(psi[k])) / std::cos(theta[k]);
  }
  return FR;
}

State solve(const std::vector<Link>& links, const RodTensions& F0, const RobotParams& params,
            const SolverSettings& settings)
{
  const std::size_t n = links.size();

  State state;
  state.F0 = {std::max(F0.left, 0.0), std::max(F0.right, 0.0)};  // rods cannot push
  state.theta.assign(n, 0.0);
  state.tension.assign(n + 1, RodTensions{});

  if (state.F0.left == 0.0 && state.F0.right == 0.0)
  {
    state.converged = true;  // both rods slack: the straight robot
    return state;
  }

  state.tension[0] = state.F0;
  state.converged = true;
  for (std::size_t k = 0; k < n; ++k)
  {
    const JointSolution joint = solve_joint(links[k], state.tension[k], params, settings);
    state.theta[k] = joint.theta;
    state.tension[k + 1] = tension_after(state.tension[k], joint.theta, params, settings);
    state.converged = state.converged && joint.converged;
  }
  return state;
}

}  // namespace lopsy_kinematics