#include "lopsy_kinematics/mechanics.hpp"

#include <algorithm>
#include <cmath>

namespace lopsy_kinematics
{

namespace
{
// Helpers used only in this file. An unnamed namespace makes them invisible
// to other .cpp files (internal linkage), like `static` functions in C.

/// (C9a): exact gap heights when the link rolls on its LEFT arc (theta >= 0).
/// The left gap closes, the right gap opens.
GapHeights gap_heights_left_branch(const Link& k, double theta)
{
  const double s = std::sin(theta);
  const double c = std::cos(theta);
  const double drop_r = k.R_r * (1.0 - std::cos(k.beta));  // R_r (1 - cos beta), constant

  GapHeights g;
  g.closing.value = k.R_l * (1.0 - std::cos(k.alpha - theta));
  g.closing.derivative = -k.R_l * std::sin(k.alpha - theta);

  g.opening.value = k.A_r * s + drop_r * c + k.R_l * (1.0 - c);
  g.opening.derivative = k.A_r * c - drop_r * s + k.R_l * s;
  return g;
}

/// (C9b): exact gap heights when the link rolls on its RIGHT arc (theta < 0).
/// Written with a = -theta (= |theta| on this branch). The right gap closes,
/// the left gap opens. Derivatives are with respect to theta, so every
/// d/da picks up a factor da/dtheta = -1.
GapHeights gap_heights_right_branch(const Link& k, double theta)
{
  const double a = -theta;
  const double s = std::sin(a);
  const double c = std::cos(a);
  const double drop_l = k.R_l * (1.0 - std::cos(k.alpha));  // R_l (1 - cos alpha), constant

  GapHeights g;
  g.closing.value = k.R_r * (1.0 - std::cos(k.beta - a));
  g.closing.derivative = k.R_r * std::sin(k.beta - a);

  g.opening.value = k.A_l * s + drop_l * c + k.R_r * (1.0 - c);
  g.opening.derivative = -k.A_l * c + drop_l * s - k.R_r * s;
  return g;
}

/// w * a + (1 - w) * b for a blend weight w and two branches a, b,
/// with the product rule for the derivative.
ValueAndDerivative blend(const ValueAndDerivative& w, const ValueAndDerivative& a,
                         const ValueAndDerivative& b)
{
  ValueAndDerivative r;
  r.value = w.value * a.value + (1.0 - w.value) * b.value;
  r.derivative = w.derivative * (a.value - b.value) + w.value * a.derivative + (1.0 - w.value) * b.derivative;
  return r;
}

}  // namespace

// ---- Smoothing helpers -------------------------------------------------------

ValueAndDerivative blend_weight(double theta, double width)
{
  const double t = std::tanh(theta / width);
  return {0.5 * (1.0 + t), 0.5 * (1.0 - t * t) / width};
}

ValueAndDerivative smooth_abs(double x, double eps)
{
  const double q = std::sqrt(x * x + eps * eps);
  return {q - eps, x / q};
}

// ---- Joint kinematics ------------------------------------------------------------

ValueAndDerivative contact_radius(const Link& k, double theta, const SolverSettings& settings)
{
  // (C4) blended: R_l for theta >> 0, R_r for theta << 0, smooth in between.
  const ValueAndDerivative w = blend_weight(theta, settings.blend_width);
  return {w.value * k.R_l + (1.0 - w.value) * k.R_r, w.derivative * (k.R_l - k.R_r)};
}

// ---- Rod free length and elastic moment ---------------------------------------

GapHeights gap_heights_exact(const Link& k, double theta)
{
  return theta >= 0.0 ? gap_heights_left_branch(k, theta) : gap_heights_right_branch(k, theta);
}

GapHeights gap_heights(const Link& k, double theta, const SolverSettings& settings)
{
  // Evaluate both branches at the same theta and mix them. Far from 0 the
  // weight is 0 or 1 and this equals gap_heights_exact().
  const ValueAndDerivative w = blend_weight(theta, settings.blend_width);
  const GapHeights left = gap_heights_left_branch(k, theta);
  const GapHeights right = gap_heights_right_branch(k, theta);

  GapHeights g;
  g.closing = blend(w, left.closing, right.closing);
  g.opening = blend(w, left.opening, right.opening);
  return g;
}

ValueAndDerivative arc_factor(double theta)
{
  // rho = theta / sin(theta): turns a vertical gap height into the length of
  // a rod bent into a circular arc through theta. Series near 0 avoids 0/0.
  if (std::abs(theta) < 1.0e-4)
  {
    return {1.0 + theta * theta / 6.0, theta / 3.0};
  }
  const double s = std::sin(theta);
  const double c = std::cos(theta);
  return {theta / s, (s - theta * c) / (s * s)};
}

ValueAndDerivative free_length(const ValueAndDerivative& h, double theta, double lambda0)
{
  // (C17): s = h rho + 2 lambda0, product rule for s'.
  const ValueAndDerivative rho = arc_factor(theta);
  return {h.value * rho.value + 2.0 * lambda0, h.derivative * rho.value + h.value * rho.derivative};
}

ValueAndDerivative joint_stop_moment(const Link& k, double theta, const SolverSettings& settings)
{
  if (theta > k.alpha)
  {
    return {settings.stop_stiffness * (theta - k.alpha), settings.stop_stiffness};
  }
  if (theta < -k.beta)
  {
    return {settings.stop_stiffness * (theta + k.beta), settings.stop_stiffness};
  }
  return {0.0, 0.0};
}

ValueAndDerivative elastic_moment(const Link& k, double theta, const RobotParams& params,
                                  const SolverSettings& settings)
{
  const double EI = params.E * second_moment_of_area(params.r);

  const GapHeights gaps = gap_heights(k, theta, settings);
  const ValueAndDerivative s_c = free_length(gaps.closing, theta, params.lambda0);
  const ValueAndDerivative s_o = free_length(gaps.opening, theta, params.lambda0);

  const double compliance = 1.0 / s_c.value + 1.0 / s_o.value;  // 1/s_c + 1/s_o
  const double d_compliance = -s_c.derivative / (s_c.value * s_c.value) - s_o.derivative / (s_o.value * s_o.value);

  // (C17) and (C19): M = EI theta (1/s_c + 1/s_o), M' = EI [(1/s_c + 1/s_o) + theta d(1/s_c + 1/s_o)/dtheta]
  ValueAndDerivative M{EI * theta * compliance, EI * (compliance + theta * d_compliance)};

  const ValueAndDerivative stop = joint_stop_moment(k, theta, settings);
  M.value += stop.value;
  M.derivative += stop.derivative;
  return M;
}

double rod_strain(const Link& k, double theta, const RobotParams& params,
                  const SolverSettings& settings)
{
  const GapHeights gaps = gap_heights(k, theta, settings);
  const double s_c = free_length(gaps.closing, theta, params.lambda0).value;
  const double s_o = free_length(gaps.opening, theta, params.lambda0).value;
  return params.r * std::abs(theta) / std::min(s_c, s_o);
}

}  // namespace lopsy_kinematics