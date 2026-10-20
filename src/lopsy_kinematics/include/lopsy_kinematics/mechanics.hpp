#pragma once

// Mechanics of ONE joint: pure functions of that joint's angle theta.
// No loops over links here; the statics layer does that.
//
// Every function returns the value and its derivative d/dtheta together.
// While you only use bisection you can leave the derivative at 0; Newton
// (and the analytic Jacobian, §10.6) needs it later.

#include "lopsy_kinematics/link_geometry.hpp"
#include "lopsy_kinematics/params.hpp"

namespace lopsy_kinematics
{

/// A scalar and its derivative with respect to theta.
struct ValueAndDerivative
{
  double value = 0.0;
  double derivative = 0.0;
};

// ---- Smoothing helpers -------------------------------------------------------

/// Weight of the left branch: w = 0.5 * (1 + tanh(theta / width)).
/// w -> 1 for theta >> width (rolling on the left arc R_l), w -> 0 for theta << -width.
ValueAndDerivative blend_weight(double theta, double width);

/// Smooth |x| for the capstan wrap (C6)-(C7), exact for |x| >> eps.
/// Any smooth, even, convex choice works, e.g. sqrt(x^2 + eps^2) - eps.
ValueAndDerivative smooth_abs(double x, double eps);

// ---- Joint kinematics ------------------------------------------------------------

/// Radius of the arc that is rolling, R_i  (C4), blended through theta = 0
/// with blend_weight(): R = w R_l + (1 - w) R_r.
ValueAndDerivative contact_radius(const Link& k, double theta, const SolverSettings& settings);

// ---- Rod free length and elastic moment ---------------------------------------

/// Vertical heights of the closing and opening rod gaps at the joint.
struct GapHeights
{
  ValueAndDerivative closing;  ///< h^c: gap on the side the link tilts toward
  ValueAndDerivative opening;  ///< h^o: gap on the other side
};

/// Exact branch formulas (C9a) for theta >= 0 and (C9b) for theta < 0.
/// Derivatives: §10.6, just below (C19).
GapHeights gap_heights_exact(const Link& k, double theta);

/// (C9) blended across theta = 0: evaluate BOTH branches at theta and mix
/// them with blend_weight() (§5.2, v2 update). Start with the exact version;
/// add this when you want to match the Python to machine precision.
GapHeights gap_heights(const Link& k, double theta, const SolverSettings& settings);

/// rho(theta) = theta / sin(theta)
/// converts gap height into arc length along the gap
/// Near 0 use the series: rho ~ 1 + theta^2/6, rho' ~ theta/3.
ValueAndDerivative arc_factor(double theta);

/// Free length of one rod across the joint  (C17):
///   s = h * rho(theta) + 2 * lambda0,   s' = h' rho + h rho'.
/// lambda0 = 0 gives the gap-only free length (C9) of v1.
/// lambda0 is the length that a rod travels in a hole before it touches the walls. Assumed constant and independent of theta.
ValueAndDerivative free_length(const ValueAndDerivative& h, double theta, double lambda0);

/// One-sided joint stop past alpha (theta > alpha) or -beta (theta < -beta):
/// stop_stiffness * (theta - alpha), resp. stop_stiffness * (theta + beta),
/// zero in between. A numerical guard (§6).
ValueAndDerivative joint_stop_moment(const Link& k, double theta, const SolverSettings& settings);

/// Elastic moment of BOTH rods across the joint, positive opposing +theta:
///   M_el = E I theta (1/s^c + 1/s^o)  (C17)  + joint_stop_moment.
/// Derivative: (C19), plus the stop's slope.
ValueAndDerivative elastic_moment(const Link& k, double theta, const RobotParams& params,
                                  const SolverSettings& settings);

/// Diagnostic: peak fibre strain of the rods, r |theta| / min(s^c, s^o).
/// About 2.2 % at full load with the recommended set (§10.5).
double rod_strain(const Link& k, double theta, const RobotParams& params,
                  const SolverSettings& settings);

}  // namespace lopsy_kinematics
