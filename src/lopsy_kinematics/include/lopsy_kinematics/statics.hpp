#pragma once

// Statics with the joint balance (C16), written in the physical frame with
// both rods: one scalar equation per joint, solved from the base to the tip.
//
//   g_i(theta_i) = M_el_i(theta_i)
//                - T^L_{i-1} (A_l_i - R_i(theta_i) sin theta_i)     left rod: bends left  (C16)
//                + T^R_{i-1} (A_r_i + R_i(theta_i) sin theta_i)     right rod: bends right (§10.2 term)
//
// The right-rod term is (C16) mirrored back into the physical frame; it is
// the same expression as the passive-rod term of theory §10.2.
// Pulling the left rod: T^R = 0, theta > 0. Pulling the right rod: T^L = 0,
// theta < 0. No relabelling of the geometry anywhere.
//
// Joint i depends on the joints below it only through T_{i-1}, and
// dg_i/dtheta_i = M_el' + (T^L + T^R)(R' sin theta + R cos theta) > 0, so each
// joint has exactly one root.
//
// Friction: the capstan law (C6) is applied to each rod with the |theta| wrap.
// For the usual case of one pulled rod and a slack one (assumption 6) this is
// exactly the theory. With BOTH rods tensioned it is an assumption: the
// theory has no friction model for a passive rod (open item 15).

#include <vector>

#include "lopsy_kinematics/link_geometry.hpp"
#include "lopsy_kinematics/mechanics.hpp"
#include "lopsy_kinematics/params.hpp"

namespace lopsy_kinematics
{

/// Tension in the two rods at one place along the chain [N]. Rods cannot
/// push, so both must be >= 0.
struct RodTensions
{
  double left = 0.0;
  double right = 0.0;
};

/// Result of one static solve.
/// Vectors are indexed from the base: element k belongs to link/joint i = k+1.
struct State
{
  RodTensions F0;                     ///< rod tensions applied at the base [N]
  std::vector<double> theta;          ///< theta_1..theta_n [rad], size n; > 0 = bent left
  std::vector<RodTensions> tension;   ///< T_0..T_n, size n+1; tension[0] = F0, tension[i] = T_i above link i
  bool converged = false;             ///< every joint converged within max_iterations
};
// Contact forces are not part of the State: the angles do not need them.
// Compute them on demand with contact_forces(links, state.theta, state.tension, params).

/// Residual g of the joint balance (top of this file) and its slope dg/dtheta
/// (C20 diagonal). T_below holds T_{i-1}, the tensions entering joint i from below.
ValueAndDerivative joint_residual(const Link& k, double theta, const RodTensions& T_below,
                                  const RobotParams& params, const SolverSettings& settings);

/// Outcome of the scalar root search at one joint.
struct JointSolution
{
  double theta = 0.0;
  int iterations = 0;
  bool converged = false;
};

/// Root of joint_residual() in the bracket (-beta - margin, alpha + margin).
/// Step 1: plain bisection (robust, about 45-60 iterations).
/// Step 2: bracketed Newton (§6): start from the linearisation at theta = 0,
///         shrink the bracket with the sign of g, and fall back to a
///         bisection step whenever Newton would leave the bracket (3-4 iterations).
JointSolution solve_joint(const Link& k, const RodTensions& T_below, const RobotParams& params,
                          const SolverSettings& settings);

/// Capstan law across one joint (C6), applied to each rod:
///   T_i = T_{i-1} exp(-mu |theta_i|), with |.| from smooth_abs().
RodTensions tension_after(const RodTensions& T_below, double theta, const RobotParams& params,
                          const SolverSettings& settings);

/// Contact forces F^R_1..F^R_n [N] from the axial balances, once the angles
/// are known, going from the tip down. Optional post-processing: solve() does
/// not call it. With both rods, the tensions and frictions add:
///   last link (C13):   F^R_n cos theta_n = (T^L_{n-1} + T^R_{n-1}) + G cos psi_n
///   links i < n (C11): F^R_i cos theta_i = F^R_{i+1} + f_i + G cos psi_i,
///                      f_i = (T^L_{i-1} - T^L_i) + (T^R_{i-1} - T^R_i)   (C7)
///   psi_i = psi0 + sum_{j<=i} theta_j  (C14)
/// (C11) omits the tangential contact force, so these F^R pick up a factor
/// 1/cos(theta) per joint toward the base (§10.1). Treat them as approximate;
/// the angles from the joint balance do not depend on them.
std::vector<double> contact_forces(const std::vector<Link>& links, const std::vector<double>& theta,
                                   const std::vector<RodTensions>& tension, const RobotParams& params);

/// Full static solve: joint by joint from the base, carrying both tensions
/// upward. Negative tensions are treated as 0 (rods cannot push); both 0
/// gives the straight robot.
State solve(const std::vector<Link>& links, const RodTensions& F0, const RobotParams& params,
            const SolverSettings& settings);

}  // namespace lopsy_kinematics