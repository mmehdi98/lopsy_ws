#pragma once

// Physical parameters of the robot and numerical settings of the solver.
//
// Units: SI everywhere inside the library (m, N, rad, Pa). Convert to mm or
// degrees only for printing and plotting.

namespace lopsy_kinematics
{

/// pi without relying on the non-standard M_PI macro (std::numbers::pi is C++20).
constexpr double kPi = 3.14159265358979323846;

/// Degrees to radians.
constexpr double deg2rad(double deg) { return deg * kPi / 180.0; }

/// Which rod the motor pulls. All equations are written for Side::Left;
/// Side::Right is the mirror image (theory §5.9) and is handled once, when
/// the per-link geometry is built (see link_geometry.hpp).
enum class Side
{
  Left,
  Right
};

/// Physical parameters of the 1-DOF robot.
///
/// The default member values are the RECOMMENDED SET of the identification
/// (theory §10.5): joint-cut statics (C16) with constant setback (C17),
/// A2 pinned at its nominal 1.0 mm. 1.92 % error (published metric).
struct RobotParams
{
  // ---- Geometry (paper Table II; fitted values where identified) ----------
  int n = 15;            ///< number of links (= number of joints)
  double L = 15.0e-3;    ///< link length, crest to flat top [m]
  double D = 6.2e-3;     ///< link diameter [m] (only used for drawing)
  double A1 = 3.21e-3;   ///< larger hole-to-crest-line distance [m]   (fit, ±0.17 mm)
  double A2 = 1.0e-3;    ///< smaller hole-to-crest-line distance [m]  (PINNED: measure it on a part)
  double R1 = 6.45e-3;   ///< larger bottom-arc radius [m]              (fit, ±0.19 mm)
  double R2 = 2.13e-3;   ///< smaller bottom-arc radius [m]             (fit, ±0.19 mm)

  // ---- Rods (superelastic NiTi) -------------------------------------------
  double r = 0.3e-3;       ///< rod radius [m] (thesis code). E was identified with this
                           ///< value: if r changes, E must scale so that E*r^4 stays the same.
  double E = 61.0e9;       ///< equivalent bending modulus [Pa]           (fit, ±18 GPa)
  double mu = 0.66;        ///< effective rod-link friction for the |theta| wrap (fit, ±0.03)
  double lambda0 = 2.2e-3; ///< setback: depth a rod is clamped inside each hole (C17) [m] (fit, ±0.7 mm)

  // ---- Gravity (C14) ---------------------------------------------------------
  // The planar test has gravity normal to the bending plane, so G = 0.
  // The real link weight is 5.685e-3 N (0.5797 g, thesis App. D).
  // NOTE: gravity is not yet derived for the joint balance (C16). It only
  // enters the contact forces (C11)/(C13) for now.
  double G = 0.0;          ///< weight of one link acting in the bending plane [N]
  double x_cg = 0.02e-3;   ///< CG offset from the link centre, toward the crest side [m]
  double y_cg = 7.35e-3;   ///< CG depth below the flat top [m]
  double psi0 = 0.0;       ///< base axis angle from the upward vertical [rad] (0 upright, pi hanging)
};

/// Numerical settings. Kept apart from RobotParams because they are not
/// properties of the robot.
struct SolverSettings
{
  /// Half-width of the tanh blend between the two arcs at theta = 0
  /// (theory §5.2, v2 update): R_l and R_r, and the two branches of (C9),
  /// are blended over about ±0.2 deg.
  double blend_width = deg2rad(0.2);

  /// Smoothing scale for |theta| in the capstan law (C6)/(C7) [rad] (§10.6).
  double abs_smoothing = 1.0e-4;

  /// Bracket for the joint root: (-beta - margin, alpha + margin) [rad] (§6).
  double bracket_margin = 0.2;

  /// Stiffness of the one-sided joint stop past alpha / -beta [N m / rad].
  /// A numerical guard only (§6): it never engaged in any fit. About 60x the
  /// joint stiffness at the recommended parameters (0.16 N m / rad).
  double stop_stiffness = 10.0;

  /// Convergence tolerance on the joint angle [rad].
  double angle_tolerance = 1.0e-12;

  /// Iteration cap per joint (Newton needs 3-4, bisection about 45-60).
  int max_iterations = 100;
};

/// Second moment of area of one round rod, I = pi r^4 / 4  (C3).
inline double second_moment_of_area(double r)
{
  return kPi * r * r * r * r / 4.0;
}

inline RobotParams default_params(){
  return RobotParams{};
}

}  // namespace lopsy_kinematics
