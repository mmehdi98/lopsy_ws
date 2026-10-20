#pragma once

// Per-link geometry in the common link frame (theory §2.2, §3.2, §3.3).
//
// Every link uses the same equations; odd and even links differ only in the
// numbers stored here. Suffix "_l" is the ACTUATED side and "_r" the other
// side, exactly as in the theory doc. For left-rod actuation that is the
// literal left and right.

#include <vector>

#include "lopsy_kinematics/params.hpp"

namespace lopsy_kinematics
{

/// Geometry of one link, computed once from RobotParams.
struct Link
{
  double A_l = 0.0;    ///< hole-to-crest-line distance on the actuated side [m]
  double A_r = 0.0;    ///< ... on the other side [m]
  double R_l = 0.0;    ///< bottom-arc radius on the actuated side [m]
  double R_r = 0.0;    ///< ... on the other side [m]
  double alpha = 0.0;  ///< angle of the actuated-side hole exit on its arc, asin(A_l / R_l)  (C1) [rad]
  double beta = 0.0;   ///< angle of the other hole exit, asin(A_r / R_r)  (C1) [rad]
  double L_tl = 0.0;   ///< hole length on the actuated side, L - R_l (1 - cos alpha)  (C2) [m]
  double L_tr = 0.0;   ///< hole length on the other side,   L - R_r (1 - cos beta)   (C2) [m]
  double Gamma = 0.0;  ///< x of the link centre relative to the crest, (A_l - A_r) / 2  (C3) [m]
  double sigma = 0.0;  ///< mirror sign: +1 odd, -1 even (left actuation)  (§3.2)
};

/// Builds the n links, base to tip (index k = 0..n-1 is link i = k+1).
///
///   odd link  i (even k): A_l = A1, A_r = A2, R_l = R1, R_r = R2, sigma = +1
///   even link i (odd k):  A_l = A2, A_r = A1, R_l = R2, R_r = R1, sigma = -1
std::vector<Link> build_links(const RobotParams& params);

}  // namespace lopsy_kinematics
