#pragma once

// The one class the ROS node (or any other user) talks to.
//
// It owns the parameters, the numerical settings and the per-link geometry
// derived from them. Because links_ is built in the constructor from
// params_, the two can never disagree: that consistency is the class
// invariant, and it is why there is no setter for the parameters.
// To change a parameter, build a new model.

#include <vector>

#include "lopsy_kinematics/link_geometry.hpp"
#include "lopsy_kinematics/params.hpp"
#include "lopsy_kinematics/statics.hpp"

namespace lopsy_kinematics
{

class ContinuumRobotModel
{
public:
  /// Builds the links with build_links(params).
  explicit ContinuumRobotModel(const RobotParams& params = RobotParams{},
                               const SolverSettings& settings = SolverSettings{});

  /// Static equilibrium for the given rod tensions [N]: calls solve().
  /// Pull the left rod with {F, 0.0}, the right rod with {0.0, F}.
  State solve(const RodTensions& F0) const;

  const RobotParams& params() const { return params_; }
  const SolverSettings& settings() const { return settings_; }
  const std::vector<Link>& links() const { return links_; }

private:
  RobotParams params_;
  SolverSettings settings_;
  std::vector<Link> links_;  // declared last: built from the members above
};

}  // namespace lopsy_kinematics
