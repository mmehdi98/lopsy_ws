#include "lopsy_kinematics/model.hpp"

namespace lopsy_kinematics
{

// Wiring only: this file connects the layers and computes nothing itself.
//
// Members are initialised in the order they are DECLARED in the class
// (params_, settings_, links_), not the order written here. links_ is
// declared last, so params_ already holds its value when build_links() reads it.
ContinuumRobotModel::ContinuumRobotModel(const RobotParams& params, const SolverSettings& settings)
  : params_(params), settings_(settings), links_(build_links(params_))
{
}

State ContinuumRobotModel::solve(const RodTensions& F0) const
{
  // Qualified name on purpose: inside the class, a plain solve(...) would
  // refer to this member function, not the free function in statics.hpp.
  return lopsy_kinematics::solve(links_, F0, params_, settings_);
}

}  // namespace lopsy_kinematics
