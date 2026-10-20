#include "lopsy_kinematics/joint_state_mapping.hpp"

#include <algorithm>

namespace lopsy_kinematics
{

std::vector<std::string> urdf_joint_names(std::size_t n)
{
  std::vector<std::string> names;
  names.reserve(2*n);
  
  for (std::size_t i=1; i <= n; i++){
    names.push_back("j" + std::to_string(i) + "_roll_l");
    names.push_back("j" + std::to_string(i) + "_roll_r");
  }
  return names;
}

std::vector<double> urdf_joint_positions(const std::vector<double>& theta)
{
  std::vector<double> positions;
  positions.reserve(2*theta.size());

  for (auto theta_i:theta){
    positions.push_back(std::max(theta_i,0.0));
    positions.push_back(std::min(theta_i,0.0));
  }
  return positions;
}

}  // namespace lopsy_kinematics
