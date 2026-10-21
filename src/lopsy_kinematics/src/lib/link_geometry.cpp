#include "lopsy_kinematics/link_geometry.hpp"

#include <cmath>

namespace lopsy_kinematics
{

std::vector<Link> build_links(const RobotParams& params)
{
  std::vector<Link> links(params.n);

  for (std::size_t k = 0; k < links.size(); ++k)
  {
    Link& link = links[k];

    if (k % 2 == 0)
    {
      link.A_l = params.A1;
      link.A_r = params.A2;
      link.R_l = params.R1;
      link.R_r = params.R2;
      link.sigma = +1.0;
    }
    else 
    {
      link.A_l = params.A2;
      link.A_r = params.A1;
      link.R_l = params.R2;
      link.R_r = params.R1;
      link.sigma = -1.0;
    }

    // Derived values: 
    link.alpha = std::asin(link.A_l / link.R_l);                 // (C1)
    link.beta = std::asin(link.A_r / link.R_r);                  // (C1)
    link.L_tl = params.L - link.R_l * (1.0 - std::cos(link.alpha));  // (C2)
    link.L_tr = params.L - link.R_r * (1.0 - std::cos(link.beta));   // (C2)
    link.Gamma = (link.A_l - link.A_r) / 2.0;                    // (C3)
  }

  return links;
}

}  // namespace lopsy_kinematics
