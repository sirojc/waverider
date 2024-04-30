#include "waverider/waverider_policy_2d.h"
#include "waverider/obstacle_filter.h"
#include <tracy/Tracy.hpp>

#include <iostream>

namespace waverider {
void WaveriderPolicy2D::updateObstacles(const wavemap::HashedWaveletOctree& map,
                                      const Point3D& robot_position) {
  ZoneScoped;
  obstacle_filter_.update(map, robot_position);
}

rmpcpp::PolicyValue<2> WaveriderPolicy2D::evaluateAt(const rmpcpp::State<2>& x, const double robot_height, const double radius) {
  ZoneScoped;
  if (!isReady()) {
    return {Eigen::Vector2d::Zero(), Eigen::Matrix2d::Zero()};
  }

  const Eigen::Vector2f x_pos = x.pos_.cast<float>();
  const Eigen::Vector2f x_vel = x.vel_.cast<float>();

  // get all cells where we should attach a policy
  const auto& policy_cells = obstacle_filter_.getObstacleCells();

  std::vector<rmpcpp::PolicyValue<2>> all_policies;
  for (size_t i = 0; i < policy_cells.cell_widths.size(); i++) {
    if (i == 0 || run_all_levels_) {
        //std::cout << "N"<< i << policy_cells.centers[i].size() << std::endl;

      // only use obstacle cells that are within [robot_height - radius, robot_height + radius]
      std::vector<Eigen::Vector3f> considered_centers;
      for (const auto& center : policy_cells.centers[i]) {
        if (center.z() >= robot_height - radius && center.z() <= robot_height + radius) {
          considered_centers.push_back(center);
        }
      }
      
      // std::cout << "---- WaveriderPolicy2D: n considered obstacles = " << considered_centers.size() << std::endl;

      ParallelizedPolicy2D pol_generator(considered_centers.size(),
                                         policy_tuning_);
      // pol_generator.setR(WavemapObstacleFilter::maxRangeForHeight(i)*1.5);

      pol_generator.init(considered_centers, x_pos, x_vel);
      all_policies.emplace_back(pol_generator.getPolicy());
    }
  }

  const auto avoidance_policy = rmpcpp::PolicyValue<2>::sum(all_policies);

  rmpcpp::PolicyValue<2> scaled_avoidance = {avoidance_policy.f_,
                                             avoidance_policy.A_};
  //  std::cout << scaled_avoidance.f_.transpose() << std::endl;
  //  std::cout << scaled_avoidance.A_ << std::endl;
  return scaled_avoidance;
}
}  // namespace waverider
