#ifndef WAVERIDER_WAVERIDER_POLICY_2D_H_
#define WAVERIDER_WAVERIDER_POLICY_2D_H_

#include <rmpcpp/core/policy_base.h>
#include <rmpcpp/core/state.h>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>

#include "waverider/parallelized_policy_2d.h"

namespace waverider {
class WaveriderPolicy2D : public rmpcpp::PolicyBase<rmpcpp::Space<2>> {
 public:
  WaveriderPolicy2D() = default;

  void setOccupancyThreshold(FloatingPoint value) {
    obstacle_filter_.setOccupancyThreshold(value);
  }
  void setRunAllLevels(bool run_all_levels) {
    run_all_levels_ = run_all_levels;
  }
  const ObstacleCells& getObstacleCells() {
    return obstacle_filter_.getObstacleCells();
  }

  void updateObstacles(const wavemap::HashedWaveletOctree& map,
                       const Point3D& robot_position);
  void updateTuning(PolicyTuning tuning) { policy_tuning_ = tuning; }

  bool isReady() const { return obstacle_filter_.isReady(); }

  rmpcpp::PolicyValue<2> evaluateAt(const rmpcpp::State<2>& x) override {
    throw std::runtime_error("Not implemented");
  }

  rmpcpp::PolicyValue<2> evaluateAt(const rmpcpp::State<2>& x, const double robot_height, const double radius) override;

 public:
  WavemapObstacleFilter obstacle_filter_;
  bool run_all_levels_ = true;
  PolicyTuning policy_tuning_;
};
}  // namespace waverider

#endif  // WAVERIDER_WAVERIDER_POLICY_2D_H_
