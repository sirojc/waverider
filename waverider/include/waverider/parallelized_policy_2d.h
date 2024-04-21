#ifndef WAVERIDER_PARALLELIZED_POLICY_2D_H_
#define WAVERIDER_PARALLELIZED_POLICY_2D_H_

#include <vector>

#include <rmpcpp/core/policy_value.h>

#include "waverider/obstacle_filter.h"
#include "waverider/policy_tuning.h"

namespace waverider {
class ParallelizedPolicy2D {
 public:
  ParallelizedPolicy2D(uint num_policies, PolicyTuning tuning);

  void init(const std::vector<Eigen::Vector3f>& x_obs, const Eigen::Vector2f& x,
            const Eigen::Vector2f& xdot);

  rmpcpp::PolicyValue<2> getPolicy() {
    return {(A_sum.completeOrthogonalDecomposition().pseudoInverse() * Af_sum)
                .cast<double>(),
            A_sum.cast<double>()};
  }

  inline Eigen::Vector2f s(const Eigen::Vector2f& x) const {
    return x / h(x.norm());
  }

  // Softmax helper function
  inline float h(float z) const {
    return (z + tuning_.c * std::log(1 + std::exp(-2.f * tuning_.c * z)));
  }
  inline static float wr(float s, float r) {
      if(s > r){
          return 0;
      }
    const float c2 = 1 / (r * r);
    const float c1 = -2 / r;
    return (static_cast<float>(c2) * s * s) + (c1 * s) + 1.f;
  }

  void setR(float r) {tuning_.r = r;
  tuning_.nu_rep = 0.5 * r;
  tuning_.nu_damp = 0.3 * r;
  }

 private:
  PolicyTuning tuning_;

  uint num_policies_;
  Eigen::Vector2f Af_sum = Eigen::Vector2f::Zero();
  Eigen::Matrix2f A_sum = Eigen::Matrix2f::Zero();
};
}  // namespace waverider

#endif  // WAVERIDER_PARALLELIZED_POLICY_2D_H_