#ifndef WAVERIDER_ROS_WAVERIDER_EVALUATOR_H_
#define WAVERIDER_ROS_WAVERIDER_EVALUATOR_H_

#include <string>
#include <thread>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <wavemap/config/config_base.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap_ros/tf_transformer.h>
#include <waverider/eval_planner.h>
#include <waverider/waverider_policy.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <rmpcpp/eval/integrator.h>

namespace waverider {
struct WaveriderEvaluatorConfig : wavemap::ConfigBase<WaveriderEvaluatorConfig, 1> {
    std::string hans;


  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};


class WaveriderEvaluator : public waverider::EvalPlanner {
 public:

    WaveriderEvaluator(const WaveriderEvaluatorConfig& config, bool only_highest_res);

  void loadMap(std::string path);
  void setTuning(/*whatever the hell we input here*/);
  EvalPlanner::Result plan(Eigen::Vector3d start, Eigen::Vector3d end);

  void publishState(Eigen::Vector3d pos, Eigen::Vector3d vel);

  inline std::string getName(){
    std::string name = "WAVE";
    if(only_highest_res_){
      name += "OH";
    }
    return name;
  }

 private:
  bool only_highest_res_ = false;
  ros::Publisher debug_pub_;
  ros::Publisher debug_pub_odom_;
  const WaveriderEvaluatorConfig config_;
  ros::Publisher map_pub_;

  wavemap::HashedWaveletOctree::Ptr map_;

  size_t max_integration_steps_{10000};

};
}  // namespace waverider

#endif  // WAVERIDER_ROS_WAVERIDER_EVALUATOR_H_
