#ifndef WAVERIDER_CHOMP_PLANNER_WAVERIDER_CHOMP_PLANNER_H_
#define WAVERIDER_CHOMP_PLANNER_WAVERIDER_CHOMP_PLANNER_H_

#include <glog/logging.h>
#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <wavemap/data_structure/volumetric/hashed_blocks.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap/indexing/index_conversions.h>
#include <wavemap/utils/esdf/collision_utils.h>
#include <wavemap/utils/esdf/esdf_generator.h>
#include <wavemap/utils/interpolation_utils.h>
#include <wavemap/config/param.h>
#include <wavemap/config/config_base.h>
#include <wavemap_io/file_conversions.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>
#include <wavemap_ros/tf_transformer.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <waverider_chomp_msgs/GetTraj.h>
#include <waverider_ros/policy_visuals.h>
#include <waverider/waverider_policy.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <rmpcpp/eval/integrator.h>
#include "chomp_ros/chomp_optimizer.h"

#include <waverider_chomp_msgs/SetPlannerType.h>

#include <actionlib/server/simple_action_server.h>
#include <waverider_chomp_msgs/PlanToGoalLocalGuidanceAction.h>
#include <navigation_msgs/FollowPathLocalGuidanceAction.h>

// TOOD: IF NO CHANGES ACTIONS - CAN ALSO USE THE ONE FROM NAVIGATION

namespace waverider_chomp_planner {

using PlanningActionServer = actionlib::SimpleActionServer<waverider_chomp_msgs::PlanToGoalLocalGuidanceAction>;
using PlanningFeedbackPtr = waverider_chomp_msgs::PlanToGoalLocalGuidanceFeedbackPtr;

using Feedback = PlanningActionServer::Feedback;
using FeedbackStatus = Feedback::_status_local_planner_type;

class Planner {
public:
  Planner(ros::NodeHandle nh, ros::NodeHandle nh_private, bool load_map_from_file, std::string frame);

  void processActionServerGoals(ros::AsyncSpinner& spinner);
  void goalCB();
  void preemptCB();
  void publishLocalPlannerFeedback(FeedbackStatus feedback);
  
  bool checkTrajCollision(const Eigen::MatrixXd& trajectory) const;


  void getTrajectory(const geometry_msgs::Pose& start,
                     const geometry_msgs::Pose& goal,
                     const waverider_chomp_msgs::PlannerType& planner_type,
                     const bool ignore_orientation,
                     const navigation_msgs::Tolerance tol,
                     const std::string local_guidance_mode);

  Eigen::MatrixXd getChompTrajectory(const geometry_msgs::Pose& start,
                                     const geometry_msgs::Pose& goal);
  Eigen::MatrixXd getWaveriderTrajectory(const geometry_msgs::Pose& start,
                                         const geometry_msgs::Pose& goal);

  Eigen::MatrixXd getFullTraj(const Eigen::MatrixXd chomp_traj,
                              const geometry_msgs::Pose start,
                              const geometry_msgs::Pose goal) const;

  void callbackMap(const wavemap_msgs::Map::ConstPtr& msg);
  void updateMap(bool update_esdf);

  bool setPlannerTypeService(waverider_chomp_msgs::SetPlannerType::Request& req,
                             waverider_chomp_msgs::SetPlannerType::Response& res);

  void visualizeTrajectory(const Eigen::MatrixXd& trajectory) const;
  void visualizeState(const Eigen::Vector3d& pos) const;

private:
  // action stuff
  PlanningActionServer get_path_action_srv_;
  PlanningFeedbackPtr feedback_{new waverider_chomp_msgs::PlanToGoalLocalGuidanceFeedback()};

  // ROS members
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher occupancy_pub_;
  ros::Publisher esdf_pub_;
  ros::Publisher waverider_map_pub_;
  ros::Publisher state_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher trajectory_pub_arrows_;

  ros::Subscriber map_sub_;

  ros::ServiceServer set_planner_type_server_;

  // wavemap
  wavemap::VolumetricDataStructureBase::Ptr occupancy_map_;
  wavemap::HashedWaveletOctree::Ptr hashed_map_;
  const float kOccupancyThreshold_ = -0.1f;
  const float kMaxDistance_ = 2.f;
  wavemap::HashedBlocks::Ptr esdf_;
  const float kRobotRadius_ = 0.5f;
  const float kSafetyPadding_ = 0.1f;
  const float paddedRobotRadius_ = kRobotRadius_ + kSafetyPadding_;
  std::function<float(const Eigen::Vector2d&)> distance_getter_esdf_;
  std::function<float(const Eigen::Vector2d&)> distance_getter_occ_;

  // robot
  const std::string planner_frame_ = "map";
  double height_robot_ = 0.65; // TODO: get from start
  const double des_lin_velocity_ = 0.1; // TODO: CHECK WHAT THIS SHOULD BE
  const double des_ang_velocity_ = 0.2;

  // planner
  bool load_map_from_file_;
  waverider_chomp_msgs::PlannerType planner_type_;

  // chomp
  chomp::ChompOptimizer chomp_;
  chomp::ChompParameters params_;

  // waverider
  // bool flat_res_ = false;
  bool flat_res_ = true;
  // double flat_res_radius_ = 0.0;
  double flat_res_radius_ = 1.0;
  size_t max_integration_steps_{10000};
};

}  //  waverider_chomp_planner

#endif  // WAVERIDER_CHOMP_PLANNER_WAVERIDER_CHOMP_PLANNER_H_
