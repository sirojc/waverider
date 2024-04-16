#ifndef WAVERIDER_PLANNER_WAVERIDER_PLANNER_H_
#define WAVERIDER_PLANNER_WAVERIDER_PLANNER_H_

#include <glog/logging.h>
#include <ros/ros.h>

// action
#include <actionlib/server/simple_action_server.h>
#include <waverider_chomp_msgs/PlanToGoalWaveriderAction.h>

// msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <navigation_msgs/PoseStamped.h>


// policies
#include <rmpcpp/policies/simple_target_policy.h>
#include <waverider/waverider_policy.h>

// wavemap
#include <wavemap_ros_conversions/map_msg_conversions.h>
#include <wavemap/data_structure/volumetric/hashed_blocks.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap_msgs/Map.h>

// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace waverider_planner {

using PlanningActionServer = actionlib::SimpleActionServer<waverider_chomp_msgs::PlanToGoalWaveriderAction>;
using PlanningFeedbackPtr = waverider_chomp_msgs::PlanToGoalWaveriderFeedbackPtr;

class Planner {
public:
  Planner(ros::NodeHandle nh, ros::NodeHandle nh_private, double delta_t);

  void callbackMap(const wavemap_msgs::Map::ConstPtr& msg);

  bool updateCurrentState();
  bool updateObstacles();

  bool isPositionTargetReached();
  bool isYawTargetReached();

  void publishTargetTwistCommandApproach();
  void publishTargetTwistCommandFinalRotation();
  Eigen::Vector2d getLinearTargetAcceleration();
  Eigen::Vector2d getLinearTargetVelocity(const Eigen::Vector2d& accelerations);
  double getTargetYawVelocity(double des_heading, double curr_heading);

  void run();

  void processActionServerGoals(ros::AsyncSpinner& spinner);
  void goalCB();
  void preemptCB();

private:
  // action stuff
  using PlanningActionServer = actionlib::SimpleActionServer<waverider_chomp_msgs::PlanToGoalWaveriderAction>;
  using Goal = waverider_chomp_msgs::PlanToGoalWaveriderResult;
  PlanningActionServer get_path_action_srv_;

  // ROS members
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher pub_twist_commands_;
  ros::Subscriber sub_wavemap_;

  // policies
  rmpcpp::SimpleTargetPolicy<rmpcpp::Space<3>> target_policy_;
  waverider::WaveriderPolicy waverider_policy_;

  // wavemap
  wavemap::VolumetricDataStructureBase::Ptr occupancy_map_;

  // tolerances
  double tol_position_;
  double tol_rotation_;

  // other stuff
  double delta_t_;
  rmpcpp::State<3> curr_state_;
  Eigen::Vector2d des_position_;
  double des_yaw_;
  double max_linear_vel_;
  double max_angular_vel_;
  bool planning_;
  Eigen::Vector2d prev_acc_;
};

}  //  waverider_planner

#endif  // WAVERIDER_PLANNER_WAVERIDER_PLANNER_H_
