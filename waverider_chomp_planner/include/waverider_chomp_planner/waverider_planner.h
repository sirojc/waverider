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
#include <geometry_msgs/Vector3Stamped.h>
#include <navigation_msgs/PoseStamped.h>

// policies
#include <rmpcpp/policies/simple_target_policy.h>
#include <waverider/waverider_policy_2d.h>

// wavemap
#include <wavemap_ros_conversions/map_msg_conversions.h>
#include <wavemap/data_structure/volumetric/hashed_blocks.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>
#include <wavemap_io/file_conversions.h>

// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>

#include <visualization_msgs/MarkerArray.h>
#include <waverider_ros/policy_visuals.h>

namespace waverider_planner {

using PlanningActionServer = actionlib::SimpleActionServer<waverider_chomp_msgs::PlanToGoalWaveriderAction>;
using PlanningFeedbackPtr = waverider_chomp_msgs::PlanToGoalWaveriderFeedbackPtr;

class Planner {
public:
  Planner(ros::NodeHandle nh, ros::NodeHandle nh_private, double delta_t, std::string planner_frame,
          int n_past_elements_pos, int n_past_elements_vel, int n_past_elements_acc,
          bool limit_acc_change, bool load_map_from_file);
  ~Planner();

  void callbackMap(const wavemap_msgs::Map::ConstPtr& msg);
  void callbackTwistMux(const geometry_msgs::TwistStamped& msg);

  bool updateCurrentState();
  bool updateObstacles();

  bool isPositionTargetReached();
  bool isYawTargetReached();

  geometry_msgs::TwistStamped transform_twist(const geometry_msgs::TwistStamped& initial_twist, const std::string& target_frame,
                                              const geometry_msgs::TransformStamped& transformStamped);

  double getMedian(const std::deque<double>& data);

  void TargetTwistCommandApproach();
  void TargetTwistCommandFinalRotation();
  void publishTargetTwist(const geometry_msgs::TwistStamped target_twist);
  Eigen::Vector2d getLinearTargetAcceleration();
  Eigen::Vector2d getLinearTargetVelocity(const Eigen::Vector2d& des_accelerations);
  double getTargetYawVelocity(double des_heading, double curr_heading);

  void run();

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

  // debugging/ visualization
  ros::Publisher pub_waverider_obstacles_;
  ros::Publisher pub_des_pos_;
  ros::Publisher pub_des_vel_;
  ros::Publisher pub_des_acc_target_;
  ros::Publisher pub_des_acc_waverider_;
  ros::Publisher pub_des_acc_final_;
  ros::Publisher pub_des_est_yaw_;
  ros::Publisher pub_estimated_pos_;
  ros::Publisher pub_estimated_vel_;
  ros::Publisher pub_estimated_acc_;
  ros::Publisher pub_occupancy_;
  ros::Subscriber sub_twist_mux_;

  // policies
  rmpcpp::SimpleTargetPolicy<rmpcpp::Space<2>> target_policy_;
  waverider::WaveriderPolicy2D waverider_policy_;

  // wavemap
  wavemap::VolumetricDataStructureBase::Ptr occupancy_map_;

  // tolerances
  double tol_position_;
  double tol_rotation_;

  // other stuff
  std::string planner_frame_;

  bool limit_acc_change_;

  double delta_t_;
  double curr_height_;
  double curr_yaw_;

  Eigen::Vector2d des_position_;
  double des_yaw_;
  double max_linear_vel_;
  double max_linear_acc_;
  double max_angular_vel_;
  bool planning_;
  bool reached_pos_;

  rmpcpp::State<2> curr_state_;

  int n_past_elements_pos_;
  std::deque<double> past_pos_x_;
  std::deque<double> past_pos_y_;
  Eigen::Vector2d prev_pos_;

  int n_past_elements_vel_;
  std::deque<double> past_vel_x_;
  std::deque<double> past_vel_y_;
  Eigen::Vector2d prev_vel_;

  int n_past_elements_acc_;
  std::deque<double> past_acc_x_;
  std::deque<double> past_acc_y_;
  Eigen::Vector2d prev_acc_;

  double last_time_stamp_;

  bool load_map_from_file_;
  double k_vel_ctrl_;

  geometry_msgs::TwistStamped twist_mux_twist_; // for visualization purposes
};

}  //  waverider_planner

#endif  // WAVERIDER_PLANNER_WAVERIDER_PLANNER_H_
