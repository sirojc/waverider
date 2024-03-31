#ifndef CHOMP_ROS_CHOMP_PLANNER_H_
#define CHOMP_ROS_CHOMP_PLANNER_H_

#include <glog/logging.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <wavemap/data_structure/volumetric/hashed_blocks.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap/indexing/index_conversions.h>
#include <wavemap/utils/esdf/collision_utils.h>
#include <wavemap/utils/esdf/esdf_generator.h>
#include <wavemap/utils/interpolation_utils.h>
#include <wavemap_io/file_conversions.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <waverider_chomp_msgs/GetTraj.h>
#include "chomp_ros/chomp_optimizer.h"

namespace chomp {

class ChompPlanner {
public:
  ChompPlanner(ros::NodeHandle nh, ros::NodeHandle nh_private, double height_robot);

  Eigen::MatrixXd get_full_traj(const Eigen::MatrixXd chomp_traj,
                                const geometry_msgs::Pose start,
                                const geometry_msgs::Pose goal) const;

  bool checkTrajCollision(const Eigen::MatrixXd& trajectory) const;

  void updateMap(const wavemap::VolumetricDataStructureBase::Ptr map);

  void visualizeTrajectory(const Eigen::MatrixXd& trajectory) const;

  bool getTrajectoryService(waverider_chomp_msgs::GetTraj::Request& req,
                             waverider_chomp_msgs::GetTraj::Response& res);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ros subscriber/ publisher
  ros::Publisher occupancy_pub_;
  ros::Publisher esdf_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher trajectory_pub_arrows_;

  //wavemap
  wavemap::VolumetricDataStructureBase::Ptr occupancy_map_;
  wavemap::HashedWaveletOctree::Ptr hashed_map_;
  const float kOccupancyThreshold_ = -0.1f;
  const float kMaxDistance_ = 2.f;
  wavemap::HashedBlocks::Ptr esdf_;
  const float kRobotRadius_ = 0.5f;
  const float kSafetyPadding_ = 0.1f;
  const float paddedRobotRadius_ = kRobotRadius_ + kSafetyPadding_;
  std::function<float(const Eigen::Vector2d&)> distance_getter_;

  // robot
  const double height_robot_;
  const double des_lin_velocity_ = 0.1; // TODO: CHECK WHAT THIS SHOULD BE
  const double des_ang_velocity_ = 0.2;

  // chomp
  chomp::ChompOptimizer chomp_;
  chomp::ChompParameters params_;
};

}  //  namespace chomp

#endif  // CHOMP_ROS_CHOMP_PLANNER_H_

