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
#include <chomp_msgs/GetTraj.h>
#include "chomp_ros/chomp_optimizer.h"


class ChompPlanner {
public:
  ChompPlanner(ros::NodeHandle nh, ros::NodeHandle nh_private);

  Eigen::MatrixXd get_full_traj(Eigen::MatrixXd chomp_traj, double yaw_goal);

  bool getTrajectoryCallback(chomp_msgs::GetTraj::Request& req,
                             chomp_msgs::GetTraj::Response& res);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ros subscriber/ publisher/ service
  ros::Publisher occupancy_pub_;
  ros::Publisher esdf_pub_;
  ros::Publisher trajectory_pub_;

  //wavemap
  wavemap::VolumetricDataStructureBase::Ptr occupancy_map_;
  wavemap::HashedWaveletOctree::Ptr hashed_map_;
  wavemap::HashedBlocks::Ptr esdf_;
  const float kRobotRadius_ = 0.5f;
  const float kSafetyPadding_ = 0.f;
  const float paddedRobotRadius_ = kRobotRadius_ + kSafetyPadding_;
  std::function<float(const Eigen::Vector2d&)> distance_getter_;

  // traj
  const double des_lin_velocity_ = 0.1; // TODO: CHECK WHAT THIS SHOULD BE
  const double des_ang_velocity_ = 0.2;

  // chomp
  chomp::ChompOptimizer chomp_;
  chomp::ChompParameters params_;
};


