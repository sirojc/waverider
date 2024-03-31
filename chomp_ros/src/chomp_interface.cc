#include "chomp_interface.h"

namespace chomp {

ChompInterface::ChompInterface(ros::NodeHandle nh, ros::NodeHandle nh_private, double height_robot) : 
      nh_(nh), nh_private_(nh_private), planner_(nh, nh_private, height_robot) {
  // ros
  map_sub_ = nh_.subscribe("map", 1, &ChompInterface::callbackMap, this);
  get_traj_client_ = nh_.serviceClient<chomp_msgs::GetTraj>("get_traj");
}

void ChompInterface::callbackMap(const wavemap_msgs::Map::ConstPtr& msg) {
  // convert map message to map
  wavemap::VolumetricDataStructureBase::Ptr map;
  wavemap::convert::rosMsgToMap(*msg, map);
}

void ChompInterface::checkReplanning(const Eigen::MatrixXd& remaining_trajectory) {
  // check if the remaining trajectory is collision free
  bool is_collision_free = checkTrajCollision(remaining_trajectory);

  if (is_collision_free) {
    LOG(INFO) << "Remaining trajectory is collision free.";
    
  } else {
    LOG(INFO) << "Remaining trajectory is NOT collision free. Replanning.";
    // TODO: CALL AGAIN THE CHOMP PLANNER, check in more detail when local traj follower clear
  }

  return;
}

} // namespace chomp

int main(int argc, char** argv) {
  // Initialize logging
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Initialize ROS
  ros::init(argc, argv, "chomp_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Create the ChompPlanner object
  chomp::ChompInterface chomp_interface(nh, nh_private, 0.65);
  ros::ServiceServer get_traj_service = nh.advertiseService("get_traj", &chomp::ChompPlanner::getTrajectoryService, &chomp_planner);

  // Spin ROS
  ros::spin();

  return 0;
}