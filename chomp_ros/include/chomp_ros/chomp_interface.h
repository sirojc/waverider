#ifndef CHOMP_ROS_CHOMP_INTERFACE_H
#define CHOMP_ROS_CHOMP_INTERFACE_H

#include <ros/service_client.h>

namespace chomp {

// TODO: Could move occupancy map here
class ChompInterface {
public:
  ChompInterface(ros::NodeHandle nh, ros::NodeHandle nh_private, double height_robot);

  void checkReplanning(const wavemap::VolumetricDataStructureBase::Ptr map);
  void callbackMap(const wavemap_msgs::Map::ConstPtr& msg);


private:
  // ros
  ros::Subscriber map_sub_;
  ros::ServiceClient get_traj_client_;

  // chomp
  ChomPlanner planner_;
};

} // namespace chomp

#endif // CHOMP_ROS_CHOMP_INTERFACE_H