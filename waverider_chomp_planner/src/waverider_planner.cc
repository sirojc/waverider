#include "waverider_chomp_planner/waverider_planner.h"

#include <cmath>

#include <cassert>
#include <iostream>

namespace waverider_planner {

Planner::Planner(ros::NodeHandle nh, ros::NodeHandle nh_private, double delta_t)
  : nh_(nh), nh_private_(nh_private), get_path_action_srv_(nh, "/waverider_planner/plan_path", false),
  tf_buffer_(), tf_listener_(tf_buffer_), tol_position_(0.1), tol_rotation_(0.1), planning_(false),
  prev_acc_(Eigen::Vector2d::Zero()), delta_t_(delta_t) {
    // action
    get_path_action_srv_.registerGoalCallback(boost::bind(&Planner::goalCB, this));
    get_path_action_srv_.registerPreemptCallback(boost::bind(&Planner::preemptCB, this));
    get_path_action_srv_.start();

    // sub/pub init
    pub_twist_commands_ = nh_.advertise<geometry_msgs::TwistStamped>("/waverider_planner/twist", 10, true);
    sub_wavemap_ = nh_.subscribe("/wavemap/map", 1, &Planner::callbackMap, this);

    // policy initialization
    target_policy_.setTuning(10, 15, 0.01);
    target_policy_.setA(Eigen::Matrix3d::Identity()*10);

    waverider_policy_.setOccupancyThreshold(0.01);
    waverider_policy_.run_all_levels_ = false;
    waverider_policy_.obstacle_filter_.lowest_level_radius_ = 1.0;
    waverider_policy_.obstacle_filter_.use_only_lowest_level_ = true;
  };

bool Planner::isPositionTargetReached() {
  std::cout << "------------------- isPositionTargetReached" << std::endl;
  Eigen::Vector2d distance(des_position_[0] - curr_state_.pos_[0], des_position_[1] - curr_state_.pos_[1]);
  if (distance.norm() < tol_position_) {
    return true;
  } else {
    return false;
  }
}

bool Planner::isYawTargetReached() {
  std::cout << "------------------- isYawTargetReached" << std::endl;
  double curr_yaw = std::fmod(std::atan2(curr_state_.pos_[1], curr_state_.pos_[0]), 2 * M_PI);

  double higher_angle = std::max(curr_yaw, des_yaw_);
  double lower_angle = std::min(curr_yaw, des_yaw_);

  double distance = std::fmod(higher_angle - lower_angle, M_PI);
  if (distance < tol_rotation_) {
    return true;
  } else {
    return false;
  }
}

Eigen::Vector2d Planner::getLinearTargetAcceleration() {
  std::cout << "-------- getLinearTargetAcceleration" << std::endl;
  // evaluate target policy
  rmpcpp::PolicyValue<3> target_result = target_policy_.evaluateAt(curr_state_);

  // evaluate waverider policy
  rmpcpp::PolicyValue<3> waverider_result = waverider_policy_.evaluateAt(curr_state_);

  // get target acceleration
  Eigen::Vector3d acceleration = (target_result + waverider_result).f_;

  return Eigen::Vector2d(acceleration.x(), acceleration.y());
}

Eigen::Vector2d Planner::getLinearTargetVelocity(const Eigen::Vector2d& accelerations) {
  std::cout << "-------- getLinearTargetVelocity" << std::endl;

  // trapezoidal rule
  Eigen::Vector2d target_velocity = (delta_t_ / 2.0) * (accelerations + prev_acc_);

  return target_velocity;
}

double Planner::getTargetYawVelocity(double des_heading, double curr_heading) {
  std::cout << "-------- getTargetYawVelocity" << std::endl;
  double diff_heading = des_heading - curr_heading;
  
  if (diff_heading > M_PI) {
    diff_heading -= 2 * M_PI;
  } else if(diff_heading < M_PI) {
    diff_heading += 2 * M_PI;
  }

  // p controller for jaw twist
  double k = 0.1; // todo: add as member var and tune
  double yaw_velocity = k * diff_heading;

  return yaw_velocity;
}

void Planner::publishTargetTwistCommandApproach() {
  std::cout << "------------------- publishTargetTwistCommandApproach" << std::endl;
  // get desired x,y accelerations
  Eigen::Vector2d accelerations_x_y = getLinearTargetAcceleration();

  // get desired x, y velocities
  Eigen::Vector2d velocities_x_y = getLinearTargetVelocity(accelerations_x_y);

  // get desired yaw velocity
  double des_yaw = std::fmod(std::atan2(velocities_x_y[1], velocities_x_y[0]), 2 * M_PI);
  double curr_yaw = std::fmod(std::atan2(curr_state_.pos_[1], curr_state_.pos_[0]), 2 * M_PI);
  double yaw_velocity = getTargetYawVelocity(des_yaw, curr_yaw);


  // ensure vel within range
  velocities_x_y[0] = std::min(std::max(velocities_x_y[0], -max_linear_vel_), max_linear_vel_);
  velocities_x_y[1] = std::min(std::max(velocities_x_y[1], -max_linear_vel_), max_linear_vel_);
  yaw_velocity = std::min(std::max(yaw_velocity, -max_angular_vel_), max_angular_vel_);

  // get entire twist command
  geometry_msgs::TwistStamped twist_command;
  twist_command.header.stamp = ros::Time::now();
  twist_command.header.frame_id = "map"; // todo: probably not in this frame --> make variable and check what it should be
  twist_command.twist.linear.x = velocities_x_y[0];
  twist_command.twist.linear.y = velocities_x_y[1];
  twist_command.twist.linear.z = 0.0;
  twist_command.twist.angular.x = 0.0; // roll
  twist_command.twist.angular.y = 0.0; // pitch
  twist_command.twist.angular.z = yaw_velocity; // yaw

  pub_twist_commands_.publish(twist_command);
}

void Planner::publishTargetTwistCommandFinalRotation() {
  std::cout << "------------------- publishTargetTwistCommandFinalRotation" << std::endl;
  // get yaw velocity
  double curr_yaw = std::atan2(curr_state_.pos_[1], curr_state_.pos_[0]);
  double yaw_velocity = getTargetYawVelocity(des_yaw_, curr_yaw);

  // ensure vel within range
  yaw_velocity = std::min(std::max(yaw_velocity, -max_angular_vel_), max_angular_vel_);

  // get entire twist command
  geometry_msgs::TwistStamped twist_command;
  twist_command.header.stamp = ros::Time::now();
  twist_command.header.frame_id = "map"; // todo: probably not in this frame --> make variable and check what it should be
  twist_command.twist.linear.x = 0.0;
  twist_command.twist.linear.y = 0.0;
  twist_command.twist.linear.z = 0.0;
  twist_command.twist.angular.x = 0.0; // roll
  twist_command.twist.angular.y = 0.0; // pitch
  twist_command.twist.angular.z = yaw_velocity; // yaw

  pub_twist_commands_.publish(twist_command);
}

void Planner::callbackMap(const wavemap_msgs::Map::ConstPtr& msg) {
  ROS_INFO_ONCE("Received first map message.");

  // convert map message to map
  wavemap::VolumetricDataStructureBase::Ptr map;
  wavemap::convert::rosMsgToMap(*msg, occupancy_map_);
}

bool Planner::updateObstacles() {
  std::cout << "------------------------- updateObstacles" << std::endl;

  // check if occupancy_map_ is set
  if (!occupancy_map_) {
    ROS_ERROR("Occupancy map not initialized yet.");
    planning_ = false;

    Goal result;
    result.success = false;
    get_path_action_srv_.setAborted(result);

    return false;
  }

  // update hashed map
  auto hashed_map_ = std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(occupancy_map_);
  if (!hashed_map_) {
    ROS_ERROR("Failed to cast occupancy map to HashedWaveletOctree.");
    planning_ = false;

    Goal result;
    result.success = false;
    get_path_action_srv_.setAborted(result);

    return false;
  }

  waverider_policy_.updateObstacles(*hashed_map_, curr_state_.pos_.cast<float>()); 

  return true; 
}

bool Planner::updateCurrentState() {
  std::cout << "------------------- updateCurrentState" << std::endl;
  geometry_msgs::TransformStamped transformStamped;

  geometry_msgs::PoseStamped current_base_pose;
  current_base_pose.header.stamp = ros::Time::now();
  current_base_pose.header.frame_id = "base";
  current_base_pose.pose.orientation.w = 1.0;

  try{
    transformStamped = tf_buffer_.lookupTransform(current_base_pose.header.frame_id, "map", ros::Time(0));
    tf2::doTransform(current_base_pose, current_base_pose, transformStamped);
    assert(current_base_pose.header.frame_id == "map");
    }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("%s",ex.what());

    planning_ = false;

    Goal result;
    result.success = false;
    get_path_action_srv_.setAborted(result);

    return false;
  }

  curr_state_.pos_ = Eigen::Vector3d(current_base_pose.pose.position.x, current_base_pose.pose.position.y, current_base_pose.pose.position.z);
  curr_state_.vel_ = Eigen::Vector3d::Zero();
  curr_state_.acc_ = Eigen::Vector3d::Zero();

  return true;
}

void Planner::run() {
  std::cout << "---------------------------------- run" << std::endl;
  if (planning_) {
    std::cout << "----------------------------- planning" << std::endl;
    // update current planning env.
    if(!(updateCurrentState() && updateObstacles())) {
      return;
    }

    // while position not reached: get to desired position
    if (!isPositionTargetReached()) {
      std::cout << "------------------- pos not reached" << std::endl;
      publishTargetTwistCommandApproach();
    } else {
      // while position reached, yaw not reached: rotate
      if (!isYawTargetReached()) {
      std::cout << "------------------- pos reached, yaw not reached" << std::endl;
        publishTargetTwistCommandFinalRotation();
      } else {
        std::cout << "------------------- everything reached" << std::endl;
        planning_ = false;

        Goal result;
        result.success = true;
        get_path_action_srv_.setSucceeded(result);
      }
    }
  } else {
    std::cout << "------------------- not planning" << std::endl;
  }

  return;
}

// todo: this can be different! Local guidance no longer used
void Planner::goalCB() {
  ROS_INFO("goalCB");
  std::cout << "------------------------------------------------------------------------ goalCB" << std::endl;
  
  // get the new goal
  auto requested_goal = get_path_action_srv_.acceptNewGoal();

  std::cout << "requested_goal: " << std::endl << *requested_goal << std::endl;

  tol_position_ = requested_goal->goal.tol.translation;
  tol_rotation_ = requested_goal->goal.tol.rotation;

  // set target
  target_policy_.setTarget(Eigen::Vector3d(requested_goal->goal.pose.position.x, requested_goal->goal.pose.position.y, requested_goal->goal.pose.position.z));

  // start planning
  des_position_ = Eigen::Vector2d(requested_goal->goal.pose.position.x, requested_goal->goal.pose.position.y);
  
  Eigen::Quaterniond q_goal(requested_goal->goal.pose.orientation.w, requested_goal->goal.pose.orientation.x, 
                            requested_goal->goal.pose.orientation.y, requested_goal->goal.pose.orientation.z);
  Eigen::Vector3d euler_angles_goal = q_goal.toRotationMatrix().eulerAngles(0, 1, 2); // XYZ order
  des_yaw_ = euler_angles_goal[2];
  max_linear_vel_ = requested_goal->max_linear_velocity;
  max_angular_vel_ = requested_goal->max_angular_velocity;

  planning_ = true;
}

void Planner::preemptCB() {
  ROS_INFO("preemptCB");
  std::cout << "------------------------------------------------------------------------ preemptCB" << std::endl;

  // set the action state to preempted
  planning_ = false;
  get_path_action_srv_.setPreempted();
}

}  //  namespace waverider_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waverider_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  double delta_t = 5.0;

  waverider_planner::Planner planner(nh, nh_private, delta_t);
  
  ros::Rate rate(1.0 / delta_t);
  while (ros::ok()) {
    ros::spinOnce();
    planner.run();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}