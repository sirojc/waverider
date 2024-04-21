#include "waverider_chomp_planner/waverider_planner.h"

#include <cmath>
#include <cassert>
#include <param_io/get_param.hpp>

#include <iostream>
#include <iomanip>

namespace waverider_planner {

Planner::Planner(ros::NodeHandle nh, ros::NodeHandle nh_private, double delta_t, bool load_map_from_file)
  : nh_(nh), nh_private_(nh_private), get_path_action_srv_(nh, "/waverider_planner/plan_path", false),
  tf_buffer_(), tf_listener_(tf_buffer_), tol_position_(0.1), tol_rotation_(0.1), planning_(false),
  last_time_stamp_ (0.0), prev_pos_(nullptr), prev_vel_(nullptr), prev_acc_(nullptr),
  reached_pos_(false), delta_t_(delta_t), curr_height_(0), curr_yaw_(0), load_map_from_file_(load_map_from_file) {
    // action
    get_path_action_srv_.registerGoalCallback(boost::bind(&Planner::goalCB, this));
    get_path_action_srv_.registerPreemptCallback(boost::bind(&Planner::preemptCB, this));
    get_path_action_srv_.start();

    // sub/pub init
    pub_twist_commands_ = nh_.advertise<geometry_msgs::TwistStamped>("/waverider_planner/twist", 10, true);
    pub_occupancy_ = nh.advertise<wavemap_msgs::Map>("waverider_planner/map", 10, true);
    sub_wavemap_ = nh_.subscribe("/wavemap/map", 1, &Planner::callbackMap, this);

    // map
    if (load_map_from_file) {
      ROS_INFO("loading map from file");
      // Load the occupancy map
      // wavemap::io::fileToMap("/home/nicole/ros/git/wavemap/data/example_maps/newer_college_mine_5cm.wvmp", occupancy_map_);
      wavemap::io::fileToMap("/home/nicole/ros/git/wavemap/data/anymal/map_lab.wvmp", occupancy_map_);
      CHECK_NOTNULL(occupancy_map_);

      // Publish the occupancy map
      wavemap_msgs::Map occupancy_map_msg;
      wavemap::convert::mapToRosMsg(*occupancy_map_, "odom", ros::Time::now(),
                                    occupancy_map_msg);
      pub_occupancy_.publish(occupancy_map_msg);
    }

    // policy initialization
    double alpha = param_io::param<double>(nh_, "/waverider_planner/target_policy/alpha", 1.0);
    double beta = param_io::param<double>(nh_, "/waverider_planner/target_policy/beta", 1.5);
    double gamma = param_io::param<double>(nh_, "/waverider_planner/target_policy/gamma", 0.001);
    double factor_A = param_io::param<double>(nh_, "/waverider_planner/target_policy/factor_A", 10.0);

    std::cout << "*********************** initializing target_policy_ with: alpha=" << alpha << ", beta=" << beta << ", gamma=" << gamma << ", factor_A=" << factor_A << std::endl;
    target_policy_.setTuning(alpha, beta, gamma);
    target_policy_.setA(Eigen::Matrix2d::Identity() * factor_A);

    waverider::PolicyTuning tuning;
    tuning.r = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/r", 1.3);
    tuning.c = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/c", 0.2);
    tuning.eta_rep = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/eta_rep", 22);
    tuning.nu_rep = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/nu_rep", 1.4);
    tuning.eta_damp = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/eta_damp", 140);
    tuning.nu_damp = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/nu_damp", 1.2);
    tuning.enable_repulsor = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/enable_repulsor", true);
    tuning.enable_damper = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/enable_damper", true);

    std::cout << "*********************** initializing PolicyTuning for waverider_policy_ with: r=" << tuning.r << ", c=" << tuning.c << ", eta_rep=" << tuning.eta_rep
          << ", nu_rep=" << tuning.nu_rep << ", eta_damp=" << tuning.eta_damp << ", nu_damp=" << tuning.nu_damp
          << ", enable_damper=" << tuning.enable_damper << ", enable_repulsor=" << tuning.enable_repulsor << std::endl; 
    waverider_policy_.updateTuning(tuning); 

    double occupancy_threshold = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/occupancy_threshold", 0.01);
    bool run_all_levels = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/run_all_levels", false);
    double lowest_level_radius = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/lowest_level_radius", 1.0);
    bool use_only_lowest_level = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/use_only_lowest_level", true);
    
    std::cout << "*********************** initializing other param waverider: occupancy_threshold=" << occupancy_threshold << ", run_all_levels=" << run_all_levels
              << ", lowest_level_radius=" << lowest_level_radius << ", use_only_lowest_level=" << use_only_lowest_level << std::endl;
    waverider_policy_.setOccupancyThreshold(occupancy_threshold);
    waverider_policy_.run_all_levels_ = run_all_levels;
    waverider_policy_.obstacle_filter_.lowest_level_radius_ = lowest_level_radius;
    waverider_policy_.obstacle_filter_.use_only_lowest_level_ = use_only_lowest_level;
  };

Planner::~Planner() {
  std::cout << "---------------------------------------------------------------- ~Planner" << std::endl;
  get_path_action_srv_.setAborted();
}

bool Planner::isPositionTargetReached() {
  // std::cout << "---------------------------------------------------------------- isPositionTargetReached" << std::endl;
  if(!reached_pos_) {
    // std::cout << "des_position_: " << des_position_ << std::endl;
    // std::cout << "curr_state_.pos_: " << curr_state_.pos_.head(2) << std::endl;

    Eigen::Vector2d distance(des_position_[0] - curr_state_.pos_[0], des_position_[1] - curr_state_.pos_[1]);

    // std::cout << "distance: " << distance.norm() << std::endl;
    // std::cout << "tol_position_: " << tol_position_ << std::endl;

    if (distance.norm() < tol_position_) {
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

bool Planner::isYawTargetReached() {
  // std::cout << "---------------------------------------------------------------- isYawTargetReached" << std::endl;

  // std::cout << "des_yaw_ (converted to deg): " << des_yaw_ * 180.0 / M_PI << std::endl;
  // std::cout << "curr_yaw_ (converted to deg): " << curr_yaw_ * 180.0 / M_PI << std::endl;

  double higher_angle = std::max(curr_yaw_, des_yaw_);
  double lower_angle = std::min(curr_yaw_, des_yaw_);

  double distance = std::fmod(higher_angle - lower_angle, M_PI);

  // std::cout << "distance: " << distance << std::endl;
  // std::cout << "tol_rotation_: " << tol_rotation_ << std::endl;

  if (distance < tol_rotation_) {
    return true;
  } else {
    return false;
  }
}

Eigen::Vector2d Planner::getLinearTargetAcceleration() {
  std::cout << "----------------------------------------------------- getLinearTargetAcceleration" << std::endl;
  // evaluate target policy
  rmpcpp::PolicyValue<2> target_result = target_policy_.evaluateAt(curr_state_);
  std::cout << "target_result acc: " << target_result.f_ << std::endl;
  std::cout << "scaled target_result acc: " << target_result.A_ * target_result.f_ << std::endl;

  // evaluate waverider policy
  rmpcpp::PolicyValue<2> waverider_result = waverider_policy_.evaluateAt(curr_state_, curr_height_, 0.5);
  std::cout << "waverider_result acc: " << waverider_result.f_ << std::endl;
  std::cout << "scaled waverider_result acc: " << waverider_result.A_ * waverider_result.f_ << std::endl;
  std::cout << "scaled waverider_result acc: deactivated" << std::endl;

  // get target acceleration
  std::vector<rmpcpp::PolicyValue<2>> policies = {target_result};
  // std::vector<rmpcpp::PolicyValue<2>> policies = {target_result, waverider_result};
  rmpcpp::PolicyValue<2> final_policy = rmpcpp::PolicyValue<2>::sum(policies);
  Eigen::Vector2d acceleration = final_policy.f_;

  std::cout << "initial accelerations_x_y: " << acceleration[0] << ", " << acceleration[1] << std::endl;

  // double scaling = std::min({max_linear_acc_ / std::abs(acceleration[0]), max_linear_acc_ / std::abs(acceleration[1]), 1.0});
  // acceleration[0] *= scaling;
  // acceleration[1] *= scaling;

  // acceleration[0] = std::min(std::max(acceleration[0], -max_linear_acc_), max_linear_acc_);
  // acceleration[1] = std::min(std::max(acceleration[1], -max_linear_acc_), max_linear_acc_);

  // std::cout << "cut accelerations_x_y: " << acceleration[0] << ", " << acceleration[1] << std::endl;

  return Eigen::Vector2d(acceleration.x(), acceleration.y());
}

Eigen::Vector2d Planner::getLinearTargetVelocity(const Eigen::Vector2d& accelerations) {
  std::cout << "----------------------------------------------------- getLinearTargetVelocity" << std::endl;

  // trapezoidal rule
  Eigen::Vector2d target_velocity = *prev_vel_ + (delta_t_ / 2.0) * (accelerations + *prev_acc_);

  std::cout << "initial target_velocity: " << target_velocity << std::endl;

  double scaling = std::min({max_linear_vel_ / std::abs(target_velocity[0]), max_linear_vel_ / std::abs(target_velocity[1]), 1.0});
  target_velocity[0] *= scaling;
  target_velocity[1] *= scaling;

  // target_velocity[0] = std::min(std::max(target_velocity[0], -max_linear_vel_), max_linear_vel_);
  // target_velocity[1] = std::min(std::max(target_velocity[1], -max_linear_vel_), max_linear_vel_);

  std::cout << "final target_velocity: " << target_velocity << std::endl;

  return target_velocity;
}

double Planner::getTargetYawVelocity(double des_heading, double curr_heading) {
  // std::cout << "----------------------------------------------------- getTargetYawVelocity" << std::endl;
  double diff_heading = des_heading - curr_heading;

  // std::cout << "des_heading (converted to deg): " << des_heading * 180.0 / M_PI << std::endl;
  // std::cout << "curr_heading (converted to deg): " << curr_heading * 180.0 / M_PI << std::endl;
  
  if (diff_heading > M_PI) {
    diff_heading -= 2 * M_PI;
  } else if(diff_heading < -M_PI) {
    diff_heading += 2 * M_PI;
  }

  // std::cout << "diff_heading (converted to deg): " << diff_heading * 180.0 / M_PI << std::endl;

  // p controller for jaw twist
  double k = 0.5; // todo: add as member var and tune
  double yaw_velocity = k * diff_heading;

  // std::cout << "initial yaw_velocity: " << yaw_velocity << std::endl;

  // ensure vel within range
  yaw_velocity = std::min(std::max(yaw_velocity, -max_angular_vel_), max_angular_vel_);

  // std::cout << "final yaw_velocity: " << yaw_velocity << std::endl;

  return yaw_velocity;
}

void Planner::TargetTwistCommandApproach() {
  std::cout << "---------------------------------------------------------------- TargetTwistCommandApproach" << std::endl;
  // get desired x,y accelerations
  Eigen::Vector2d accelerations_x_y = getLinearTargetAcceleration();

  // get desired x, y velocities
  Eigen::Vector2d velocities_x_y = getLinearTargetVelocity(accelerations_x_y);

  // get desired yaw velocity
  double des_yaw = std::fmod(std::atan2(velocities_x_y[1], velocities_x_y[0]), 2 * M_PI);
  double yaw_velocity = getTargetYawVelocity(des_yaw, curr_yaw_);

  // get entire twist command
  geometry_msgs::TwistStamped twist_command;
  twist_command.header.stamp = ros::Time::now();
  twist_command.header.frame_id = "odom"; // todo: probably not in this frame --> make variable and check what it should be
  twist_command.twist.linear.x = velocities_x_y[0];
  twist_command.twist.linear.y = velocities_x_y[1];
  twist_command.twist.linear.z = 0.0;
  twist_command.twist.angular.x = 0.0; // roll
  twist_command.twist.angular.y = 0.0; // pitch
  twist_command.twist.angular.z = yaw_velocity; // yaw

  publishTargetTwist(twist_command);
}

void Planner::TargetTwistCommandFinalRotation() {
  std::cout << "---------------------------------------------------------------- TargetTwistCommandFinalRotation" << std::endl;
  // get yaw velocity
  double yaw_velocity = getTargetYawVelocity(des_yaw_, curr_yaw_);

  // get entire twist command
  geometry_msgs::TwistStamped twist_command;
  twist_command.header.stamp = ros::Time::now();
  twist_command.header.frame_id = "odom"; // todo: probably not in this frame --> make variable and check what it should be
  twist_command.twist.linear.x = 0.0;
  twist_command.twist.linear.y = 0.0;
  twist_command.twist.linear.z = 0.0;
  twist_command.twist.angular.x = 0.0; // roll
  twist_command.twist.angular.y = 0.0; // pitch
  twist_command.twist.angular.z = yaw_velocity; // yaw

  publishTargetTwist(twist_command);
}

void Planner::publishTargetTwist(const geometry_msgs::TwistStamped target_twist_planner_frame) {
  std::cout << "---------------------------------------------------------------- publishTargetTwist" << std::endl;
  geometry_msgs::TransformStamped transformStamped;

  // todo: does it make sense to get transform again? maybe .inverse one from currentstate but might make sense to have up to date
  try{
    geometry_msgs::TransformStamped transformStamped = tf_buffer_.lookupTransform("base", target_twist_planner_frame.header.frame_id, ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s",ex.what());

    // planning_ = false;
    // reached_pos_ = false;

    // Goal result;
    // result.success = false;
    // get_path_action_srv_.setAborted(result);

    return;
  }

  geometry_msgs::TwistStamped target_twist_base;
  target_twist_base.header.stamp = ros::Time::now();
  target_twist_base.header.frame_id = "base";

  tf2::Quaternion rotation;
  tf2::fromMsg(transformStamped.transform.rotation, rotation);
  tf2::Vector3 lin_vel, ang_vel;
  tf2::fromMsg(target_twist_planner_frame.twist.linear, lin_vel);
  tf2::fromMsg(target_twist_planner_frame.twist.angular, ang_vel);

  tf2::Vector3 transformed_lin_vel, transformed_ang_vel;
  transformed_lin_vel = tf2::quatRotate(rotation, lin_vel);
  transformed_ang_vel = tf2::quatRotate(rotation, ang_vel);

  target_twist_base.twist.linear = tf2::toMsg(lin_vel);
  target_twist_base.twist.angular = tf2::toMsg(ang_vel);
  
  std::cout << "target_twist_base: " << target_twist_base << std::endl;

  pub_twist_commands_.publish(target_twist_base);
}

void Planner::callbackMap(const wavemap_msgs::Map::ConstPtr& msg) {
  ROS_INFO_ONCE("Received first map message.");

  // convert map message to map
  if (!load_map_from_file_) {
    wavemap::VolumetricDataStructureBase::Ptr map;
    wavemap::convert::rosMsgToMap(*msg, occupancy_map_);
  } else {
    ROS_INFO_ONCE("Using preloaded map, ignoring map messages.");
  }
}

bool Planner::updateObstacles() {
  std::cout << "---------------------------------------------------------------------- updateObstacles" << std::endl;

  // check if occupancy_map_ is set
  if (!occupancy_map_) {
    ROS_ERROR("Occupancy map not initialized yet.");
    planning_ = false;
    reached_pos_ = false;

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
    reached_pos_ = false;

    Goal result;
    result.success = false;
    get_path_action_srv_.setAborted(result);

    return false;
  }

  waverider_policy_.updateObstacles(*hashed_map_, Eigen::Vector3f(curr_state_.pos_[0], curr_state_.pos_[1], curr_height_)); 

  return true; 
}

bool Planner::updateCurrentState() {
  std::cout << "---------------------------------------------------------------- updateCurrentState" << std::endl;
  geometry_msgs::TransformStamped transformStamped;

  geometry_msgs::PoseStamped current_base_pose_base_frame;
  current_base_pose_base_frame.header.stamp = ros::Time::now();
  current_base_pose_base_frame.header.frame_id = "base";
  current_base_pose_base_frame.pose.orientation.w = 1.0;

  geometry_msgs::PoseStamped current_base_pose;
  current_base_pose.header.stamp = ros::Time::now();
  current_base_pose.header.frame_id = "odom";

  double current_time = 0.0;
  try{
    do {
      // std::cout << "updateCurrentState: waiting for transform" << std::endl;
      transformStamped = tf_buffer_.lookupTransform("odom", current_base_pose_base_frame.header.frame_id, ros::Time(0));
      current_time = transformStamped.header.stamp.sec + (transformStamped.header.stamp.nsec  / 1e9);
     } while (current_time == last_time_stamp_);
    
    // std::cout << "transformStamped: " << transformStamped << std::endl;
    
    tf2::doTransform(current_base_pose_base_frame, current_base_pose, transformStamped);
    assert(current_base_pose.header.frame_id == "odom");
    }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("%s",ex.what());

    // planning_ = false;
    // reached_pos_ = false;

    // Goal result;
    // result.success = false;
    // get_path_action_srv_.setAborted(result);

    return false;
  }

  std::cout << "current_base_pose: " << current_base_pose << std::endl;

  curr_height_ = current_base_pose.pose.position.z;

  tf2::Quaternion quat;
  tf2::convert(current_base_pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  curr_yaw_ = yaw;

  if(prev_pos_) {
    ROS_INFO_ONCE("prev_pos_ initialized");

    // std::cout << std::setprecision(9) << "current_time: " << current_time << std::endl;
    delta_t_ = current_time - last_time_stamp_;
    std::cout << std::setprecision(9) << "delta_t_: " << delta_t_ << std::endl;

    if (delta_t_ == 0) {
      ROS_WARN("No new transform, no need to update.");
      return true;
    }
    
    last_time_stamp_  = current_time;
    
    *prev_pos_ = curr_state_.pos_;
    curr_state_.pos_ = Eigen::Vector2d(current_base_pose.pose.position.x, current_base_pose.pose.position.y);
    
    if(prev_vel_) {
      ROS_INFO_ONCE("prev_vel_ initialized");
      *prev_vel_ = curr_state_.vel_;
      curr_state_.vel_ = Eigen::Vector2d((curr_state_.pos_[0] - (*prev_pos_)[0]) / delta_t_,
                                         (curr_state_.pos_[1] - (*prev_pos_)[1]) / delta_t_);

      if(prev_acc_) {
        ROS_INFO_ONCE("prev_acc_ initialized");
        *prev_acc_ = curr_state_.acc_;
        curr_state_.acc_ = Eigen::Vector2d((curr_state_.vel_[0] - (*prev_vel_)[0]) / delta_t_,
                                         (curr_state_.vel_[1] - (*prev_vel_)[1]) / delta_t_);
      } else {
        curr_state_.acc_ = Eigen::Vector2d((curr_state_.vel_[0] - (*prev_vel_)[0]) / delta_t_,
                                         (curr_state_.vel_[1] - (*prev_vel_)[1]) / delta_t_);
        prev_acc_ = new Eigen::Vector2d(curr_state_.acc_);;
      }

    } else {
      curr_state_.vel_ = Eigen::Vector2d((curr_state_.pos_[0] - (*prev_pos_)[0]) / delta_t_,
                                         (curr_state_.pos_[1] - (*prev_pos_)[1]) / delta_t_);
      prev_vel_ = new Eigen::Vector2d(curr_state_.vel_);
      curr_state_.acc_ = Eigen::Vector2d::Zero();
    }
  } else {
    last_time_stamp_ = transformStamped.header.stamp.sec + (transformStamped.header.stamp.nsec / 1e9);
    std::cout << std::setprecision(9) << "last_time_stamp_: " << last_time_stamp_ << std::endl;

    curr_state_.pos_ = Eigen::Vector2d(current_base_pose.pose.position.x, current_base_pose.pose.position.y);
    prev_pos_ = new Eigen::Vector2d(curr_state_.pos_.head(2));

    curr_state_.vel_ = Eigen::Vector2d::Zero();
    curr_state_.acc_ = Eigen::Vector2d::Zero();
  }

  // std::cout << "-- finished calc" << std::endl;
  // if (prev_pos_) {std::cout << "prev pos: " << *prev_pos_ << std::endl;}
  // if (prev_vel_) {std::cout << "prev vel: " << *prev_vel_ << std::endl;}
  // if (prev_acc_) {std::cout << "prev acc: " << *prev_acc_ << std::endl;}

  std::cout << "current pos: " << curr_state_.pos_ << std::endl;
  std::cout << "current yaw (converted to deg): " << curr_yaw_ * 180.0 / M_PI << std::endl;
  std::cout << "current vel: " << curr_state_.vel_ << std::endl;
  std::cout << "current acc: " << curr_state_.acc_ << std::endl;

  return true;
}

void Planner::run() {
  std::cout << "------------------------------------------------------------------------------- run" << std::endl;
  if(!updateCurrentState()) {// to always have accurate pos, vel and acc.
    return;
  }

  if (planning_) {
    std::cout << "-------------------------------------------------------------------------- planning" << std::endl;
    // update current planning env.
    if(!(updateObstacles())) {
      return;
    }

    // while position not reached: get to desired position
    if (!isPositionTargetReached()) {
      std::cout << "---------------------------------------------------------------- pos not reached" << std::endl;
      TargetTwistCommandApproach();
    } else {
      // while position reached, yaw not reached: rotate
      if (!isYawTargetReached()) {
        std::cout << "---------------------------------------------------------------- pos reached, yaw not reached" << std::endl;
        TargetTwistCommandFinalRotation();
      } else {
        std::cout << "---------------------------------------------------------------- everything reached" << std::endl;
        planning_ = false;

        Goal result;
        result.success = true;
        get_path_action_srv_.setSucceeded(result);
      }
    }
  } else {
    std::cout << "---------------------------------------------------------------- not planning" << std::endl;
  }

  return;
}

// todo: this can be different! Local guidance no longer used
void Planner::goalCB() {
  ROS_INFO("goalCB");
  std::cout << "--------------------------------------------------------------------------------------------------------------------- goalCB" << std::endl;
  
  // get the new goal
  auto requested_goal = get_path_action_srv_.acceptNewGoal();

  std::cout << "requested_goal: " << std::endl << *requested_goal << std::endl;

  // set tolerances
  tol_position_ = requested_goal->goal.tol.translation;
  tol_rotation_ = requested_goal->goal.tol.rotation;

  // set target
  des_position_ = Eigen::Vector2d(requested_goal->goal.pose.position.x, requested_goal->goal.pose.position.y);
  target_policy_.setTarget(Eigen::Vector2d(requested_goal->goal.pose.position.x, requested_goal->goal.pose.position.y));

  tf2::Quaternion quat;
  tf2::convert(requested_goal->goal.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  des_yaw_ = yaw;

  // limits
  max_linear_vel_ = requested_goal->max_linear_velocity;
  max_linear_acc_ = requested_goal->max_linear_acceleration;
  max_angular_vel_ = requested_goal->max_angular_velocity;

  // other stuff
  planning_ = true;
  }

void Planner::preemptCB() {
  ROS_INFO("preemptCB");
  std::cout << "--------------------------------------------------------------------------------------------------------------------- preemptCB" << std::endl;

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

  bool load_map_from_file = false;
  nh_private.getParam("load_map_from_file", load_map_from_file);
  std::cout << "load_map_from_file: " << load_map_from_file << std::endl;

  double delta_t = 0.1;

  waverider_planner::Planner planner(nh, nh_private, delta_t, load_map_from_file);
  
  ros::Rate rate(1.0 / delta_t);
  while (ros::ok()) {
    ros::spinOnce();
    planner.run();
    ros::spinOnce();
    rate.sleep();
  }
  // ros::spin();

  return 0;
}