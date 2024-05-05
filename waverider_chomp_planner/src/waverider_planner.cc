#include "waverider_chomp_planner/waverider_planner.h"

#include <cmath>
#include <cassert>
#include <param_io/get_param.hpp>

#include <iostream>
#include <iomanip>

namespace waverider_planner {

Planner::Planner(ros::NodeHandle nh, ros::NodeHandle nh_private, double delta_t, std::string planner_frame, int n_past_elements_pos,
                 int n_past_elements_vel, int n_past_elements_acc, int n_past_elements_des_vel_yaw, bool limit_acc_change, bool load_map_from_file)
  : nh_(nh), nh_private_(nh_private), get_path_action_srv_(nh, "/waverider_planner/plan_path", false), tf_buffer_(), tf_listener_(tf_buffer_),
  tol_position_(0.1), tol_rotation_(0.1), planning_(false), last_time_stamp_ (0.0), n_past_elements_pos_(n_past_elements_pos),
  n_past_elements_vel_(n_past_elements_vel), n_past_elements_acc_(n_past_elements_acc), n_past_elements_des_vel_yaw_(n_past_elements_des_vel_yaw), reached_pos_(false),
  prev_diff_heading_(nullptr), planner_frame_(planner_frame), limit_acc_change_(limit_acc_change), delta_t_(delta_t), curr_height_(0),
  curr_yaw_(0), load_map_from_file_(load_map_from_file) {
    // action
    get_path_action_srv_.registerGoalCallback(boost::bind(&Planner::goalCB, this));
    get_path_action_srv_.registerPreemptCallback(boost::bind(&Planner::preemptCB, this));
    get_path_action_srv_.start();

    // sub/pub init
    pub_twist_commands_ = nh_.advertise<geometry_msgs::TwistStamped>("/waverider_planner/twist", 10, true);
    sub_wavemap_ = nh_.subscribe("/wavemap/map", 1, &Planner::callbackMap, this);


    // for debug/ visualization
    pub_estimated_state_ = nh_.advertise<waverider_chomp_msgs::EstimatedState>("/waverider_planner/estimated_state", 10, true);
    pub_des_pos_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/waverider_planner/des_pos", 10, true);
    pub_des_vel_ = nh_.advertise<geometry_msgs::TwistStamped>("/waverider_planner/des_vel", 10, true);
    pub_des_acc_calc_ = nh_.advertise<waverider_chomp_msgs::DesiredLinearAccelerationCalculation>("/waverider_planner/des_acc_calc", 10, true);
    pub_des_yaw_vel_calc_ = nh_.advertise<waverider_chomp_msgs::DesiredYawVelocityCalculation>("/waverider_planner/des_yaw_vel_calc", 10, true);

    sub_twist_mux_ = nh_.subscribe("/twist_mux/twist", 1, &Planner::callbackTwistMux, this);
    pub_occupancy_ = nh.advertise<wavemap_msgs::Map>("waverider_planner/map", 10, true);
    pub_waverider_obstacles_ = nh_.advertise<visualization_msgs::MarkerArray>("/waverider_planner/waverider_obstacles", 1);

    // map
    if (load_map_from_file) {
      ROS_INFO("loading map from file");
      // Load the occupancy map
      // wavemap::io::fileToMap("/home/nicole/ros_rebased/catkin_ws/src/alma_rsl/dependencies/anymal_rsl/dependencies/wavemap/data/example_maps/newer_college_mine_5cm.wvmp", occupancy_map_);
      // wavemap::io::fileToMap("/home/nicole/ros_rebased/catkin_ws/src/alma_rsl/dependencies/anymal_rsl/dependencies/wavemap/data/anymal/map_lab.wvmp", occupancy_map_);
      wavemap::io::fileToMap("/home/nicole/ros_rebased/catkin_ws/src/alma_rsl/dependencies/anymal_rsl/dependencies/wavemap/data/anymal/slalom.wvmp", occupancy_map_);
      CHECK_NOTNULL(occupancy_map_);

      // Publish the occupancy map
      wavemap_msgs::Map occupancy_map_msg;
      wavemap::convert::mapToRosMsg(*occupancy_map_, planner_frame_, ros::Time::now(),
                                    occupancy_map_msg);
      pub_occupancy_.publish(occupancy_map_msg);
    }

    // velocity control
    k_vel_ctrl_ = param_io::param<double>(nh_, "/waverider_planner/velocity_controller/k", 0.5);
    std::cout << "*********************** initializing yaw_contrl with: k_vel_ctrl_=" << k_vel_ctrl_ << std::endl;

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
    tuning.enable_repulsor = param_io::param<bool>(nh_, "/waverider_planner/waverider_policy/enable_repulsor", true);
    tuning.enable_damper = param_io::param<bool>(nh_, "/waverider_planner/waverider_policy/enable_damper", true);

    std::cout << "*********************** initializing PolicyTuning for waverider_policy_ with: r=" << tuning.r << ", c=" << tuning.c << ", eta_rep=" << tuning.eta_rep
          << ", nu_rep=" << tuning.nu_rep << ", eta_damp=" << tuning.eta_damp << ", nu_damp=" << tuning.nu_damp
          << ", enable_damper=" << tuning.enable_damper << ", enable_repulsor=" << tuning.enable_repulsor << std::endl; 
    waverider_policy_.updateTuning(tuning); 

    double occupancy_threshold = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/occupancy_threshold", 0.01);
    bool run_all_levels = param_io::param<bool>(nh_, "/waverider_planner/waverider_policy/run_all_levels", false);
    double lowest_level_radius = param_io::param<double>(nh_, "/waverider_planner/waverider_policy/lowest_level_radius", 1.0);
    bool use_only_lowest_level = param_io::param<bool>(nh_, "/waverider_planner/waverider_policy/use_only_lowest_level", true);
    
    std::cout << "*********************** initializing other param waverider: occupancy_threshold=" << occupancy_threshold << ", run_all_levels=" << run_all_levels
              << ", lowest_level_radius=" << lowest_level_radius << ", use_only_lowest_level=" << use_only_lowest_level << std::endl;
    waverider_policy_.setOccupancyThreshold(occupancy_threshold);
    waverider_policy_.run_all_levels_ = run_all_levels;
    waverider_policy_.obstacle_filter_.lowest_level_radius_ = lowest_level_radius;
    waverider_policy_.obstacle_filter_.use_only_lowest_level_ = use_only_lowest_level;
  };

Planner::~Planner() {
  get_path_action_srv_.setAborted();
}

bool Planner::isPositionTargetReached() {
  // std::cout << "-------------------------------------------- isPositionTargetReached" << std::endl;
  if(!reached_pos_) {
    // std::cout << "------ des_position_: " << std::endl << des_position_ << std::endl;
    // std::cout << "------ curr_state_.pos_: " << std::endl << curr_state_.pos_.head(2) << std::endl;

    Eigen::Vector2d distance(des_position_[0] - curr_state_.pos_[0], des_position_[1] - curr_state_.pos_[1]);

    // std::cout << "------ distance: " << std::endl << distance.norm() << std::endl;
    // std::cout << "------ tol_position_: " << std::endl << tol_position_ << std::endl;

    if (distance.norm() < tol_position_) {
      reached_pos_ = true;
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

bool Planner::isYawTargetReached() {
  // std::cout << "-------------------------------------------- isYawTargetReached" << std::endl;

  // std::cout << "------ des_yaw_ (converted to deg): " << std::endl << des_yaw_ * 180.0 / M_PI << std::endl;
  // std::cout << "------ curr_yaw_ (converted to deg): " << std::endl << curr_yaw_ * 180.0 / M_PI << std::endl;

  double higher_angle = std::max(curr_yaw_, des_yaw_);
  double lower_angle = std::min(curr_yaw_, des_yaw_);

  double distance = std::fmod(higher_angle - lower_angle, M_PI);

  // std::cout << "------ distance: " << std::endl << distance << std::endl;
  // std::cout << "------ tol_rotation_: " << std::endl << tol_rotation_ << std::endl;

  if (distance < tol_rotation_) {
    return true;
  } else {
    return false;
  }
}

geometry_msgs::TwistStamped Planner::transform_twist(const geometry_msgs::TwistStamped& initial_twist, const std::string& target_frame,
                                                     const geometry_msgs::TransformStamped& transformStamped) {
                                                      
  geometry_msgs::TwistStamped target_twist_transformed;
  target_twist_transformed.header.stamp = ros::Time::now();
  target_twist_transformed.header.frame_id = target_frame;

  tf2::Quaternion rotation;
  tf2::fromMsg(transformStamped.transform.rotation, rotation);

  tf2::Vector3 lin_vel, ang_vel;
  tf2::fromMsg(initial_twist.twist.linear, lin_vel);
  tf2::fromMsg(initial_twist.twist.angular, ang_vel);

  // std::cout << "------ lin_vel: " << std::endl << lin_vel.x() << ", " << lin_vel.y() << ", " << lin_vel.z() << std::endl;
  // std::cout << "------ ang_vel: " << std::endl << ang_vel.x() << ", " << ang_vel.y() << ", " << ang_vel.z() << std::endl;

  tf2::Vector3 transformed_lin_vel, transformed_ang_vel;
  transformed_lin_vel = tf2::quatRotate(rotation, lin_vel);
  transformed_ang_vel = tf2::quatRotate(rotation, ang_vel);

  // std::cout << "------ transformed_lin_vel: " << std::endl << transformed_lin_vel.x() << ", " << transformed_lin_vel.y() << ", " << transformed_lin_vel.z() << std::endl;
  // std::cout << "------ transformed_ang_vel: " << std::endl << transformed_ang_vel.x() << ", " << transformed_ang_vel.y() << ", " << transformed_ang_vel.z() << std::endl;

  target_twist_transformed.twist.linear = tf2::toMsg(transformed_lin_vel);
  target_twist_transformed.twist.linear.z = 0.0;

  target_twist_transformed.twist.angular = tf2::toMsg(transformed_ang_vel);
  target_twist_transformed.twist.angular.x = 0.0;
  target_twist_transformed.twist.angular.y = 0.0;
  
  return target_twist_transformed;
}

double Planner::getMedian(const std::deque<double>& data) {
  std::deque<double> sorted_data(data);
  std::sort(sorted_data.begin(), sorted_data.end());
  int size = sorted_data.size();
  if (size % 2 == 0) {
      // Even number: median = average of middle two element
      return (sorted_data[size / 2 - 1] + sorted_data[size / 2]) / 2.0;
  } else {
      // Odd number: median =  middle element
      return sorted_data[size / 2];
  }
}

Eigen::Vector2d Planner::getLinearTargetAcceleration() {
  //  "----------------------------------------------------- getLinearTargetAcceleration" << std::endl;
  //  "------ curr_state_.pos_: " << std::endl << curr_state_.pos_ << std::endl;
  //  "------ curr_state_.vel_: " << std::endl << curr_state_.vel_ << std::endl;
  //  "------ curr_state_.acc_: " << std::endl << curr_state_.acc_ << std::endl;

  // evaluate target policy
  rmpcpp::PolicyValue<2> target_result = target_policy_.evaluateAt(curr_state_);
  //  "------ target_result acc: " << std::endl << target_result.f_ << std::endl;

  // evaluate waverider policy
  rmpcpp::PolicyValue<2> waverider_result = waverider_policy_.evaluateAt(curr_state_, curr_height_, 0.4);
  //  "------ waverider_result acc: " << std::endl << waverider_result.f_ << std::endl;
  //  "------ waverider_result A: " << std::endl << waverider_result.A_ << std::endl;
  // std::cout << "------ scaled waverider_result acc: deactivated" << std::endl;

  // get target acceleration
  // std::vector<rmpcpp::PolicyValue<2>> policies = {target_result};
  std::vector<rmpcpp::PolicyValue<2>> policies = {target_result, waverider_result};
  rmpcpp::PolicyValue<2> final_policy = rmpcpp::PolicyValue<2>::sum(policies);
  Eigen::Vector2d acceleration = final_policy.f_;

  //  "------ initial accelerations_x_y: " << std::endl << acceleration[0] << ", " << acceleration[1] << std::endl;

  double scaling = std::min({max_linear_acc_ / std::abs(acceleration[0]), max_linear_acc_ / std::abs(acceleration[1]), 1.0});
  acceleration[0] *= scaling;
  acceleration[1] *= scaling;

  //  "------ cut accelerations_x_y: " << std::endl << acceleration[0] << ", " << acceleration[1] << std::endl;

  Eigen::Vector2d scaled_target_acc = rmpcpp::PolicyValue<2>::pinv(final_policy.A_) * target_result.A_ * target_result.f_;
  Eigen::Vector2d scaled_waverider_acc = rmpcpp::PolicyValue<2>::pinv(final_policy.A_) * waverider_result.A_ * waverider_result.f_;

  waverider_chomp_msgs::DesiredLinearAccelerationCalculation des_acc_calc_msg;
  des_acc_calc_msg.header.stamp = ros::Time::now();
  des_acc_calc_msg.header.frame_id = planner_frame_;

  des_acc_calc_msg.des_lin_acc_target.x = scaled_target_acc[0];
  des_acc_calc_msg.des_lin_acc_target.y = scaled_target_acc[1];

  des_acc_calc_msg.des_lin_acc_waverider.x = scaled_waverider_acc[0];
  des_acc_calc_msg.des_lin_acc_waverider.y = scaled_waverider_acc[1];

  des_acc_calc_msg.des_lin_acc_final.x = acceleration[0];
  des_acc_calc_msg.des_lin_acc_final.y = acceleration[1];

  pub_des_acc_calc_.publish(des_acc_calc_msg);

  return acceleration;
}

Eigen::Vector2d Planner::getLinearTargetVelocity(const Eigen::Vector2d& des_accelerations) {
  // std::cout << "----------------------------------------------------- getLinearTargetVelocity" << std::endl;

  // trapezoidal rule
  // std::cout << "------ curr_state_.vel_: " << std::endl << curr_state_.vel_ << std::endl;
  // std::cout << "------ curr_state_.acc_: " << std::endl << curr_state_.acc_ << std::endl;
  // std::cout << "------ des_accelerations: " << std::endl << des_accelerations << std::endl;

  Eigen::Vector2d target_velocity;
  if(limit_acc_change_) {
    // std::cout << "------ acc. change is limited" << std::endl;
    target_velocity = curr_state_.vel_ + (delta_t_ / 2.0) * (des_accelerations + curr_state_.acc_);
  } else {
    // std::cout << "------ acc. change is NOT limited" << std::endl;
    target_velocity = curr_state_.vel_ + delta_t_ * des_accelerations;
  }

  // std::cout << "------ initial target_velocity: " << std::endl << target_velocity << std::endl;

  double scaling = std::min({max_linear_vel_ / std::abs(target_velocity[0]), max_linear_vel_ / std::abs(target_velocity[1]), 1.0});
  target_velocity[0] *= scaling;
  target_velocity[1] *= scaling;

  // std::cout << "------ final target_velocity: " << std::endl << target_velocity << std::endl;

  return target_velocity;
}

double Planner::getTargetYawVelocity(double des_heading, double curr_heading) {
  // des_heading = std::fmod(des_heading, 2 * M_PI);
  // curr_heading = std::fmod(curr_heading, 2 * M_PI);

  // std::cout << "----------------------------------------------------- getTargetYawVelocity" << std::endl;
  // current shortest angle
  double diff_heading = des_heading - curr_heading;

  // std::cout << "------ des_heading: " << std::endl << des_heading << "rad = " << des_heading * 180.0 / M_PI << "deg" << std::endl;
  // std::cout << "------ curr_heading: " << std::endl << curr_heading << "rad = " << curr_heading * 180.0 / M_PI << "deg" << std::endl;
  
  if (diff_heading > M_PI) {
    diff_heading -= 2 * M_PI;
  } else if(diff_heading < -M_PI) {
    diff_heading += 2 * M_PI;
  }

  if(!prev_diff_heading_) {
    prev_diff_heading_ = new double(diff_heading);
  }

  // std::cout << "------ prev_diff_heading_: " << std::endl << *prev_diff_heading_ << "rad = " << *prev_diff_heading_ * 180.0 / M_PI << "deg" << std::endl;
  // std::cout << "------ diff_heading before: " << std::endl << diff_heading << "rad = " << diff_heading * 180.0 / M_PI << "deg" << std::endl;
  
  if (abs(diff_heading) > 2.6 && diff_heading * (*prev_diff_heading_) < 0) {
    // std::cout << "************************** DIFFERENT SIGNS, CHECK IF SWITCH REQUIRED" << std::endl;
    double diff = std::fmod(std::max(diff_heading, *prev_diff_heading_) - std::min(diff_heading, *prev_diff_heading_), M_PI);
    diff = std::min(diff, M_PI - diff);
    // std::cout << "------ diff: " << std::endl << diff << "rad = " << diff * 180.0 / M_PI << "deg" << std::endl;

    if (diff < M_PI / 2) {
      // std::cout << "******************************************************************************** SWITCH TO INITIAL SIGN" << std::endl;
      if(diff_heading < 0) {
        diff_heading += 2 * M_PI;
      } else {
        diff_heading -= 2 * M_PI;
      }
    }
  }
  // std::cout << "------ diff_heading after: " << std::endl << diff_heading << "rad = " << diff_heading * 180.0 / M_PI << "deg" << std::endl;

  *prev_diff_heading_ = diff_heading;


  // p controller for jaw twist
  double yaw_velocity_unfiltered = k_vel_ctrl_ * diff_heading;
  std::cout << "---------------max_angular_vel_: " << max_angular_vel_ << std::endl;
  yaw_velocity_unfiltered = std::min(std::max(yaw_velocity_unfiltered, -max_angular_vel_), max_angular_vel_); // ensure within range

  if (past_des_vel_yaw_.size() == n_past_elements_des_vel_yaw_) {
    past_des_vel_yaw_.pop_front();
  } 
  past_des_vel_yaw_.push_back(yaw_velocity_unfiltered);

  double yaw_velocity = getMedian(past_des_vel_yaw_);

  // std::cout << "------ final yaw_velocity: " << std::endl << yaw_velocity << std::endl;


  waverider_chomp_msgs::DesiredYawVelocityCalculation yaw_vel_calc_msg;
  yaw_vel_calc_msg.header.stamp = ros::Time::now();
  yaw_vel_calc_msg.header.frame_id = planner_frame_;

  yaw_vel_calc_msg.curr_heading = curr_heading * 180.0 / M_PI;
  yaw_vel_calc_msg.des_heading = des_heading * 180.0 / M_PI;
  yaw_vel_calc_msg.diff_heading = diff_heading * 180.0 / M_PI;
  yaw_vel_calc_msg.yaw_vel_unfiltered = yaw_velocity_unfiltered * 180.0 / M_PI;
  yaw_vel_calc_msg.yaw_vel_filtered = yaw_velocity * 180.0 / M_PI;

  pub_des_yaw_vel_calc_.publish(yaw_vel_calc_msg);

  return diff_heading;
}

void Planner::TargetTwistCommandApproach() {
  // std::cout << "-------------------------------------------- TargetTwistCommandApproach" << std::endl;
  // get desired x,y accelerations
  Eigen::Vector2d accelerations_x_y = getLinearTargetAcceleration();

  // get desired x, y velocities
  Eigen::Vector2d velocities_x_y = getLinearTargetVelocity(accelerations_x_y);

  // get desired yaw velocity
  double des_yaw_vel = std::atan2(velocities_x_y[1], velocities_x_y[0]);
  double yaw_velocity = getTargetYawVelocity(des_yaw_vel, curr_yaw_);

  // get entire twist command
  geometry_msgs::TwistStamped twist_command;
  twist_command.header.stamp = ros::Time::now();
  twist_command.header.frame_id = planner_frame_;
  twist_command.twist.linear.x = velocities_x_y[0];
  twist_command.twist.linear.y = velocities_x_y[1];
  twist_command.twist.linear.z = 0.0;
  twist_command.twist.angular.x = 0.0; // roll
  twist_command.twist.angular.y = 0.0; // pitch
  twist_command.twist.angular.z = yaw_velocity; // yaw

  publishTargetTwist(twist_command);
}

void Planner::TargetTwistCommandFinalRotation() {
  // std::cout << "-------------------------------------------- TargetTwistCommandFinalRotation" << std::endl;
  // get yaw velocity
  double yaw_velocity = getTargetYawVelocity(des_yaw_, curr_yaw_);

  // get entire twist command
  geometry_msgs::TwistStamped twist_command;
  twist_command.header.stamp = ros::Time::now();
  twist_command.header.frame_id = planner_frame_;
  twist_command.twist.linear.x = 0.0;
  twist_command.twist.linear.y = 0.0;
  twist_command.twist.linear.z = 0.0;
  twist_command.twist.angular.x = 0.0; // roll
  twist_command.twist.angular.y = 0.0; // pitch
  twist_command.twist.angular.z = yaw_velocity; // yaw

  publishTargetTwist(twist_command);
}

void Planner::publishTargetTwist(const geometry_msgs::TwistStamped target_twist_planner_frame) {
  // std::cout << "-------------------------------------------- publishTargetTwist" << std::endl;

  assert(target_twist_planner_frame.header.frame_id == planner_frame_);
  pub_des_vel_.publish(target_twist_planner_frame);

  geometry_msgs::TransformStamped transformStamped;

  try{
    transformStamped = tf_buffer_.lookupTransform("base", target_twist_planner_frame.header.frame_id, ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s",ex.what());

    // planning_ = false;
    // reached_pos_ = false;

    // Goal result;
    // result.success = false;
    // get_path_action_srv_.setAborted(result);

    return;
  }

  // std::cout << "------ target_twist_planner_frame: " << std::endl << target_twist_planner_frame << std::endl;
  // std::cout << "------ transformStamped: " << std::endl << transformStamped << std::endl;

  geometry_msgs::TwistStamped target_twist_base = transform_twist(target_twist_planner_frame, "base", transformStamped);
  
  // std::cout << "------ target_twist_base: " << std::endl << target_twist_base << std::endl;
  assert(target_twist_base.header.frame_id == "base");
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

void Planner::callbackTwistMux(const geometry_msgs::TwistStamped& msg) {
  twist_mux_twist_ = msg;
}

bool Planner::updateObstacles() {
  //std::cout << "---------------------------------------------------------------------- updateObstacles" << std::endl;

  // check if occupancy_map_ is set
  if (!occupancy_map_) {
    ROS_ERROR("Occupancy map not initialized yet.");
    return false;
  }

  // update hashed map
  auto hashed_map_ = std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(occupancy_map_);
  if (!hashed_map_) {
    ROS_ERROR("Failed to cast occupancy map to HashedWaveletOctree.");
    return false;
  }

  waverider_policy_.updateObstacles(*hashed_map_, Eigen::Vector3f(curr_state_.pos_[0], curr_state_.pos_[1], curr_height_)); 

  visualization_msgs::MarkerArray marker_array;
  waverider::addFilteredObstaclesToMarkerArray(waverider_policy_.getObstacleCells(), planner_frame_, marker_array);
  pub_waverider_obstacles_.publish(marker_array);

  return true; 
}

bool Planner::updateCurrentState() {
  // std::cout << "-------------------------------------------- updateCurrentState" << std::endl;
  geometry_msgs::TransformStamped transformStamped;

  geometry_msgs::PoseStamped current_base_pose_base_frame;
  current_base_pose_base_frame.header.stamp = ros::Time::now();
  current_base_pose_base_frame.header.frame_id = "base";
  current_base_pose_base_frame.pose.orientation.w = 1.0;

  geometry_msgs::PoseStamped current_base_pose;

  double current_time = 0.0;
  try{
    do {
      // std::cout << "updateCurrentState: waiting for transform" << std::endl;
      transformStamped = tf_buffer_.lookupTransform(planner_frame_, current_base_pose_base_frame.header.frame_id, ros::Time(0));
      current_time = transformStamped.header.stamp.sec + (transformStamped.header.stamp.nsec  / 1e9);
    } while (current_time == last_time_stamp_);
    
    // std::cout << "transformStamped: " << transformStamped << std::endl;
    
    tf2::doTransform(current_base_pose_base_frame, current_base_pose, transformStamped);
    assert(current_base_pose.header.frame_id == planner_frame_);
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

  //std::cout << "------ current_base_pose: " << std::endl << current_base_pose << std::endl;

  curr_height_ = current_base_pose.pose.position.z;

  tf2::Quaternion quat;
  tf2::convert(current_base_pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  Eigen::Vector2d vel_unfiltered;
  Eigen::Vector2d acc_unfiltered;

  // solution using consecutive frames
  if(past_pos_x_.size() == n_past_elements_pos_){
    ROS_INFO_ONCE("past_pos_ initialized");

    prev_pos_ = curr_state_.pos_;

    past_pos_x_.push_back(current_base_pose.pose.position.x);
    past_pos_y_.push_back(current_base_pose.pose.position.y);
    past_yaw_.push_back(yaw);
    past_pos_x_.pop_front();
    past_pos_y_.pop_front();
    past_yaw_.pop_front();

    curr_state_.pos_ = Eigen::Vector2d(getMedian(past_pos_x_), getMedian(past_pos_y_));
    curr_yaw_ = getMedian(past_yaw_);

    // std::cout << std::setprecision(9) << "current_time: " << current_time << std::endl;
    delta_t_ = current_time - last_time_stamp_;
    // std::cout << std::setprecision(9) << "delta_t_: " << std::endl << delta_t_ << std::endl;
    
    // if(delta_t_ < 1.0) {
    //   return false;
    // }

    last_time_stamp_  = current_time;
    
    if(past_vel_x_.size() == n_past_elements_vel_) {
      ROS_INFO_ONCE("past_vel initialized");

      prev_vel_ = curr_state_.vel_;

      vel_unfiltered = Eigen::Vector2d((curr_state_.pos_[0] - prev_pos_[0]) / delta_t_, (curr_state_.pos_[1] - prev_pos_[1]) / delta_t_);

      past_vel_x_.push_back(vel_unfiltered[0]);
      past_vel_y_.push_back(vel_unfiltered[1]);
      past_vel_x_.pop_front();
      past_vel_y_.pop_front();

      curr_state_.vel_ = Eigen::Vector2d(getMedian(past_vel_x_), getMedian(past_vel_y_));

      if(past_acc_x_.size() == n_past_elements_acc_) {
        ROS_INFO_ONCE("past_acc initialized");

        prev_acc_ = curr_state_.acc_;

        past_acc_x_.push_back((curr_state_.vel_[0] - prev_vel_[0]) / delta_t_);
        past_acc_y_.push_back((curr_state_.vel_[1] - prev_vel_[1]) / delta_t_);
        past_acc_x_.pop_front();
        past_acc_y_.pop_front();

        curr_state_.acc_ = Eigen::Vector2d(getMedian(past_acc_x_), getMedian(past_acc_y_));
      } else {
        // initialize past_acc_
        acc_unfiltered = Eigen::Vector2d((curr_state_.vel_[0] - prev_vel_[0]) / delta_t_, (curr_state_.vel_[1] - prev_vel_[1]) / delta_t_);
        past_acc_x_.push_back(acc_unfiltered[0]);
        past_acc_y_.push_back(acc_unfiltered[1]);

        curr_state_.acc_ = Eigen::Vector2d(getMedian(past_acc_x_), getMedian(past_acc_y_));
        prev_acc_ = curr_state_.acc_;
      }

    } else {
      // initialize past_vel_
      vel_unfiltered = Eigen::Vector2d((curr_state_.pos_[0] - prev_pos_[0]) / delta_t_, (curr_state_.pos_[1] - prev_pos_[1]) / delta_t_);
      past_vel_x_.push_back(vel_unfiltered[0]);
      past_vel_y_.push_back(vel_unfiltered[1]);

      curr_state_.vel_ = Eigen::Vector2d(getMedian(past_vel_x_), getMedian(past_vel_y_));
      prev_vel_ = curr_state_.vel_;

      curr_state_.acc_ = Eigen::Vector2d::Zero();
      acc_unfiltered = Eigen::Vector2d::Zero();
    }
  } else {
    // initialize past_pos_
    last_time_stamp_ = transformStamped.header.stamp.sec + (transformStamped.header.stamp.nsec / 1e9);
    //std::cout << std::setprecision(9) << "last_time_stamp_: " << std::endl << last_time_stamp_ << std::endl;

    past_pos_x_.push_back(current_base_pose.pose.position.x);
    past_pos_y_.push_back(current_base_pose.pose.position.y);
    past_yaw_.push_back(yaw);

    curr_state_.pos_ = Eigen::Vector2d(getMedian(past_pos_x_), getMedian(past_pos_y_));
    curr_yaw_ = getMedian(past_yaw_);
    prev_pos_ = curr_state_.pos_;

    curr_state_.vel_ = Eigen::Vector2d::Zero();
    vel_unfiltered =  Eigen::Vector2d::Zero();
    curr_state_.acc_ = Eigen::Vector2d::Zero();
    acc_unfiltered = Eigen::Vector2d::Zero();
  }

  assert(past_pos_x_.size() == past_pos_y_.size());
  // assert(past_pos_x_.size() == past_yaw_.size());
  assert(past_vel_x_.size() == past_vel_y_.size());
  assert(past_acc_x_.size() == past_acc_y_.size());

  // std::cout << "-- finished calc" << std::endl;
  // if (prev_pos_) {std::cout << "------ prev pos: " << std::endl << prev_pos_ << std::endl;}
  // if (prev_vel_) {std::cout << "------ prev vel: " << std::endl << prev_vel_ << std::endl;}
  // if (prev_acc_) {std::cout << "------ prev acc: " << std::endl << prev_acc_ << std::endl;}

  //std::cout << "------ current height: " << std::endl << curr_height_ << std::endl;
  // std::cout << "------ current pos: " << std::endl << curr_state_.pos_ << std::endl;
  //std::cout << "------ current yaw (converted to deg): " << std::endl << curr_yaw_ * 180.0 / M_PI << std::endl;
  // std::cout << "------ current vel: " << std::endl << curr_state_.vel_ << std::endl;
  // std::cout << "------ current acc: " << std::endl << curr_state_.acc_ << std::endl;

  // publish desired pos
  geometry_msgs::Vector3Stamped des_pos_msg;
  des_pos_msg.header.stamp = ros::Time::now();
  des_pos_msg.header.frame_id = planner_frame_;
  des_pos_msg.vector.x = des_position_[0];
  des_pos_msg.vector.y = des_position_[1];
  des_pos_msg.vector.z = curr_height_;
  pub_des_pos_.publish(des_pos_msg);

  // publish estimated state
  waverider_chomp_msgs::EstimatedState est_state_msg;
  est_state_msg.header.stamp = ros::Time::now();
  est_state_msg.header.frame_id = planner_frame_;

  est_state_msg.yaw_unfiltered = yaw;
  est_state_msg.yaw_filtered = curr_yaw_;

  est_state_msg.pos_unfiltered.x = current_base_pose.pose.position.x;
  est_state_msg.pos_unfiltered.y = current_base_pose.pose.position.y;
  est_state_msg.pos_unfiltered.z = current_base_pose.pose.position.z;
  est_state_msg.pos_filtered.x = curr_state_.pos_[0];
  est_state_msg.pos_filtered.y = curr_state_.pos_[1];
  est_state_msg.pos_filtered.z = curr_height_;

  est_state_msg.lin_vel_unfiltered.x = vel_unfiltered[0];
  est_state_msg.lin_vel_unfiltered.y = vel_unfiltered[1];
  est_state_msg.lin_vel_filtered.x = curr_state_.vel_[0];
  est_state_msg.lin_vel_filtered.y = curr_state_.vel_[1];

  est_state_msg.lin_acc_unfiltered.x = acc_unfiltered[0];
  est_state_msg.lin_acc_unfiltered.y = acc_unfiltered[1];
  est_state_msg.lin_acc_filtered.x = curr_state_.acc_[0];
  est_state_msg.lin_acc_filtered.y = curr_state_.acc_[1];

  pub_estimated_state_.publish(est_state_msg);

  // publish transformed twist mux
  // std::cout << "------ twist base: " << std::endl << twist_mux_twist_ << std::endl;
  // geometry_msgs::TwistStamped target_twist_planner = transform_twist(twist_mux_twist_, planner_frame_, transformStamped);
  // std::cout << "------ twist planner: " << std::endl << target_twist_planner << std::endl;

  // pub_des_vel_.publish(target_twist_planner);

  return true;
}

void Planner::run() {
  std::cout << "------------------------------------------------------------------------------- run" << std::endl;
  if(!updateCurrentState()) {// to always have accurate pos, vel and acc.
    return;
  }

  if (planning_) {
    std::cout << "---------------------------------------------------------------- planning" << std::endl;
    // update current planning env.
    if(!(updateObstacles())) {
      planning_ = false;
      reached_pos_ = false;

      Goal result;
      result.success = false;
      get_path_action_srv_.setAborted(result);
      return;
    }

    // while position not reached: get to desired position
    if (!isPositionTargetReached()) {
      std::cout << "-------------------------------------------- pos not reached" << std::endl;
      TargetTwistCommandApproach();
    } else {
      // while position reached, yaw not reached: rotate
      if (!isYawTargetReached()) {
        std::cout << "-------------------------------------------- pos reached, yaw not reached" << std::endl;
        TargetTwistCommandFinalRotation();
      } else {
        std::cout << "-------------------------------------------- everything reached" << std::endl;
        planning_ = false;
        reached_pos_ = false;

        Goal result;
        result.success = true;
        get_path_action_srv_.setSucceeded(result);
      }
    }
  } else {
    std::cout << "-------------------------------------------- not planning" << std::endl;
  }

  return;
}

// todo: this can be different! Local guidance no longer used
void Planner::goalCB() {
  ROS_INFO("goalCB");
  //  "--------------------------------------------------------------------------------------------------------------------- goalCB" << std::endl;
  
  // get the new goal
  auto requested_goal = get_path_action_srv_.acceptNewGoal();
  assert(requested_goal->goal.header.frame_id == planner_frame_);

  std::cout << "----------------------requested_goal: " << std::endl << *requested_goal << std::endl;

  // set tolerances
  tol_position_ = requested_goal->goal.tol.translation;
  tol_rotation_ = requested_goal->goal.tol.rotation;

  // set target
  des_position_ = Eigen::Vector2d(requested_goal->goal.pose.position.x, requested_goal->goal.pose.position.y);
  target_policy_.setTarget(des_position_);

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
  reached_pos_ = false;

  past_des_vel_yaw_.clear();
}

void Planner::preemptCB() {
  ROS_INFO("preemptCB");
  //  "--------------------------------------------------------------------------------------------------------------------- preemptCB" << std::endl;

  // set the action state to preempted
  planning_ = false;
  reached_pos_ = false;
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

  std::string planner_frame;
  nh_private.getParam("planner_frame", planner_frame);
  std::cout << "planner_frame: " << planner_frame.c_str() << std::endl;

  double n_past_seconds_pos, n_past_seconds_vel, n_past_seconds_acc, n_past_seconds_des_yaw;
  nh_private.getParam("n_past_seconds_pos", n_past_seconds_pos);
  nh_private.getParam("n_past_seconds_vel", n_past_seconds_vel);
  nh_private.getParam("n_past_seconds_acc", n_past_seconds_acc);
  nh_private.getParam("n_past_seconds_des_yaw", n_past_seconds_acc);
  std::cout << "n_past_seconds_pos: " << n_past_seconds_pos << std::endl;
  std::cout << "n_past_seconds_vel: " << n_past_seconds_vel << std::endl;
  std::cout << "n_past_seconds_acc: " << n_past_seconds_acc << std::endl;
  std::cout << "n_past_seconds_des_yaw: " << n_past_seconds_des_yaw << std::endl;

  double delta_t = 0.1;
  int n_past_elements_pos = std::ceil(n_past_seconds_pos / delta_t); // round up
  n_past_elements_pos += std::fmod(n_past_elements_pos + 1, 2); // make uneven
  std::cout << "n_past_elements_pos: " << n_past_elements_pos << std::endl;

  int n_past_elements_vel = std::ceil(n_past_seconds_vel / delta_t); // round up
  n_past_elements_vel += std::fmod(n_past_elements_vel + 1, 2); // make uneven
  std::cout << "n_past_elements_vel: " << n_past_elements_vel << std::endl;

  int n_past_elements_acc = std::ceil(n_past_seconds_acc / delta_t); // round up
  n_past_elements_acc += std::fmod(n_past_elements_acc + 1, 2); // make uneven
  std::cout << "n_past_elements_acc: " << n_past_elements_acc << std::endl;

  int n_past_elements_des_vel_yaw = std::ceil(n_past_seconds_des_yaw / delta_t); // round up
  n_past_elements_des_vel_yaw += std::fmod(n_past_elements_des_vel_yaw + 1, 2); // make uneven
  std::cout << "n_past_elements_des_vel_yaw: " << n_past_elements_des_vel_yaw << std::endl;

  bool limit_acc_change;
  nh_private.getParam("limit_acc_change", limit_acc_change);
  std::cout << "limit_acc_change: " << limit_acc_change << std::endl;
  

  // n_past_elements = 1;

  waverider_planner::Planner planner(nh, nh_private, delta_t, planner_frame, n_past_elements_pos,
                                     n_past_elements_vel, n_past_elements_acc, n_past_elements_des_vel_yaw,
                                     limit_acc_change, load_map_from_file);
  
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