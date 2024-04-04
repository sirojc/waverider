#include "waverider_chomp_planner/waverider_chomp_planner.h"

#include <iostream>

namespace waverider_chomp_planner {

Planner::Planner(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nh_(nh), nh_private_(nh_private), get_path_action_srv_(nh, "/waverider_chomp_planner/plan_path", false) {
  std::cout << "1" << std::endl;
  //register the goal and feeback callbacks
  get_path_action_srv_.registerGoalCallback(boost::bind(&Planner::goalCB, this));
  get_path_action_srv_.registerPreemptCallback(boost::bind(&Planner::preemptCB, this));
  
  get_path_action_srv_.start();
  std::cout << "2" << std::endl;
  // this can't be here -> should be beginning of callback?

  // ros subcribers/ publisher
  occupancy_pub_ = nh.advertise<wavemap_msgs::Map>("map", 10, true);
  esdf_pub_ = nh.advertise<wavemap_msgs::Map>("esdf", 10, true);
  waverider_map_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>("filtered_obstacles", 1);
  state_pub_ = nh.advertise<visualization_msgs::Marker>("rmp_state", 10, true);
  trajectory_pub_ = nh.advertise<visualization_msgs::Marker>("trajectory", 10, true);
  trajectory_pub_arrows_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_arrows", 10, true);
  
  map_sub_ = nh_.subscribe("map", 1, &Planner::callbackMap, this);

  // TODO: MAKE MAP UPDATE AND NOT FROM FILE, CHECK WAVERIDER HOW THEY DO IT
  // Load the occupancy map
  // wavemap::io::fileToMap("/home/nicole/ros/git/wavemap/data/example_maps/newer_college_mine_5cm.wvmp", occupancy_map_);
  wavemap::io::fileToMap("/home/nicole/ros/git/wavemap/data/anymal/map_lab.wvmp", occupancy_map_);
  CHECK_NOTNULL(occupancy_map_);

  // Publish the occupancy map
  wavemap_msgs::Map occupancy_map_msg;
  wavemap::convert::mapToRosMsg(*occupancy_map_, planner_frame_, ros::Time::now(),
                                occupancy_map_msg);
  occupancy_pub_.publish(occupancy_map_msg);

  // Currently, only hashed wavelet octree maps are supported as input
  hashed_map_ = std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(occupancy_map_);
  if (!hashed_map_) {
    ROS_ERROR("Failed to cast occupancy map to HashedWaveletOctree.");
  }

  // Generate the ESDF
  esdf_ = std::make_shared<wavemap::HashedBlocks>(generateEsdf(*hashed_map_, kOccupancyThreshold_, kMaxDistance_));

  // Publish the ESDF
  wavemap_msgs::Map msg;
  wavemap::convert::mapToRosMsg(*esdf_, planner_frame_, ros::Time::now(), msg);
  esdf_pub_.publish(msg);

  // Define the ESDF distance getter
  distance_getter_ = [this](const Eigen::Vector2d& position_d) {
    const wavemap::Point3D position(position_d[0], position_d[1], this->height_robot_);
    return wavemap::interpolateTrilinear(*this->esdf_, position);
    // if (wavemap::interpolateTrilinear(*this->occupancy_map_, position) < kOccupancyThreshold_) { // I think this checks whether or not within map?
    //     return wavemap::interpolateTrilinear(*this->esdf_, position);
    // } else {
    //     return 0.f;
    // }
  };

  // Initialize CHOMP
  params_.map_resolution =(*esdf_).getMinCellWidth();
  params_.w_collision = 10.0;
  params_.w_smooth = 0.1;
  params_.lambda = 120;  // 20.1
  params_.max_iter = 100;
  params_.epsilon = paddedRobotRadius_;
  params_.decrease_step_size = true;
  params_.D = 2; // dimensionalilty state (here: only x and y position)
  chomp_.setParameters(params_);
  chomp_.setDistanceFunction(distance_getter_);

  // Initialize Waverider
  // TODO: WHAT COULD BE HERE?

  ROS_INFO("Ready for queries.");
}

bool Planner::checkTrajCollision(const Eigen::MatrixXd& trajectory) const {
  // check if the trajectory is collision free
  bool is_collision_free = true;
  for (int idx = 0; idx < trajectory.rows(); ++idx) {
    const auto position = trajectory.row(idx);
    const float esdf_distance = distance_getter_(position);
    if (esdf_distance <= kRobotRadius_) {
      std::cout << "Collision at position: " << position << " with distance: " << esdf_distance << std::endl;
      is_collision_free = false;
      break;
    }
  }

  return is_collision_free;
}

void Planner::getTrajectory(const geometry_msgs::Pose& start,
                            const geometry_msgs::Pose& goal,
                            const waverider_chomp_msgs::PlannerType& planner_type,
                            const bool ignore_orientation,
                            const navigation_msgs::Tolerance tol,
                            const std::string local_guidance_mode) {
  publishLocalPlannerFeedback(Feedback::PLANNING);
  
  // Publish the start and goal positions
  {
    // Set up the marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = planner_frame_;
    marker.header.stamp = ros::Time::now();
    marker.id = 100;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = kRobotRadius_;
    marker.scale.y = kRobotRadius_;
    marker.scale.z = kRobotRadius_;
    marker.color.a = 0.5;

    // Publish the start position sphere
    marker.ns = "start";
    marker.color.b = 1.0;
    marker.pose = start; // TODO: DECIDE WHAT TO USE HERE
    trajectory_pub_.publish(marker);
    // Publish the goal position sphere
    marker.ns = "goal";
    marker.color.b = 0.0;
    marker.color.g = 1.0;
    marker.pose = goal;
    trajectory_pub_.publish(marker);

    // publish the start pose arrow
    marker.ns = "start_arrow";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.2; // arrow shaft diameter
    marker.scale.y = 0.3; // arrow head diameter
    marker.scale.z = 0.2; // arrow head length
    marker.color.b = 1.0;
    marker.color.g = 0.0;
    marker.color.a = 1.0;
    marker.pose = start;
    trajectory_pub_.publish(marker);
    // publish the goal pose arrow
    marker.ns = "goal_arrow";
    marker.color.b = 0.0;
    marker.color.g = 1.0;
    marker.pose = goal;
    trajectory_pub_.publish(marker);
  }
  
  Eigen::MatrixXd traj;
  
  if (planner_type.type == waverider_chomp_msgs::PlannerType::CHOMP) {
     traj = getChompTrajectory(start, goal);
  } else if (planner_type.type == waverider_chomp_msgs::PlannerType::WAVERIDER) {
    traj = getWaveriderTrajectory(start, goal);
  } else {
    ROS_ERROR("Unknown planner type");
  }

  // Check if the trajectory is collision free
  bool is_collision_free = checkTrajCollision(traj);

  if (!is_collision_free) {
    LOG(INFO) << "Solution trajectory is NOT collision free";
    publishLocalPlannerFeedback(Feedback::NO_SOLUTION);
    return;
  } else {
    LOG(INFO) << "Solution trajectory is collision free";
    publishLocalPlannerFeedback(Feedback::FOUND_SOLUTION);
  
    // get the full trajectory
    Eigen::MatrixXd full_traj = get_full_traj(traj, start, goal);

    // visualization marker
    visualizeTrajectory(full_traj);

    // response trajectory
    navigation_msgs::PathLocalGuidance optimized_path;
    navigation_msgs::PathSegmentLocalGuidance segment;
    
    // Set values that are the same for all.
    segment.goal.tol = tol;
    segment.goal.ignore_orientation = ignore_orientation;
    segment.goal.header.frame_id = planner_frame_;
    segment.goal.header.stamp = ros::Time::now();
    segment.local_guidance_mode = local_guidance_mode;
    optimized_path.header = segment.goal.header;

    // set response trajectory
    double prev_time = 0.0;
    double delta_t = 0.0;
    for (int idx = 0; idx < full_traj.rows(); ++idx) {
      geometry_msgs::Pose traj_step;

      // position
      traj_step.position.x = full_traj(idx, 0);
      traj_step.position.y = full_traj(idx, 1);
      traj_step.position.z = full_traj(idx, 2);

      // get quaternion from euler angles
      Eigen::Quaterniond q_curr;
      q_curr = Eigen::AngleAxisd(full_traj(idx, 3), Eigen::Vector3d::UnitX())
               * Eigen::AngleAxisd(full_traj(idx, 4), Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(full_traj(idx, 5), Eigen::Vector3d::UnitZ());

      traj_step.orientation.x = q_curr.x();  
      traj_step.orientation.y = q_curr.y();
      traj_step.orientation.z = q_curr.z();
      traj_step.orientation.w = q_curr.w();

      segment.goal.pose = traj_step;
      optimized_path.path_segments.push_back(segment);
    }

    // Send path as feedback
    feedback_->path_optimized = optimized_path;
    publishLocalPlannerFeedback(Feedback::PUBLISHED_SOLUTION);
  }

  return;
}

Eigen::MatrixXd Planner::getChompTrajectory(const geometry_msgs::Pose& start,
                                            const geometry_msgs::Pose& goal) {    
  Eigen::Vector2d start_pos(start.position.x, start.position.y);
  Eigen::Vector2d goal_pos(goal.position.x, goal.position.y);
  
  chomp::ChompTrajectory chomp_output;
  int N = 500;
  chomp_.solveProblem(start_pos, goal_pos, N, &chomp_output);

  return chomp_output.trajectory;
}

Eigen::MatrixXd Planner::getWaveriderTrajectory(const geometry_msgs::Pose& start,
                                                         const geometry_msgs::Pose& goal) {
  std::cout << "1" << std::endl;
  // rest to rest trajectories!
  rmpcpp::State<3> start_r3;
  start_r3.pos_ = Eigen::Vector3d(start.position.x, start.position.y, start.position.z);
  start_r3.vel_ = Eigen::Vector3d::Zero();
  start_r3.acc_ = Eigen::Vector3d::Zero();
  std::cout << "2" << std::endl;

  // configure policies
  Eigen::Vector3d end(goal.position.x, goal.position.y, goal.position.z);
  rmpcpp::SimpleTargetPolicy<rmpcpp::Space<3>> target_policy;
  target_policy.setTuning(10, 15, 0.01);
  target_policy.setTarget(end);
  target_policy.setA(Eigen::Matrix3d::Identity()*10);
  std::cout << "3" << std::endl;

  waverider::WaveriderPolicy waverider_policy;
  if(flat_res_){
    waverider_policy.run_all_levels_ = false;
    waverider_policy.obstacle_filter_.lowest_level_radius_ = flat_res_radius_;
    waverider_policy.obstacle_filter_.use_only_lowest_level_ = true;
  }
  std::cout << "4" << std::endl;

  std::vector<Eigen::Vector3d> trajectory_pieces;

  // TODO: CURRENTLY INTEGRATE TO POSITION HERE TO TAKE DERIVATIVE AFTERWARDS
  // -> THINK ABOUT HOW THIS CAN BE DONE BETTER
  rmpcpp::TrapezoidalIntegrator<rmpcpp::State<3>> integrator(start_r3, 0.01);
  Eigen::Vector3d last_updated_pos = {-10000.0, -10000.0, -10000.0};
  int i =0;

  std::cout << "5" << std::endl;
  // lambda to make victor happy
  // tiny bit more efficient -> victor only slightly angry/disappointed.
  auto policy_sum = [&](const rmpcpp::State<3>& state) {

    trajectory_pieces.push_back(state.pos_);

    // update obstacles at current position

    // sum policies
    if((last_updated_pos - state.pos_).norm() > 0.1) {
      waverider_policy.updateObstacles(*hashed_map_, state.pos_.cast<float>());
     
      // visualization
      visualization_msgs::MarkerArray marker_array;
      waverider::addFilteredObstaclesToMarkerArray(waverider_policy.getObstacleCells(), planner_frame_, marker_array);
      waverider_map_pub_.publish(marker_array);

      last_updated_pos = state.pos_;
    }

    // publishState(state.pos_, state.vel_);

    auto waverider_result =waverider_policy.evaluateAt(state);
    auto target_result = target_policy.evaluateAt(state);

    // return acceleration
    return (target_result+waverider_result).f_;
  };
  std::cout << "6" << std::endl;

  bool got_to_rest =
      integrator.integrateSteps(policy_sum, max_integration_steps_);

  std::cout << "7" << std::endl;
  bool success = got_to_rest;

  std::cout << "success: " << success << std::endl;

  std::cout << "8" << std::endl;
  Eigen::MatrixXd trajectory(trajectory_pieces.size(), 2);
  for (int i = 0; i < trajectory_pieces.size(); ++i) {
    trajectory(i, 0) = trajectory_pieces[i][0];
    trajectory(i, 1) = trajectory_pieces[i][1];
  }

  std::cout << "9" << std::endl;
  return trajectory;
}

// todo: CHECK IF START IS PART OF TRAJECTORY OR NOT - HANDLE ACCORDINGLY
// TODO: implement this better (include z, roll, pitch for start? To start from current configuration?)
Eigen::MatrixXd Planner::get_full_traj(const Eigen::MatrixXd chomp_traj, const geometry_msgs::Pose start,
                                            const geometry_msgs::Pose goal) const {
  int n_elements = chomp_traj.rows();

  // x, y given through trajectory

  // calculate z for each step
  Eigen::MatrixXd z_pos = Eigen::MatrixXd::Ones(n_elements, 1) * height_robot_;

  // calculate roll, pitch for each step
  // TODO: CHECK IF NORMAL ANYMAL CONFIGURATION ROLL = 0 AND PITCH = 0 (with simulation)
  Eigen::MatrixXd roll = Eigen::MatrixXd::Zero(n_elements, 1);
  Eigen::MatrixXd pitch = Eigen::MatrixXd::Zero(n_elements, 1);

  // calculate yaw for each step
  Eigen::MatrixXd yaw_angle = Eigen::MatrixXd::Zero(n_elements, 1);
  for (int i = 0; i < chomp_traj.rows() - 1; ++i) {
      double diff_x = chomp_traj(i + 1, 0) - chomp_traj(i, 0);
      double diff_y = chomp_traj(i + 1, 1) - chomp_traj(i, 1);
      
      yaw_angle(i) = atan2(diff_y, diff_x);
  }
  Eigen::Quaterniond q_goal(goal.orientation.w, goal.orientation.x, goal.orientation.y, goal.orientation.z);
  Eigen::Vector3d euler_angles_goal = q_goal.toRotationMatrix().eulerAngles(0, 1, 2); // XYZ order
  yaw_angle(n_elements - 1) = euler_angles_goal[2];

  Eigen::MatrixXd full_traj(n_elements, 6);
  full_traj << chomp_traj, z_pos, roll, pitch, yaw_angle;

  CHECK_EQ(full_traj.cols(), 6);

  return full_traj;
}

void Planner::callbackMap(const wavemap_msgs::Map::ConstPtr& msg) {
  // convert map message to map
  wavemap::VolumetricDataStructureBase::Ptr map;
  wavemap::convert::rosMsgToMap(*msg, occupancy_map_);
}

void Planner::updateMap(bool update_esdf) {
  // republish occupancy map, use latest received occupancy map
  wavemap_msgs::Map occupancy_map_msg;
  wavemap::convert::mapToRosMsg(*occupancy_map_, planner_frame_, ros::Time::now(), occupancy_map_msg);
  occupancy_pub_.publish(occupancy_map_msg);

  // update hashed map
  hashed_map_ = std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(occupancy_map_);
  if (!hashed_map_) {
    ROS_ERROR("Failed to cast occupancy map to HashedWaveletOctree.");
  }

  if (update_esdf) {
    // update and publish esdf
    esdf_ = std::make_shared<wavemap::HashedBlocks>(generateEsdf(*hashed_map_, kOccupancyThreshold_, kMaxDistance_));

    wavemap_msgs::Map msg;
    wavemap::convert::mapToRosMsg(*esdf_, planner_frame_, ros::Time::now(), msg);
    esdf_pub_.publish(msg);
  }
}


void Planner::visualizeTrajectory(const Eigen::MatrixXd& trajectory) const {
  LOG(INFO) << "Publishing trajectory";
  visualization_msgs::Marker trajectory_msg;
  trajectory_msg.header.frame_id = planner_frame_;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_msg.action = visualization_msgs::Marker::ADD;
  trajectory_msg.id = 100;
  trajectory_msg.ns = "trajectory";
  trajectory_msg.scale.x = kRobotRadius_;
  trajectory_msg.scale.y = kRobotRadius_;
  trajectory_msg.scale.z = kRobotRadius_;
  trajectory_msg.color.r = 1.0;
  trajectory_msg.color.a = 1.0;
  trajectory_msg.pose.orientation.w = 1.0;

  visualization_msgs::MarkerArray trajectory_arrow_msg;

  for (int idx = 0; idx < trajectory.rows(); ++idx) {
    auto& position_msg = trajectory_msg.points.emplace_back();
    position_msg.x = trajectory(idx, 0);
    position_msg.y = trajectory(idx, 1);
    position_msg.z = trajectory(idx, 2);

    if (idx % 50 == 0) {
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = planner_frame_;
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "trajectory";
      arrow.id = idx; // Use idx as the unique ID for each marker
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.scale.x = 0.2; // Shaft diameter
      arrow.scale.y = 0.4; // Head diameter
      arrow.scale.z = 0.3; // Head length
      arrow.color.b = 1.0;
      arrow.color.g = 1.0;
      arrow.color.a = 1.0;

      // the position of the arrow
      arrow.pose.position.x = trajectory(idx, 0);
      arrow.pose.position.y = trajectory(idx, 1);
      arrow.pose.position.z = trajectory(idx, 2);

      // orientation of the arrow
      Eigen::Quaterniond q;
      q = Eigen::AngleAxisd(trajectory(idx, 3), Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(trajectory(idx, 4), Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(trajectory(idx, 5), Eigen::Vector3d::UnitZ());
      arrow.pose.orientation.x = q.x();
      arrow.pose.orientation.y = q.y();
      arrow.pose.orientation.z = q.z();
      arrow.pose.orientation.w = q.w();

      // Add the arrow marker to the MarkerArray
      trajectory_arrow_msg.markers.push_back(arrow);
    }
  }

  trajectory_pub_.publish(trajectory_msg);
  trajectory_pub_arrows_.publish(trajectory_arrow_msg);
}

void Planner::visualizeState(const Eigen::Vector3d& pos) const {
  // nav_msgs::Odometry msg;
  // msg.header.frame_id = "map";
  // msg.child_frame_id = "rmp_state";
  // msg.pose.pose.position.x = pos.x();
  // msg.pose.pose.position.y = pos.y();
  // msg.pose.pose.position.z = pos.z();

  // state_pub_.publish(msg);

  // Set up the marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = planner_frame_;
  marker.header.stamp = ros::Time::now();
  marker.id = 100;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = kRobotRadius_;
  marker.scale.y = kRobotRadius_;
  marker.scale.z = kRobotRadius_;
  marker.color.a = 0.5;
  marker.color.b = 1.0;
  marker.pose.position.x = pos.x();
  marker.pose.position.y = pos.y();
  marker.pose.position.z = pos.z();
  state_pub_.publish(marker);

}

void Planner::publishLocalPlannerFeedback(FeedbackStatus feedback) {
  feedback_->status_local_planner = feedback;
  get_path_action_srv_.publishFeedback(feedback_);
}

void Planner::goalCB() {
  ROS_INFO("goalCB");
  
  // get the new goal
  auto plan_req = get_path_action_srv_.acceptNewGoal();

  int n_elements = plan_req->path.path_segments.size();
  navigation_msgs::PoseStamped start_pose = plan_req->path.path_segments[0].goal;
  navigation_msgs::PoseStamped goal_pose = plan_req->path.path_segments[n_elements - 1].goal;
  // // waverider_chomp_msgs::PlannerType planner_type = plan_req->planner_type; // TODO: ADD CHOICE IN MESSAGE OR WHEN STARTUP PLANNER?
  waverider_chomp_msgs::PlannerType planner_type;
  planner_type.type = waverider_chomp_msgs::PlannerType::CHOMP;

  std::cout << "---------------- start_pose: " << std::endl << start_pose << std::endl;
  std::cout << "---------------- goal_pose: " << std::endl << goal_pose << std::endl;

  // update map to get newest data
  // updateMap(planner_type == waverider_chomp_msgs::CHOMP);

  // adapt robot height
  height_robot_ = start_pose.pose.position.z;

  double min_distance = distance_getter_(Eigen::Vector2d(start_pose.pose.position.x, start_pose.pose.position.y));
  if (min_distance < kRobotRadius_) {
    publishLocalPlannerFeedback(Feedback::INVALID_START);
    ROS_INFO("Not planning, start pose invalid");
    return;
  }

  // TODO: check if goal is collision free
  min_distance = distance_getter_(Eigen::Vector2d(goal_pose.pose.position.x, goal_pose.pose.position.y));
  if (min_distance < kRobotRadius_) {
    publishLocalPlannerFeedback(Feedback::INVALID_GOAL);
    ROS_INFO("Not planning, goal pose invalid");
    return;
  }

  getTrajectory(start_pose.pose, goal_pose.pose, planner_type, start_pose.ignore_orientation, start_pose.tol, plan_req->path.path_segments[0].local_guidance_mode);
}

void Planner::preemptCB() {
  ROS_INFO("preemptCB");

  // set the action state to preempted
  get_path_action_srv_.setPreempted();
}

}  //  namespace waverider_chomp_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waverider_chomp_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  waverider_chomp_planner::Planner planner(nh, nh_private);
  ros::spin();

  return 0;
}