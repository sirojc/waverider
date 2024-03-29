#include <chomp_ros/chomp_planner.h>

#include <algorithm>
#include <cmath>

#include <iostream>

ChompPlanner::ChompPlanner(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nh_private_(nh_private) {
  // ros subcribers/ publisher/ service
  occupancy_pub_ = nh.advertise<wavemap_msgs::Map>("map", 10, true);
  esdf_pub_ = nh.advertise<wavemap_msgs::Map>("esdf", 10, true);
  trajectory_pub_ = nh.advertise<visualization_msgs::Marker>("trajectory", 10, true);
  // get_traj_service_ = nh.advertiseService("get_traj", &getTrajectoryCallback, this);
  
  // TODO: MAKE MAP UPDATE AND NOT FROM FILE, CHECK WAVERIDER HOW THEY DO IT
  // Load the occupancy map
  // wavemap::io::fileToMap(
  //     "/home/nicole/ros/git/wavemap/data/example_maps/newer_college_mine_5cm.wvmp", occupancy_map);
  wavemap::io::fileToMap("/home/nicole/ros/git/wavemap/data/anymal/map_lab.wvmp", occupancy_map_);
  CHECK_NOTNULL(occupancy_map_);

  // Publish the occupancy map
  wavemap_msgs::Map occupancy_map_msg;
  wavemap::convert::mapToRosMsg(*occupancy_map_, "odom", ros::Time::now(),
                                occupancy_map_msg);
  occupancy_pub_.publish(occupancy_map_msg);

  // Currently, only hashed wavelet octree maps are supported as input
  hashed_map_ = std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(occupancy_map_);
  if (!hashed_map_) {
    ROS_ERROR("Failed to cast occupancy map to HashedWaveletOctree.");
  }

  // Generate the ESDF
  constexpr float kOccupancyThreshold = -0.1f;
  constexpr float kMaxDistance = 2.f;
  esdf_ = std::make_shared<wavemap::HashedBlocks>(generateEsdf(*hashed_map_, kOccupancyThreshold, kMaxDistance));

  // Publish the ESDF
  wavemap_msgs::Map msg;
  wavemap::convert::mapToRosMsg(*esdf_, "odom", ros::Time::now(), msg);
  esdf_pub_.publish(msg);

  // Define the ESDF distance getter
  distance_getter_ = [this](const Eigen::Vector2d& position_d) {
    const wavemap::Point3D position(position_d[0], position_d[1], this->paddedRobotRadius_);
    if (wavemap::interpolateTrilinear(*this->occupancy_map_, position) < kOccupancyThreshold) {
        return wavemap::interpolateTrilinear(*this->esdf_, position);
    } else {
        return 0.f;
    }
  };

  // Initialize CHOMP
  params_.map_resolution =(*esdf_).getMinCellWidth();
  params_.w_collision = 10.0;
  params_.w_smooth = 0.1;
  params_.lambda = 100;  // 20.1
  params_.max_iter = 100;
  params_.epsilon = paddedRobotRadius_;
  params_.decrease_step_size = true;
  params_.D = 2; // dimensionalilty state (here: only x and y position)
  chomp_.setParameters(params_);
  chomp_.setDistanceFunction(distance_getter_);

  ROS_INFO("Ready for queries.");
}

Eigen::MatrixXd ChompPlanner::get_full_traj(Eigen::MatrixXd chomp_traj, double yaw_goal) {
  int n_elements = chomp_traj.rows();

  // todo: CHECK IF START IS PART OF TRAJECTORY OR NOT - HANDLE ACCORDINGLY
  
  // TODO: implement this better

  // calculate z for each step
  Eigen::MatrixXd z_pos = Eigen::MatrixXd::Ones(n_elements, 1) * paddedRobotRadius_;

  // calculate pitch, roll for each step
  Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(n_elements, 1);

  // calculate yaw for each step
  Eigen::MatrixXd yaw_angle = Eigen::MatrixXd::Zero(n_elements, 1);
  for (int i = 0; i < chomp_traj.rows() - 1; ++i) {
      double diff_x = chomp_traj(i + 1, 0) - chomp_traj(i, 0);
      double diff_y = chomp_traj(i + 1, 1) - chomp_traj(i, 1);
      
      yaw_angle(i) = atan2(diff_y, diff_x);
  }
  yaw_angle(n_elements - 1) = yaw_goal;

  Eigen::MatrixXd full_traj(n_elements, 6);
  full_traj << chomp_traj, z_pos, yaw_angle, zero, zero;

  CHECK_EQ(full_traj.cols(), 6);

  return full_traj;
}

bool ChompPlanner::getTrajectoryCallback(chomp_msgs::GetTraj::Request& req,
                             chomp_msgs::GetTraj::Response& res) {    
  // TODO: rosservice: first check if requested goal is collisionfree and then calculate path

  Eigen::Vector2d start(req.start.position.x, req.start.position.y);
  Eigen::Vector2d goal(req.goal.position.x, req.goal.position.y);
  
  // Publish the start and goal positions
  {
    // Set up the marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.id = 100;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = kRobotRadius_;
    marker.scale.y = kRobotRadius_;
    marker.scale.z = kRobotRadius_;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
    // Publish the start position
    marker.ns = "start";
    marker.color.b = 1.0;
    marker.pose.position.x = start[0];
    marker.pose.position.y = start[1];
    marker.pose.position.z = params_.epsilon; // TODO: DECIDE WHAT TO USE HERE
    trajectory_pub_.publish(marker);
    // Publish the goal position
    marker.ns = "goal";
    marker.color.b = 0.0;
    marker.color.g = 1.0;
    marker.pose.position.x = goal[0];
    marker.pose.position.y = goal[1];
    marker.pose.position.z = params_.epsilon;
    trajectory_pub_.publish(marker);
  }
  
  chomp::ChompTrajectory chomp_output;
  int N = 500;
  chomp_.solveProblem(start, goal, N, &chomp_output);

  // Check if the trajectory is collision free
  bool is_collision_free = true;
  for (int idx = 0; idx < chomp_output.trajectory.rows(); ++idx) {
    const auto position = chomp_output.trajectory.row(idx);
    const float esdf_distance = distance_getter_(position);
    if (esdf_distance <= kRobotRadius_) {
      is_collision_free = false;
      break;
    }
  }

  if (!is_collision_free) {
    LOG(INFO) << "Solution trajectory is NOT collision free";
    res.success = false;
    return true;
  } else {
    LOG(INFO) << "Solution trajectory is collision free";
    res.success = true;
  

    // get the full trajectory
    Eigen::MatrixXd full_traj = get_full_traj(chomp_output.trajectory, goal[2]);

        LOG(INFO) << "Publishing trajectory";
    visualization_msgs::Marker trajectory_msg;
    trajectory_msg.header.frame_id = "odom";
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
    for (int idx = 0; idx < chomp_output.trajectory.rows(); ++idx) {
      const auto position = chomp_output.trajectory.row(idx);
      auto& position_msg = trajectory_msg.points.emplace_back();
      position_msg.x = position.x();
      position_msg.y = position.y();
      position_msg.z = params_.epsilon;
    }
    trajectory_pub_.publish(trajectory_msg);  

    // Construct the response trajectory
    nav_msgs::Path path;

    // set response trajectory
    double prev_time = 0.0;
    double delta_t = 0.0;
    for (int i = 0; i < full_traj.rows(); ++i) {
      geometry_msgs::PoseStamped traj_step;
      traj_step.header.frame_id = "odom";

      // time - TODO: THIS CORRESPONDS TO THE TIME WHEN TRAJ. SHOULD BE EXECUTED RIGHT?
      if (i > 0) {
        double diff_pos = (full_traj.row(i + 1).head(2) - full_traj.row(i).head(2)).norm();
        double diff_angle = fmod(full_traj(i + 1, 3) - full_traj(i, 3), 2 * M_PI);

        delta_t = std::max(diff_pos / des_lin_velocity_, diff_angle / des_ang_velocity_);
      }
      prev_time += delta_t;
      traj_step.header.stamp = ros::Time(prev_time);

      // position
      traj_step.pose.position.x = full_traj(i, 0);
      traj_step.pose.position.y = full_traj(i, 1);
      traj_step.pose.position.z = full_traj(i, 2);

      // get quaternion from euler angles
      Eigen::Quaterniond q_curr;
      q_curr = Eigen::AngleAxisd(full_traj(i, 3), Eigen::Vector3d::UnitX())
               * Eigen::AngleAxisd(full_traj(i, 4), Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(full_traj(i, 5), Eigen::Vector3d::UnitZ());

      traj_step.pose.orientation.x = q_curr.x();  
      traj_step.pose.orientation.y = q_curr.y();
      traj_step.pose.orientation.z = q_curr.z();
      traj_step.pose.orientation.w = q_curr.w();

      path.poses.push_back(traj_step);
    }

    res.trajectory = path;
  }

  return true;
}

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
  ChompPlanner chomp_planner(nh, nh_private);
  ros::ServiceServer get_traj_service = nh.advertiseService("get_traj", &ChompPlanner::getTrajectoryCallback, &chomp_planner);

  // Spin ROS
  ros::spin();

  return 0;
}