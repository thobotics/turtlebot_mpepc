/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Austin Hendrix
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Austin Hendrix
*********************************************************************/

#include <mpepc_local_planner/mpepc_planner_ros.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <mpepc_global_planner/GetNavCost.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(mpepc_local_planner::MpepcPlannerROS, nav_core::BaseLocalPlanner)

namespace mpepc_local_planner {

#define RATE_FACTOR 0.2
#define DEFAULT_LOOP_RATE 10

#define RESULT_BEGIN 1
#define RESULT_SUCCESS 2
#define RESULT_CANCEL 3

// Trajectory Model Params
#define K_1 1.2           // 2
#define K_2 3             // 8
#define BETA 0.4          // 0.5
#define LAMBDA 2          // 3
#define R_THRESH 0.05
#define V_MAX 0.3         // 0.3
#define V_MIN 0.0

// Trajectory Optimization Params
#define TIME_HORIZON 5.0
#define DELTA_SIM_TIME 0.2
#define SAFETY_ZONE 0.225
#define WAYPOINT_THRESH 1.75

// Cost function params
static const double C1 = 0.05;
static const double C2 = 2.5;
static const double C3 = 0.05;        // 0.15
static const double C4 = 0.05;        // 0.2 //turn
static const double PHI_COL = 1.0;   // 0.4
static const double SIGMA = 0.2;    // 0.10

static const double PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
static const double TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;
static const double minusPI= -3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;

char* MpepcPlannerROS::cost_translation_table_ = NULL;

  MpepcPlannerROS::MpepcPlannerROS() : initialized_(false),
    goal_reached_(false) {

  }

  void MpepcPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {

			ros::NodeHandle private_nh("~/" + name);

			l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
			g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
			tf_ = tf;
			costmap_ros_ = costmap_ros;

			// make sure to update the costmap we'll use for this cycle
			// costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

			// TODO: Is using odom ??
			// Default use for compute called in isGoalReached and compute cmd_vel
			std::string odom_topic;
			private_nh.param<std::string>("odom_topic", odom_topic, "odom");
			odom_helper_.setOdomTopic( odom_topic );

			// For compute obstacle tree
			// NOTE: Copy from costmap_2d_publisher.
			if (cost_translation_table_ == NULL)
			{
				cost_translation_table_ = new char[256];

				// special values:
				cost_translation_table_[0] = 0;  // NO obstacle
				cost_translation_table_[253] = 99;  // INSCRIBED obstacle
				cost_translation_table_[254] = 100;  // LETHAL obstacle
				cost_translation_table_[255] = -1;  // UNKNOWN

				// regular cost values scale the range 1 to 252 (inclusive) to fit
				// into 1 to 98 (inclusive).
				for (int i = 1; i < 253; i++)
				{
				  cost_translation_table_[ i ] = char(1 + (97 * (i - 1)) / 251);
				}
			}

			navfn_cost_ = private_nh.serviceClient<mpepc_global_planner::GetNavCost>("/move_base/NavfnROSExt/nav_cost");

			initialized_ = true;
		}
		else{
		  ROS_WARN_NAMED("MPEPCPlanner", "This planner has already been initialized, doing nothing.");
		}
  }

  bool MpepcPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if(!initialized_)
		  {
		    ROS_ERROR("MPEPCPlanner has not been initialized, please call initialize() before using this planner");
		    return false;
		  }

		  // store the global plan
		  global_plan_.clear();
		  global_plan_ = orig_global_plan;

		  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
		  // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

		  // reset goal_reached_ flag
		  goal_reached_ = false;

		  return true;
  }

  bool MpepcPlannerROS::isGoalReached() {
    if (! isInitialized()) {
		  ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		  return false;
		}
		// TODO:
		//  probably use some sort of goal tolerance parameters here
		return goal_reached_;
  }


  MpepcPlannerROS::~MpepcPlannerROS(){  }


  bool MpepcPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // if we don't have a plan, what are we doing here???
    // check if plugin initialized
		if(!initialized_)
		{
			ROS_ERROR("MPEPCPlanner has not been initialized, please call initialize() before using this planner");
			return false;
		}

		cmd_vel.linear.x = 0;
		cmd_vel.linear.y = 0;
		cmd_vel.angular.z = 0;
		goal_reached_ = false;

		/**
		 * TODO: Clear below and Apply real local plan
		 * ---------------- START TESTING ----------------
		 */

		// Get robot pose
		geometry_msgs::Pose currentPose = getCurrentRobotPose();

		double cost = getGlobalPlannerCost(currentPose);

		// Must update Obstacle Tree before calculate minimum distance to obstacle
		updateObstacleTree(costmap_ros_->getCostmap());
		double obstacle_heading = 0.0;
		double minDist = min_distance_to_obstacle(currentPose, &obstacle_heading);

		ROS_INFO("Min distance of %f, %f: %f", currentPose.position.x, currentPose.position.y, minDist);

		/*
		 * -------------------- END --------------------
		 */
		return true;
  }

  geometry_msgs::Pose MpepcPlannerROS::getCurrentRobotPose(){
	// Get robot pose from local cost_map
	tf::Stamped<tf::Pose> robot_pose;
	costmap_ros_->getRobotPose(robot_pose);

	geometry_msgs::PoseStamped result;
	tf::poseStampedTFToMsg(robot_pose, result);

	return result.pose;
  }

  double MpepcPlannerROS::getGlobalPlannerCost(geometry_msgs::Pose local_pose){
	// Transform to global_pose
	geometry_msgs::PoseStamped local_pose_stamp;
	local_pose_stamp.header.frame_id = costmap_ros_->getGlobalFrameID();
	// This is important!!!
	// It make transformPose to lookup the latest available transform
	local_pose_stamp.header.stamp = ros::Time(0);
	local_pose_stamp.pose = local_pose;

	geometry_msgs::PoseStamped global_pose_stamp;
	try{
		tf_->waitForTransform("/map", costmap_ros_->getGlobalFrameID(), ros::Time(0), ros::Duration(10.0));
		tf_->transformPose("/map", local_pose_stamp, global_pose_stamp);
	}catch (tf::TransformException & ex){
		ROS_ERROR("Transform exception : %s", ex.what());
	}

	geometry_msgs::Point currentPoint;
	currentPoint = global_pose_stamp.pose.position;

	// Service request
	mpepc_global_planner::GetNavCost service;
	service.request.world_point = currentPoint;

	if (navfn_cost_.call(service))
	{
		ROS_INFO("Response cost at %f %f : %f",
				currentPoint.x, currentPoint.y,
				(double)service.response.cost);
		return (double)service.response.cost;
	}
	else
	{
		ROS_ERROR("Failed to call service");
	}

	return -1;
  }

  void MpepcPlannerROS::updateObstacleTree(costmap_2d::Costmap2D *costmap){

	// Create occupancy grid message
	// Copy from costmap_2d_publisher.cpp
	nav_msgs::OccupancyGrid::_info_type::_resolution_type resolution = costmap->getResolution();
	nav_msgs::OccupancyGrid::_info_type::_width_type width = costmap->getSizeInCellsX();
	nav_msgs::OccupancyGrid::_info_type::_height_type height = costmap->getSizeInCellsY();
	double wx, wy;
	costmap->mapToWorld(0, 0, wx, wy);
	nav_msgs::OccupancyGrid::_info_type::_origin_type::_position_type::_x_type x = wx - resolution / 2;
	nav_msgs::OccupancyGrid::_info_type::_origin_type::_position_type::_y_type y = wy - resolution / 2;
	nav_msgs::OccupancyGrid::_info_type::_origin_type::_position_type::_z_type z = 0.0;
	nav_msgs::OccupancyGrid::_info_type::_origin_type::_orientation_type::_w_type w = 1.0;

	nav_msgs::OccupancyGrid::_data_type grid_data;
	grid_data.resize(width * height);

	unsigned char* charData = costmap->getCharMap();
	for (unsigned int i = 0; i < grid_data.size(); i++)
	{
		grid_data[i] = cost_translation_table_[ charData[ i ]];
	}

	// Copy from costmap_translator
	nav_msgs::GridCells obstacles;
	obstacles.cell_width = resolution;
	obstacles.cell_height = resolution;
	for (unsigned int i = 0 ; i < height; ++i)
	{
		for(unsigned int j = 0; j < width; ++j)
		{
		  if(grid_data[i*height+j] == 100)
		  {
			geometry_msgs::Point obstacle_coordinates;
			obstacle_coordinates.x = (j * obstacles.cell_height) + x + (resolution/2.0);
			obstacle_coordinates.y = (i * obstacles.cell_width) + y + (resolution/2.0);
			obstacle_coordinates.z = 0;
			obstacles.cells.push_back(obstacle_coordinates);
		  }
		}
	}

	// Copy from nav_cb
	cost_map = obstacles;

	if(cost_map.cells.size() > 0)
	{
		delete obs_tree;
		delete data;
		data = new flann::Matrix<float>(new float[obstacles.cells.size()*2], obstacles.cells.size(), 2);

		for (size_t i = 0; i < data->rows; ++i)
		{
		  for (size_t j = 0; j < data->cols; ++j)
		  {
			if (j == 0)
			  (*data)[i][j] = cost_map.cells[i].x;
			else
			  (*data)[i][j] = cost_map.cells[i].y;
		  }
		}
		// Obstacle index for fast nearest neighbor search
		obs_tree = new flann::Index<flann::L2<float> >(*data, flann::KDTreeIndexParams(4));
		obs_tree->buildIndex();
	}
  }

  vector<MinDistResult> MpepcPlannerROS::find_points_within_threshold(Point newPoint, double threshold)
  {
    vector<MinDistResult> results;

    flann::Matrix<float> query(new float[2], 1, 2);
    query[0][0] = newPoint.a;
    query[0][1] = newPoint.b;

    std::vector< std::vector<int> > indices;
    std::vector< std::vector<float> > dists;

    flann::SearchParams params;
    params.checks = 128;
    params.max_neighbors = -1;
    params.sorted = true;
    // ROS_INFO("Do search");
    {
      boost::mutex::scoped_lock lock(cost_map_mutex_);
      obs_tree->radiusSearch(query, indices, dists, threshold, params);

      // ROS_INFO("Finished search");
      for (int i = 0; i < indices[0].size(); i++)
      {
        MinDistResult result;
        result.p = Point((*data)[indices[0][i]][0], (*data)[indices[0][i]][1]);
        result.dist = static_cast<double>(dists[0][i]);
        results.push_back(result);
      }
    }

    delete[] query.ptr();
    indices.clear();
    dists.clear();

    return results;
  }

  MinDistResult MpepcPlannerROS::find_nearest_neighbor(Point queryPoint)
  {
    MinDistResult results;

    flann::Matrix<float> query(new float[2], 1, 2);
    query[0][0] = queryPoint.a;
    query[0][1] = queryPoint.b;

    std::vector< std::vector<int> > indices;
    std::vector< std::vector<float> > dists;

    flann::SearchParams params;
    params.checks = 128;
    params.sorted = true;

    {
      boost::mutex::scoped_lock lock(cost_map_mutex_);
      obs_tree->knnSearch(query, indices, dists, 1, params);
      results.p = Point((*data)[indices[0][0]][0], (*data)[indices[0][0]][1]);
      results.dist = static_cast<double>(dists[0][0]);
    }

    MinDistResult tempResults;
    tempResults.p = Point(cost_map.cells[indices[0][0]].x, cost_map.cells[indices[0][0]].y);

    delete[] query.ptr();
    indices.clear();
    dists.clear();

    return results;
  }

  double MpepcPlannerROS::min_distance_to_obstacle(geometry_msgs::Pose local_current_pose, double *heading)
  {
    // ROS_INFO("In minDist Function");
    Point global(local_current_pose.position.x, local_current_pose.position.y);
    MinDistResult nn_graph_point = find_nearest_neighbor(global);

    double minDist = 100000;
    double head = 0;

    double SOME_THRESH = 1.5;

    if(nn_graph_point.dist < SOME_THRESH)
    {
      int min_i = 0;
      vector<MinDistResult> distResult;
      distResult = find_points_within_threshold(global, 1.1*SOME_THRESH);

      //ROS_INFO("Loop through %d points from radius search", distResult.size());
      for (unsigned int i = 0 ; i < distResult.size() && minDist > 0; i++)
      {
        double dist = distance(local_current_pose.position.x, local_current_pose.position.y, cost_map.cells[i].x, cost_map.cells[i].y);
        if (dist < minDist)
        {
          minDist = dist;
          min_i = i;
        }
      }

      // ROS_INFO("Calculate heading");
      head = tf::getYaw(local_current_pose.orientation) - atan2(cost_map.cells[min_i].y - local_current_pose.position.y, cost_map.cells[min_i].x - local_current_pose.position.x);
      head = mod(head + PI, TWO_PI) - PI;
      //ROS_INFO("Got nearest radius neighbor, poly dist: %f", minDist);
    }
    else
    {
      minDist = distance(local_current_pose.position.x, local_current_pose.position.y, nn_graph_point.p.a, nn_graph_point.p.b);
      //ROS_INFO("Got nearest neighbor, poly dist: %f", minDist);
    }

    *heading = head;

    return minDist;
  }

  double MpepcPlannerROS::sim_trajectory(double r, double delta, double theta, double vMax, double time_horizon){
	// Get robot pose from local cost map
	// TODO: Change this to getCurrentRobotPose.
	// 1. Why need to change this? Ans: May be it help for faster reading current pose,
	// but not sure because getCurrentRobotPose use transform instead of callback in Odom.
	// 2. Why not change it now? Ans: Because it may need to change other file: control_law, ...

	nav_msgs::Odometry sim_pose;
	odom_helper_.getOdom(sim_pose);

	EgoPolar sim_goal;
	sim_goal.r = r;
	sim_goal.delta = delta;
	sim_goal.theta = theta;

	geometry_msgs::Pose current_goal = cl->convert_from_egopolar(sim_pose, sim_goal);

	double SIGMA_DENOM = pow(SIGMA, 2);

	double sim_clock = 0.0;

	geometry_msgs::Twist sim_cmd_vel;
	double current_yaw = tf::getYaw(sim_pose.pose.pose.orientation);
	geometry_msgs::Point collisionPoint;
	bool collision_detected = false;

	double expected_progress = 0.0;
	double expected_action = 0.0;
	double expected_collision = 0.0;

	double nav_fn_t0 = 0;
	double nav_fn_t1 = 0;
	double collision_prob = 0.0;
	double survivability = 1.0;
	double obstacle_heading = 0.0;

	while (sim_clock < time_horizon)
	{
	  // Get Velocity Commands
	  sim_cmd_vel = cl->get_velocity_command(sim_goal, vMax);

	  // get navigation function at orig pose
	  nav_fn_t0 = getGlobalPlannerCost(sim_pose.pose.pose);

	  // Update pose
	  current_yaw = current_yaw + (sim_cmd_vel.angular.z * DELTA_SIM_TIME);
	  sim_pose.pose.pose.position.x = sim_pose.pose.pose.position.x + (sim_cmd_vel.linear.x * DELTA_SIM_TIME * cos(current_yaw));
	  sim_pose.pose.pose.position.y = sim_pose.pose.pose.position.y + (sim_cmd_vel.linear.x * DELTA_SIM_TIME * sin(current_yaw));
	  sim_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw);

	  // Get navigation function at new pose
	  nav_fn_t1 = getGlobalPlannerCost(sim_pose.pose.pose);

	  double minDist = min_distance_to_obstacle(sim_pose.pose.pose, &obstacle_heading);

	  if (minDist <= SAFETY_ZONE)
	  {
		// ROS_INFO("Collision Detected");
		collision_detected = true;
	  }

	  // Get collision probability
	  if (!collision_detected)
	  {
		collision_prob = exp(-1*pow(minDist, 2)/SIGMA_DENOM);  // sigma^2
	  }
	  else
	  {
		collision_prob = 1;
	  }

	  // Get survivability
	  survivability = survivability*(1 - collision_prob);

	  expected_collision = expected_collision + ((1-survivability) * C2);

	  // Get progress cost
	  expected_progress = expected_progress + (survivability * (nav_fn_t1 - nav_fn_t0));

	  // Get action cost
	  expected_action = expected_action + (C3 * pow(sim_cmd_vel.linear.x, 2) + C4 * pow(sim_cmd_vel.angular.z, 2))*DELTA_SIM_TIME;

	  // Calculate new EgoPolar coords for goal
	  sim_goal = cl->convert_to_egopolar(sim_pose, current_goal);

	  sim_clock = sim_clock + DELTA_SIM_TIME;
	}

	// Update with angle heuristic - weighted difference between final pose and gradient of navigation function
	double gradient_angle = getGlobalPlannerCost(sim_pose.pose.pose);

	expected_progress = expected_progress + C1 * abs(tf::getYaw(sim_pose.pose.pose.orientation) - gradient_angle);

	++trajectory_count;

	// SUM collision cost, progress cost, action cost
	return (expected_collision + expected_progress + expected_action);
  }

};
