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

#include <ctime>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(mpepc_local_planner::MpepcPlannerROS, nav_core::BaseLocalPlanner)

namespace mpepc_local_planner {

  char* MpepcPlannerROS::cost_translation_table_ = NULL;

  MpepcPlannerROS::MpepcPlannerROS() : initialized_(false),
		  odom_helper_("odom"), goal_reached_(false), isPlanThreadStart_(false),
  	  	  run_planner_(true)
  {
	  inter_goal_coords_.r = -1;
  }

  void MpepcPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
	  	  if (! isInitialized()) {

			ros::NodeHandle private_nh("~/" + name);

			l_plan_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("mpepc_local_plan", 1);
			tf_ = tf;
			costmap_ros_ = costmap_ros;

			// make sure to update the costmap we'll use for this cycle
			costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

			// TODO: Is using odom ??
			// Default use for compute called in isGoalReached and compute cmd_vel
			std::string odom_topic;
			private_nh.param<std::string>("odom_topic", odom_topic, "odom");
			odom_helper_.setOdomTopic( odom_topic );

			// Set parameters
			private_nh.param<double>("cost_theta", C1, 0.05);
			private_nh.param<double>("cost_collision", C2, 1.0);
			private_nh.param<double>("cost_v", C3, 0.05);
			private_nh.param<double>("cost_w", C4, 0.05);
			private_nh.param<double>("cost_collision_sigma", SIGMA, 0.2);
			private_nh.param<double>("goal_dist_tol", GOAL_DIST_UPDATE_THRESH,  0.15);
			private_nh.param<double>("goal_angle_tol", GOAL_ANGLE_UPDATE_THRESH, 0.1);
			private_nh.param<double>("goal_dist_same", GOAL_DIST_ID_THRESH, 0.1);
			private_nh.param<double>("goal_angle_same", GOAL_ANGLE_ID_THRESH, 0.3);

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

			// FOR mpepc_plan
			std::string potential_topic;
			private_nh.param<std::string>("potential_topic", potential_topic, "/move_base/NavfnROSExt/potential");
			//navfn_cost_sub_ = private_nh.subscribe<nav_msgs::OccupancyGrid> ("/move_base/Srl_global_planner/rrt_potential_collision_free", 1, &MpepcPlannerROS::nav_cost_cb, this);
			navfn_cost_sub_ = private_nh.subscribe<nav_msgs::OccupancyGrid> (potential_topic, 1, &MpepcPlannerROS::nav_cost_cb, this);

			dsrv_ = new dynamic_reconfigure::Server<MPEPCPlannerConfig>(private_nh);
		    dynamic_reconfigure::Server<MPEPCPlannerConfig>::CallbackType cb = boost::bind(&MpepcPlannerROS::reconfigureCB, this, _1, _2);
			dsrv_->setCallback(cb);

			// Initialize Motion Model
			settings_.m_K1 = K_1;
			settings_.m_K2 = K_2;
			settings_.m_BETA = BETA;
			settings_.m_LAMBDA = LAMBDA;
			settings_.m_R_THRESH = R_THRESH;
			settings_.m_V_MAX = V_MAX;
			settings_.m_V_MIN = W_TURN;//V_MIN;
			cl = new ControlLaw(&settings_);

			initialized_ = true;
		}
		else{
		  ROS_WARN_NAMED("MPEPCPlanner", "This planner has already been initialized, doing nothing.");
		}
  }

  void MpepcPlannerROS::reconfigureCB(MPEPCPlannerConfig &config, uint32_t level){
	  K_1 = config.cl_K_1; //1.2           // 2
	  K_2 = config.cl_K_2; //3             // 8
	  BETA = config.cl_BETA; //0.4          // 0.5
	  LAMBDA = config.cl_LAMBDA; //2          // 3
	  R_THRESH = config.cl_R_THRESH; //0.05
	  V_MAX = config.cl_V_MAX; //0.5         // 0.3
	  V_MIN = config.cl_V_MIN; //0.0
	  W_TURN = config.cl_W_TURN;//0.2

	  // Initialize Motion Model
	  settings_.m_K1 = K_1;
	  settings_.m_K2 = K_2;
	  settings_.m_BETA = BETA;
	  settings_.m_LAMBDA = LAMBDA;
	  settings_.m_R_THRESH = R_THRESH;
	  settings_.m_V_MAX = V_MAX;
	  settings_.m_V_MIN = W_TURN;//V_MIN;

	  // Trajectory Optimization Params
	  TIME_HORIZON = config.TIME_HORIZON; 		//5.0
	  DELTA_SIM_TIME = config.DELTA_SIM_TIME; 	//0.2
	  SAFETY_ZONE = config.SAFETY_ZONE; 		//0.225
	  WAYPOINT_THRESH = config.WAYPOINT_THRESH; 	//1.75
  }

  void MpepcPlannerROS::planThread(){
	ROS_INFO("Starting local planner thread...");

	ros::NodeHandle n;
	ros::Rate rate_obj(5.0); // Maximum 5Hz each loop

	boost::unique_lock<boost::mutex> lock(planner_mutex_);
	while(n.ok()){
		while (!run_planner_){
			ROS_DEBUG("Planner thread is suspending");
			planner_cond_.wait(lock);
		}
		lock.unlock();
		EgoGoal new_coords;
		// Must update Obstacle Tree before using optimization technique
		updateObstacleTree(costmap_ros_->getCostmap());
		sim_current_pose_ = getCurrentRobotPose();
		find_intermediate_goal_params(&new_coords);

		{
			boost::mutex::scoped_lock lock(inter_goal_mutex_);
			inter_goal_coords_.r = new_coords.r;
			inter_goal_coords_.delta = new_coords.delta;
			inter_goal_coords_.theta = new_coords.theta;
			inter_goal_vMax_ = new_coords.vMax;
			inter_goal_k1_ = new_coords.k1;
			inter_goal_k2_ = new_coords.k2;
		}

		l_plan_pub_.publish(get_trajectory_viz(new_coords));
		rate_obj.sleep();

		// lock to next
		lock.lock();
	}
  }

  void MpepcPlannerROS::nav_cost_cb(const nav_msgs::OccupancyGrid::ConstPtr& nav_cost)
  {
	  global_potarr_ = nav_cost->data;
	  global_width_ = nav_cost->info.width;
	  global_height_ = nav_cost->info.height;
	  origin_x_ = nav_cost->info.origin.position.x;
	  origin_y_ = nav_cost->info.origin.position.y;
	  resolution_ = nav_cost->info.resolution;
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

	// Get goal pose. Note that plan from start to goal
	geometry_msgs::PoseStamped global_goal_pose = global_plan_.back();
	if(!same_global_goal(global_goal_pose)){
		global_goal_pose_stamped_ = global_goal_pose;

		global_goal_pose.header.frame_id = "/map";
		// This is important!!!
		// It make transformPose to lookup the latest available transform
		global_goal_pose.header.stamp = ros::Time(0);
		geometry_msgs::PoseStamped local_pose_stamp;
		tf::StampedTransform transform;
		try{
			tf_->waitForTransform(costmap_ros_->getGlobalFrameID(), "/map", ros::Time(0), ros::Duration(10.0));
			tf_->transformPose(costmap_ros_->getGlobalFrameID(), global_goal_pose, local_pose_stamp);
		}catch (tf::TransformException & ex){
			ROS_ERROR("Transform exception 222 : %s", ex.what());
		}

		local_goal_pose_ = local_pose_stamp.pose;

		/*ROS_INFO("Local goal update: x %f, y %f; ",
			local_goal_pose_.position.x, global_goal_pose.pose.position.y);
		ROS_INFO("Orientation: w %f, x %f, y %f, z %f",
			local_goal_pose_.orientation.w,
			local_goal_pose_.orientation.x,
			local_goal_pose_.orientation.y,
			local_goal_pose_.orientation.z);*/

		// we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
		// the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

		// reset goal_reached_ flag
		goal_reached_ = false;
		boost::unique_lock<boost::mutex> lock(planner_mutex_);
		run_planner_ = true;
		planner_cond_.notify_one();
		lock.unlock();
	}


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

  MpepcPlannerROS::~MpepcPlannerROS(){
	  if(!isPlanThreadStart_)
	  {
		  planner_thread_->interrupt();
		  planner_thread_->join();

		  delete planner_thread_;
	  }
  }

  bool MpepcPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // if we don't have a plan, what are we doing here???
    // check if plugin initialized
	if(!initialized_)
	{
		ROS_ERROR("MPEPCPlanner has not been initialized, please call initialize() before using this planner");
		return false;
	}

	// TODO: Uncomment this
	if(!isPlanThreadStart_)
	{
		//set up the local planner's thread
		planner_thread_ = new boost::thread(boost::bind(&MpepcPlannerROS::planThread, this));
		isPlanThreadStart_ = true;
	}

	// Default
	cmd_vel.linear.x = 0;
	cmd_vel.linear.y = 0;
	cmd_vel.angular.z = 0;

	// TODO: Uncomment this
	geometry_msgs::Pose current_pose = getCurrentRobotPose();

	EgoPolar global_goal_coords;
	global_goal_coords = cl->convert_to_egopolar(current_pose, local_goal_pose_);

	ROS_DEBUG("Distance to goal: %f", global_goal_coords.r);
	if (global_goal_coords.r <= GOAL_DIST_UPDATE_THRESH)
	{
		// Stop planner. Now just steering to right orientation
		boost::unique_lock<boost::mutex> lock(planner_mutex_);
		run_planner_ = false;
		lock.unlock();

		double angle_error = tf::getYaw(current_pose.orientation) - tf::getYaw(local_goal_pose_.orientation);
		angle_error = cl->wrap_pos_neg_pi(angle_error);
		ROS_DEBUG("Angle error: %f", angle_error);

		if (fabs(angle_error) > GOAL_ANGLE_UPDATE_THRESH)
		{
		  cmd_vel.linear.x = 0;
		  if (angle_error > 0)
			cmd_vel.angular.z = -4 * settings_.m_V_MIN;
		  else
			cmd_vel.angular.z = 4 * settings_.m_V_MIN;
		}
		else
		{
		  ROS_INFO("[MPEPC] Completed normal trajectory following");
		  goal_reached_ = true;
		  cmd_vel.linear.x = 0;
		  cmd_vel.angular.z = 0;
		}
	}
	else
	{
	  {
		boost::mutex::scoped_lock lock(inter_goal_mutex_);
		if(inter_goal_coords_.r != -1){
			geometry_msgs::Pose inter_goal_pose = cl->convert_from_egopolar(current_pose, inter_goal_coords_);
			cmd_vel = cl->get_velocity_command(current_pose, inter_goal_pose, inter_goal_k1_, inter_goal_k2_, inter_goal_vMax_);
		}
	  }
	}



	/*
	 * -------------------- END --------------------
	 */
	return true;
  }

  bool MpepcPlannerROS::same_global_goal(geometry_msgs::PoseStamped new_goal){
	bool isSameGoal = false;

	float goal_dist = sqrt(((global_goal_pose_stamped_.pose.position.x - new_goal.pose.position.x)
            * (global_goal_pose_stamped_.pose.position.x - new_goal.pose.position.x)) +
             ((global_goal_pose_stamped_.pose.position.y - new_goal.pose.position.y)
            * (global_goal_pose_stamped_.pose.position.y - new_goal.pose.position.y)));
	float goal_angle_dist = fabs(tf::getYaw(global_goal_pose_stamped_.pose.orientation) - tf::getYaw(new_goal.pose.orientation));
	if (goal_dist < GOAL_DIST_ID_THRESH)
	{
		if (goal_angle_dist < GOAL_ANGLE_ID_THRESH)
		{
		  isSameGoal = true;
		}
	}

	return isSameGoal;
  }

  geometry_msgs::Pose MpepcPlannerROS::getCurrentRobotPose(){
	// Get robot pose from local cost_map
	tf::Stamped<tf::Pose> robot_pose;
	costmap_ros_->getRobotPose(robot_pose);

	geometry_msgs::PoseStamped result;
	tf::poseStampedTFToMsg(robot_pose, result);

	return result.pose;
  }

  geometry_msgs::PoseArray MpepcPlannerROS::get_trajectory_viz(EgoGoal new_coords){
	geometry_msgs::PoseArray viz_plan;
	viz_plan.header.stamp = ros::Time::now();
	viz_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
	viz_plan.poses.resize(1);

	geometry_msgs::Pose sim_pose = getCurrentRobotPose();

	EgoPolar sim_goal;
	sim_goal.r = new_coords.r;
	sim_goal.delta = new_coords.delta;
	sim_goal.theta = new_coords.theta;

	geometry_msgs::Pose current_goal = cl->convert_from_egopolar(sim_pose, sim_goal);

	double sim_clock = 0.0;

	geometry_msgs::Twist sim_cmd_vel;
	double current_yaw = tf::getYaw(sim_pose.orientation);

	while (sim_clock < TIME_HORIZON)
	{
	  sim_cmd_vel = cl->get_velocity_command(sim_goal, new_coords.k1, new_coords.k2, new_coords.vMax);

	  // Update pose
	  current_yaw = current_yaw + (sim_cmd_vel.angular.z * DELTA_SIM_TIME);
	  sim_pose.position.x = sim_pose.position.x + (sim_cmd_vel.linear.x * DELTA_SIM_TIME * cos(current_yaw));
	  sim_pose.position.y = sim_pose.position.y + (sim_cmd_vel.linear.x * DELTA_SIM_TIME * sin(current_yaw));
	  sim_pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw);
	  viz_plan.poses.push_back(sim_pose);

	  sim_goal = cl->convert_to_egopolar(sim_pose, current_goal);

	  sim_clock = sim_clock + DELTA_SIM_TIME;
	}

	return viz_plan;
  }

  geometry_msgs::Point MpepcPlannerROS::transformOdomToMap(geometry_msgs::Pose local_pose){
	    // Transform to global_pose
	  	//clock_t begin_time = clock();
	  	geometry_msgs::PointStamped local_point;
		local_point.header.frame_id = costmap_ros_->getGlobalFrameID();
		// This is important!!!
		// It make transformPose to lookup the latest available transform
		local_point.header.stamp = ros::Time(0);
		local_point.point = local_pose.position;
		geometry_msgs::PointStamped global_point_stamp;
		try{
			tf_->waitForTransform("/map", costmap_ros_->getGlobalFrameID(), ros::Time(0), ros::Duration(10.0));
			tf_->transformPoint("/map", local_point, global_point_stamp);
		}catch (tf::TransformException & ex){
			ROS_ERROR("Transform exception 111 : %s", ex.what());
		}
		//ROS_INFO("Transform plan take %f", float( clock() - begin_time ) /  CLOCKS_PER_SEC);
		return global_point_stamp.point;
  }

  double MpepcPlannerROS::getGlobalPointPotential(geometry_msgs::Pose local_pose){
	//clock_t begin_time = clock();
	geometry_msgs::Point currentPoint = transformOdomToMap(local_pose);

	// TODO: Interpolate here
	bool flag = false;
	unsigned int mx, my;
	// Copy from worldToMap from Costmap2D
	if (currentPoint.x < origin_x_ || currentPoint.y < origin_y_)
	  flag = false;
	mx = (int)((currentPoint.x - origin_x_) / resolution_);
	my = (int)((currentPoint.y - origin_y_) / resolution_);
	if (mx < global_width_ && my < global_height_)
	  flag = true;

	//ROS_INFO("Get point potential take %f", float( clock() - begin_time ) /  CLOCKS_PER_SEC);

	if(!flag)
	  return DBL_MAX;

	unsigned int index = my * global_width_ + mx;

	if(global_potarr_[index] == -1){
		return POT_HIGH;
	}else
		return global_potarr_[index];
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
	if(cost_map.cells.size() == 0){
		return 100000;
	}
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

  double MpepcPlannerROS::sim_trajectory(double r, double delta, double theta, double vMax){
	// Get robot pose from local cost map
	// DONE: Change this to getCurrentRobotPose.
	// 1. Why need to change this? Ans: May be it help for faster reading current pose,
	// but not sure because getCurrentRobotPose use transform instead of callback in Odom.
	// 2. Why not change it now? Ans: Because it may need to change other file: control_law, ...

	double time_horizon = TIME_HORIZON;
	geometry_msgs::Pose sim_pose = sim_current_pose_;

	EgoPolar sim_goal;
	sim_goal.r = r;
	sim_goal.delta = delta;
	sim_goal.theta = theta;

	geometry_msgs::Pose current_goal = cl->convert_from_egopolar(sim_pose, sim_goal);

	double SIGMA_DENOM = pow(SIGMA, 2);

	double sim_clock = 0.0;

	geometry_msgs::Twist sim_cmd_vel;
	double current_yaw = tf::getYaw(sim_pose.orientation);
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
	  nav_fn_t0 = getGlobalPointPotential(sim_pose);

	  // Update pose
	  current_yaw = current_yaw + (sim_cmd_vel.angular.z * DELTA_SIM_TIME);
	  sim_pose.position.x = sim_pose.position.x + (sim_cmd_vel.linear.x * DELTA_SIM_TIME * cos(current_yaw));
	  sim_pose.position.y = sim_pose.position.y + (sim_cmd_vel.linear.x * DELTA_SIM_TIME * sin(current_yaw));
	  sim_pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw);

	  // Get navigation function at new pose
	  nav_fn_t1 = getGlobalPointPotential(sim_pose);

	  double minDist = min_distance_to_obstacle(sim_pose, &obstacle_heading);

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
	double gradient_angle = getGlobalPointPotential(sim_pose);

	expected_progress = expected_progress + C1 * abs(tf::getYaw(sim_pose.orientation) - gradient_angle);

	++trajectory_count;

	// SUM collision cost, progress cost, action cost
	return (expected_collision + expected_progress + expected_action);
  }

  /**
   * Call back function for nlopt objective function
   * Note: this function is not belong to class
   */
  double score_trajectory(const std::vector<double> &x, std::vector<double> &grad, void* f_data){
	MpepcPlannerROS * planner = static_cast<MpepcPlannerROS *>(f_data);
	return planner->sim_trajectory(x[0], x[1], x[2], x[3]);
  }

  void MpepcPlannerROS::find_intermediate_goal_params(EgoGoal *next_step)
  {
    trajectory_count = 0;

    int max_iter = 250;  // 30
    nlopt::opt opt = nlopt::opt(nlopt::GN_DIRECT_NOSCAL, 4);
    opt.set_min_objective(score_trajectory, this);
    opt.set_xtol_rel(0.0001);
    std::vector<double> lb;
    std::vector<double> rb;
    lb.push_back(0);
    lb.push_back(-1.8);
    lb.push_back(-1.8);
    lb.push_back(V_MIN);
    rb.push_back(5.0);
    rb.push_back(1.8);
    rb.push_back(1.8);
    rb.push_back(V_MAX);
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(rb);
    opt.set_maxeval(max_iter);

    std::vector<double> k(4);
    k[0] = 0.0;
    k[1] = 0.0;
    k[2] = 0.0;
    k[3] = 0.0;
    double minf;

    opt.optimize(k, minf);

    ROS_DEBUG("Global Optimization - Trajectories evaluated: %d", trajectory_count);
    trajectory_count = 0;

    max_iter = 75;  // 200
    nlopt::opt opt2 = nlopt::opt(nlopt::LN_BOBYQA, 4);
    opt2.set_min_objective(score_trajectory, this);
    opt2.set_xtol_rel(0.0001);
    std::vector<double> lb2;
    std::vector<double> rb2;
    lb2.push_back(0);
    lb2.push_back(-1.8);
    lb2.push_back(-3.1);
    lb2.push_back(V_MIN);
    rb2.push_back(5.0);
    rb2.push_back(1.8);
    rb2.push_back(3.1);
    rb2.push_back(V_MAX);
    opt2.set_lower_bounds(lb2);
    opt2.set_upper_bounds(rb2);
    opt2.set_maxeval(max_iter);

    opt2.optimize(k, minf);

    ROS_DEBUG("Local Optimization - Trajectories evaluated: %d", trajectory_count);
    trajectory_count = 0;

    next_step->r = k[0];
    next_step->delta = k[1];
    next_step->theta = k[2];
    next_step->vMax = k[3];
    next_step->k1 = K_1;
    next_step->k2 = K_2;

    return;
  }
};
