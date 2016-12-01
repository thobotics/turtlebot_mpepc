/*
 * navfn_ros_ext.cpp
 *
 *  Created on: Nov 28, 2016
 *      Author: thobotics
 */

#include <mpepc_global_planner/navfn_ros_ext.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_DECLARE_CLASS(navfn, NavfnROSExt, navfn::NavfnROSExt, nav_core::BaseGlobalPlanner)

namespace navfn {

	NavfnROSExt::NavfnROSExt() :
		NavfnROS(){}
	NavfnROSExt::NavfnROSExt(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
		NavfnROS(name, costmap_ros){}

	void NavfnROSExt::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		ros::NodeHandle private_nh("~/" + name);
		cost_service_ =  private_nh.advertiseService("nav_cost", &NavfnROSExt::getNavigationCost, this);

		NavfnROS::initialize(name, costmap_ros);

		ROS_INFO("Initialize NavfnROSExt");
	}

	bool NavfnROSExt::makePlan(const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan){
		boost::mutex::scoped_lock lock(mutex_);
		if(!initialized_){
		  ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		  return false;
		}

		//clear the plan, just in case
		plan.clear();

		ros::NodeHandle n;
		costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
		std::string global_frame = costmap_ros_->getGlobalFrameID();

		//until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
		if(tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)){
		  ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
					tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
		  return false;
		}

		if(tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)){
		  ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
					tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
		  return false;
		}

		double wx = start.pose.position.x;
		double wy = start.pose.position.y;

		unsigned int mx, my;
		if(!costmap->worldToMap(wx, wy, mx, my)){
		  ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
		  return false;
		}

		//clear the starting cell within the costmap because we know it can't be an obstacle
		tf::Stamped<tf::Pose> start_pose;
		tf::poseStampedMsgToTF(start, start_pose);
		clearRobotCell(start_pose, mx, my);

	#if 0
		{
		  static int n = 0;
		  static char filename[1000];
		  snprintf( filename, 1000, "navfnros-makeplan-costmapB-%04d.pgm", n++ );
		  costmap->saveRawMap( std::string( filename ));
		}
	#endif

		//make sure to resize the underlying array that Navfn uses
		planner_->setNavArr(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
		planner_->setCostmap(costmap->getCharMap(), true, allow_unknown_);

	#if 0
		{
		  static int n = 0;
		  static char filename[1000];
		  snprintf( filename, 1000, "navfnros-makeplan-costmapC-%04d", n++ );
		  planner_->savemap( filename );
		}
	#endif

		int map_start[2];
		map_start[0] = mx;
		map_start[1] = my;

		wx = goal.pose.position.x;
		wy = goal.pose.position.y;

		if(!costmap->worldToMap(wx, wy, mx, my)){
		  if(tolerance <= 0.0){
			ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
			return false;
		  }
		  mx = 0;
		  my = 0;
		}

		int map_goal[2];
		map_goal[0] = mx;
		map_goal[1] = my;

		planner_->setStart(map_goal);
		planner_->setGoal(map_start);

		//bool success = planner_->calcNavFnAstar();
		planner_->calcNavFnDijkstra(true);

		double resolution = costmap->getResolution();
		geometry_msgs::PoseStamped p, best_pose;
		p = goal;

		bool found_legal = false;
		double best_sdist = DBL_MAX;

		p.pose.position.y = goal.pose.position.y - tolerance;

		while(p.pose.position.y <= goal.pose.position.y + tolerance){
		  p.pose.position.x = goal.pose.position.x - tolerance;
		  while(p.pose.position.x <= goal.pose.position.x + tolerance){
			double potential = getPointPotential(p.pose.position);
			double sdist = sq_distance(p, goal);
			if(potential < POT_HIGH && sdist < best_sdist){
			  best_sdist = sdist;
			  best_pose = p;
			  found_legal = true;
			}
			p.pose.position.x += resolution;
		  }
		  p.pose.position.y += resolution;
		}

		if(found_legal){
		  //extract the plan
		  if(getPlanFromPotential(best_pose, plan)){
			//make sure the goal we push on has the same timestamp as the rest of the plan
			geometry_msgs::PoseStamped goal_copy = best_pose;
			goal_copy.header.stamp = ros::Time::now();
			plan.push_back(goal_copy);
		  }
		  else{
			ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
		  }
		}

		if (visualize_potential_){
		  //publish potential array
		  pcl::PointCloud<PotarrPoint> pot_area;
		  pot_area.header.frame_id = global_frame;
		  pot_area.points.clear();
		  std_msgs::Header header;
		  pcl_conversions::fromPCL(pot_area.header, header);
		  header.stamp = ros::Time::now();
		  pot_area.header = pcl_conversions::toPCL(header);

		  PotarrPoint pt;
		  float *pp = planner_->potarr;
		  double pot_x, pot_y;
		  for (unsigned int i = 0; i < (unsigned int)planner_->ny*planner_->nx ; i++)
		  {
			if (pp[i] < 10e7)
			{
			  mapToWorld(i%planner_->nx, i/planner_->nx, pot_x, pot_y);
			  pt.x = pot_x;
			  pt.y = pot_y;
			  pt.z = pp[i]/pp[planner_->start[1]*planner_->nx + planner_->start[0]]*20;
			  pt.pot_value = pp[i];
			  pot_area.push_back(pt);
			}
		  }
		  potarr_pub_.publish(pot_area);
		}

		//publish the plan for visualization purposes
		publishPlan(plan, 0.0, 1.0, 0.0, 0.0);

		return !plan.empty();
	}

	bool NavfnROSExt::getNavigationCost(mpepc_global_planner::GetNavCost::Request& req, mpepc_global_planner::GetNavCost::Response& resp){
		/*geometry_msgs::PoseStamped start;
		// Get robot pose
		tf::Stamped<tf::Pose> robot_pose;
		costmap_ros_->getRobotPose(robot_pose);
		tf::poseStampedTFToMsg(robot_pose, start);

		geometry_msgs::Point currentPoint;
		currentPoint = start.pose.position;

		ROS_INFO("Receieved %f, %f", currentPoint.x, currentPoint.y);*/

		geometry_msgs::Point current_point = req.world_point;
		if(validPointPotential(current_point))
			resp.cost =  getPointPotential(current_point);
		else
			resp.cost = -1;
		return true;
	}

}

