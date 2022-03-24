/*
 * navfn_ros_ext.cpp
 *
 *  Created on: Nov 28, 2016
 *      Author: thobotics
 */

#include <mpepc_global_planner/navfn_ros_ext.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <algorithm>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(navfn::NavfnROSExt, nav_core::BaseGlobalPlanner)

namespace navfn {

	static const double PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;

	NavfnROSExt::NavfnROSExt() :
		global_planner::GlobalPlanner(){}
	NavfnROSExt::NavfnROSExt(std::string name, costmap_2d::Costmap2D* costmap_ros, std::string frame_id) :
		global_planner::GlobalPlanner(name, costmap_ros, frame_id){}

	void NavfnROSExt::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		global_planner::GlobalPlanner::initialize(name, costmap_ros);
		ROS_INFO("Initialized NavfnROSExt");
	}

	bool NavfnROSExt::makePlan(const geometry_msgs::PoseStamped& start,
				  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
		bool result = global_planner::GlobalPlanner::makePlan(goal, start, plan);

		// Now copy orientation of goal pose to plan
		geometry_msgs::PoseStamped goal_copy = goal;
		goal_copy.header.stamp = ros::Time::now();
		plan[0] = goal_copy;
		std::reverse(plan.begin(), plan.end());

		return result;
	}
}

