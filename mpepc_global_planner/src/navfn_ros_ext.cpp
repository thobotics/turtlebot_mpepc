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

		ROS_INFO("Initialized NavfnROSExt");
	}

	bool NavfnROSExt::makePlan(const geometry_msgs::PoseStamped& start,
				  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
		return NavfnROS::makePlan(goal, start, plan);
	}

	bool NavfnROSExt::getNavigationCost(mpepc_global_planner::GetNavCost::Request& req, mpepc_global_planner::GetNavCost::Response& resp){
		geometry_msgs::Point current_point = req.world_point;
		if(!validPointPotential(current_point))
			ROS_WARN("Non-valid point to get potential cost");
		resp.cost =  getPointPotential(current_point);
		return true;
	}

}

