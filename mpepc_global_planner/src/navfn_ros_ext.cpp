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

	bool NavfnROSExt::getNavigationCost(mpepc_global_planner::GetNavCost::Request& req, mpepc_global_planner::GetNavCost::Response& resp){
		resp.cost =  getPointPotential(req.world_point );

		return true;
	}

}

