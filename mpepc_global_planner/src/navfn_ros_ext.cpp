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
#include <math.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_DECLARE_CLASS(navfn, NavfnROSExt, navfn::NavfnROSExt, nav_core::BaseGlobalPlanner)

namespace navfn {

	static const double PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;

	NavfnROSExt::NavfnROSExt() :
		NavfnROS(){}
	NavfnROSExt::NavfnROSExt(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
		NavfnROS(name, costmap_ros){}

	void NavfnROSExt::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		NavfnROS::initialize(name, costmap_ros);

		// Must initialize first and then get costmap
		map_resolution = (costmap_ros_->getCostmap())->getResolution();
		interp_rotation_factor = (sqrt(2) * map_resolution - map_resolution)/2;

		ros::NodeHandle private_nh("~/" + name);
		cost_service_ =  private_nh.advertiseService("nav_cost", &NavfnROSExt::getNavigationCost, this);

		ROS_INFO("Initialized NavfnROSExt");
	}

	bool NavfnROSExt::makePlan(const geometry_msgs::PoseStamped& start,
				  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
		return NavfnROS::makePlan(goal, start, plan);
	}

	bool NavfnROSExt::getNavigationCost(mpepc_global_planner::GetNavCost::Request& req, mpepc_global_planner::GetNavCost::Response& resp){
		geometry_msgs::Point position = req.world_point;

		//if(!validPointPotential(position))
		//	ROS_WARN("Non-valid point to get potential cost");
		resp.cost =  getPointPotential(position);

		/*costmap_2d::Costmap2D * temp_costmap;
		temp_costmap = costmap_ros_->getCostmap();

		// find world coords of current cell
		double cell_wx, cell_wy;
		unsigned int cell_mx, cell_my;
		temp_costmap->worldToMap(position.x, position.y, cell_mx, cell_my);
		temp_costmap->mapToWorld(cell_mx, cell_my, cell_wx, cell_wy);

		geometry_msgs::Point tempPoint = position;

		tempPoint.x = tempPoint.x + map_resolution;
		double cost0 = getPointPotential(tempPoint);

		tempPoint = position;
		tempPoint.y = tempPoint.y + map_resolution;
		double cost90 = getPointPotential(tempPoint);

		tempPoint = position;
		tempPoint.x = tempPoint.x - map_resolution;
		double cost180 = getPointPotential(tempPoint);

		// Block at 270
		tempPoint = position;
		tempPoint.y = tempPoint.y - map_resolution;
		double cost270 = getPointPotential(tempPoint);

		geometry_msgs::Point rotPoint;
		rotPoint.x = ((position.x - cell_wx)*cos(-1*PI/4) - (position.y - cell_wy)*sin(-1*PI/4)) + interp_rotation_factor;
		rotPoint.y = ((position.x - cell_wx)*sin(-1*PI/4) + (position.y - cell_wy)*cos(-1*PI/4)) + interp_rotation_factor;

		double a00 = cost180;
		double a10 = cost270 - cost180;
		double a01 = cost90 - cost180;
		double a11 = cost180 - cost270 - cost90 + cost0;

		resp.cost = a00 + a10*rotPoint.x + a01*rotPoint.y + a11*rotPoint.x*rotPoint.y;*/

		return true;
	}

}

