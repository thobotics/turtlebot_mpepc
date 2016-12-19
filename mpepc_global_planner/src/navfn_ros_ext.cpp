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
		nav_cost_pub_ = private_nh.advertise<mpepc_global_planner::NavigationCost>("nav_cost_arr", 1);
		nav_cost_map_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("nav_cost_map", 1);
		cost_service_ = private_nh.advertiseService("nav_cost", &NavfnROSExt::getNavigationCost, this);

		ROS_INFO("Initialized NavfnROSExt");
	}

	bool NavfnROSExt::makePlan(const geometry_msgs::PoseStamped& start,
				  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
		bool result = NavfnROS::makePlan(goal, start, plan);

		// Now copy orientation of goal pose to plan
		/*geometry_msgs::PoseStamped goal_copy = goal;
		goal_copy.header.stamp = ros::Time::now();
		plan[0] = goal_copy;*/
		std::reverse(plan.begin(), plan.end());

		mpepc_global_planner::NavigationCost nav_cost;
		float * tmp = planner_->potarr;
		std::vector<float> potarr(tmp, tmp + (unsigned int)planner_->ns);
		nav_cost.potential_cost = potarr;
		nav_cost.info.width = costmap_ros_->getCostmap()->getSizeInCellsX();
		nav_cost.info.height = costmap_ros_->getCostmap()->getSizeInCellsY();
		nav_cost.info.resolution = map_resolution;
		nav_cost.info.origin.position.x = costmap_ros_->getCostmap()->getOriginX();
		nav_cost.info.origin.position.y = costmap_ros_->getCostmap()->getOriginY();
		nav_cost_pub_.publish(nav_cost);

		nav_msgs::OccupancyGrid potentialMap;
		std::vector<int8_t> potmap;
		float max = -1;
		float min = -1;
		// Find min and max
		for(std::vector<float>::size_type i = 0; i != potarr.size(); i++) {
			if(potarr[i] < POT_HIGH && potarr[i] >= 0){
				if(max == -1 || potarr[i] > max)
					max = potarr[i];
				if(min == -1 || potarr[i] < min)
					min = potarr[i];
			}
		}
		// Scale it to int 0 -> 100
		for(std::vector<float>::size_type i = 0; i != potarr.size(); i++) {
			float scale;
			if(potarr[i] < POT_HIGH && potarr[i] >= 0){
				scale = (potarr[i]- min)/(max - min)*100.0;
				potmap.push_back(static_cast<int8_t>(scale));
			}else{
				potmap.push_back(-1);
			}
		}

		potentialMap.data = potmap;
		potentialMap.header.frame_id = costmap_ros_->getGlobalFrameID();
		potentialMap.header.stamp = ros::Time::now();
		potentialMap.info.width = costmap_ros_->getCostmap()->getSizeInCellsX();
		potentialMap.info.height = costmap_ros_->getCostmap()->getSizeInCellsY();
		potentialMap.info.resolution = map_resolution;
		potentialMap.info.origin.position.x = costmap_ros_->getCostmap()->getOriginX();
		potentialMap.info.origin.position.y = costmap_ros_->getCostmap()->getOriginY();
		nav_cost_map_pub_.publish(potentialMap);
		return result;
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

