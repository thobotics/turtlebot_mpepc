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
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
		bool result = NavfnROS::makePlan(start, goal, plan);

		geometry_msgs::Point currentPoint = goal.pose.position;
		double cost = getPointPotential(currentPoint);
		ROS_INFO("AFTER PLANNED goal cost at %f,%f: %f",
				currentPoint.x, currentPoint.y, cost);

		return result;
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

