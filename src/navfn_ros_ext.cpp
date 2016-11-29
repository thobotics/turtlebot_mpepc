/*
 * navfn_ros_ext.cpp
 *
 *  Created on: Nov 28, 2016
 *      Author: thobotics
 */

#include <turtlebot_mpepc/navfn_ros_ext.h>

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

	bool NavfnROSExt::getNavigationCost(turtlebot_mpepc::GetNavCost::Request& req, turtlebot_mpepc::GetNavCost::Response& resp){
		geometry_msgs::Point currentPoint;

		//if(req.world_point == NULL){
			tf::Stamped<tf::Pose> global_pose;
			costmap_ros_->getRobotPose(global_pose);
			geometry_msgs::PoseStamped start;
			tf::poseStampedTFToMsg(global_pose, start);
			currentPoint = start.pose.position;
		//}else{
		//	currentPoint = req.world_point;
		//}

		ROS_INFO("Cost at %f %f", currentPoint.x, currentPoint.y);

		resp.cost =  getPointPotential(currentPoint);

		return true;
	}

}

