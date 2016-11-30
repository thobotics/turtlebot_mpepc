/*
 * mpepc_local_planner.cpp
 *
 *  Created on: Nov 29, 2016
 *      Author: thobotics
 */

#include <turtlebot_mpepc/mpepc_local_planner.h>
#include <base_local_planner/goal_functions.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(mpepc_local_planner, MPEPCPlanner, mpepc_local_planner::MPEPCPlanner, nav_core::BaseLocalPlanner)

namespace mpepc_local_planner {
	MPEPCPlanner::MPEPCPlanner() : initialized_(false),
	    goal_reached_(false) { }

	void MPEPCPlanner::initialize(std::string name, tf::TransformListener* tf,
	          costmap_2d::Costmap2DROS* costmap_ros){
		if (! isInitialized()) {

			ros::NodeHandle private_nh("~/" + name);

			l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
			g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
			tf_ = tf;
			costmap_ros_ = costmap_ros;

			// make sure to update the costmap we'll use for this cycle
			costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

			std::string odom_topic;
			private_nh.param<std::string>("odom_topic", odom_topic, "odom");
			odom_helper_.setOdomTopic( odom_topic );

			//navfn_cost_ = private_nh.serviceClient<turtlebot_mpepc::GetNavCost>("/move_base/NavfnROSExt/nav_cost");

			initialized_ = true;
		}
		else{
		  ROS_WARN_NAMED("MPEPCPlanner", "This planner has already been initialized, doing nothing.");
		}
	}

	/**
	 * Copy from teb_local_planner
	 */
	bool MPEPCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
		  // check if plugin is initialized
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

	bool MPEPCPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
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

		// Get robot pose
		tf::Stamped<tf::Pose> robot_pose;
		costmap_ros_->getRobotPose(robot_pose);

		geometry_msgs::PoseStamped start;
		tf::poseStampedTFToMsg(robot_pose, start);
		geometry_msgs::Point currentPoint;
		currentPoint = start.pose.position;

		ROS_INFO("Request cost at %f %f", currentPoint.x, currentPoint.y);

		/*turtlebot_mpepc::GetNavCost service;
		service.request.world_point = currentPoint;
		if (navfn_cost_.call(service))
		{
			ROS_INFO("Response cost: %f", (double)service.response.cost);
			return true;
		}
		else
		{
			ROS_ERROR("Failed to call service");
		}*/

		return true;
	}


	bool MPEPCPlanner::isGoalReached() {
		if (! isInitialized()) {
		  ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		  return false;
		}
		// TODO:
		//  probably use some sort of goal tolerance parameters here
		return goal_reached_;
	  }

};


