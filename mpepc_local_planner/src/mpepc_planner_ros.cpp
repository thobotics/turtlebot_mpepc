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
#include <mpepc_global_planner/GetNavCost.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(mpepc_local_planner::MpepcPlannerROS, nav_core::BaseLocalPlanner)

namespace mpepc_local_planner {

  MpepcPlannerROS::MpepcPlannerROS() : initialized_(false),
    goal_reached_(false) {

  }

  void MpepcPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
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

			navfn_cost_ = private_nh.serviceClient<mpepc_global_planner::GetNavCost>("/move_base/NavfnROSExt/nav_cost");

			initialized_ = true;
		}
		else{
		  ROS_WARN_NAMED("MPEPCPlanner", "This planner has already been initialized, doing nothing.");
		}
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

		  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
		  // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

		  // reset goal_reached_ flag
		  goal_reached_ = false;

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


  MpepcPlannerROS::~MpepcPlannerROS(){  }


  bool MpepcPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // if we don't have a plan, what are we doing here???
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

		mpepc_global_planner::GetNavCost service;
		service.request.world_point = currentPoint;
		if (navfn_cost_.call(service))
		{
			ROS_INFO("Response cost: %f", (double)service.response.cost);
			return true;
		}
		else
		{
			ROS_ERROR("Failed to call service");
		}

		return true;
  }
};
