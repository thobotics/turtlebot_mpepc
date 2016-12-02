/*
 * navfn_ros_ext.h
 *
 *  Created on: Nov 28, 2016
 *      Author: thobotics
 */

#ifndef TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_NAVFN_ROS_EXT_H_
#define TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_NAVFN_ROS_EXT_H_

#include <navfn/navfn_ros.h>
#include <mpepc_global_planner/GetNavCost.h>

#include <ros/ros.h>
#include <navfn/navfn.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <navfn/potarr_point.h>
#include <pcl_ros/publisher.h>

namespace navfn {
/**
   * @class NavfnROS
   * @brief Provides a ROS wrapper for the navfn planner which runs a fast, interpolated navigation function on a costmap.
   */
  class NavfnROSExt : public NavfnROS {
  public:
	  NavfnROSExt();
	  NavfnROSExt(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	  /**
	   * Note: it is overload function
	   */
	  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

	  /**
	   * Service call back
	   */
	  bool getNavigationCost(mpepc_global_planner::GetNavCost::Request& req, mpepc_global_planner::GetNavCost::Response& resp);


	  /**
	   * This is important !!!. Move_base call this, need to force this
	   * call edited NavfnROSExt
	   */
	  bool makePlan(const geometry_msgs::PoseStamped& start,
			  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

	  /**
	   * Create a copy from NavfnROS
	   * where change a little bit to support Navigation Cost service
	   */
	  bool makePlan(const geometry_msgs::PoseStamped& start,
			  const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);
  private:
	  ros::ServiceServer cost_service_;

	  /**
	   * Copy from NavfnROS to make makePlan work
	   */
	  inline double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
		double dx = p1.pose.position.x - p2.pose.position.x;
		double dy = p1.pose.position.y - p2.pose.position.y;
		return dx*dx +dy*dy;
	  }

	  void mapToWorld(double mx, double my, double& wx, double& wy);
	  void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);
	  double default_tolerance_;
	  std::string tf_prefix_;
	  boost::mutex mutex_;
  };
};



#endif /* TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_NAVFN_ROS_EXT_H_ */
