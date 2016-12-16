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
#include <mpepc_global_planner/NavigationCost.h>

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
	   * call swapped makePlan
	   */
	  bool makePlan(const geometry_msgs::PoseStamped& start,
			  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
  private:
	  ros::Publisher nav_cost_pub_;
	  ros::Publisher nav_cost_map_pub_;
	  ros::ServiceServer cost_service_;

	  double map_resolution;
	  double interp_rotation_factor;
  };
};



#endif /* TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_NAVFN_ROS_EXT_H_ */
