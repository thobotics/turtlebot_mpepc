/*
 * navfn_ros_ext.h
 *
 *  Created on: Nov 28, 2016
 *      Author: thobotics
 */

#ifndef TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_NAVFN_ROS_EXT_H_
#define TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_NAVFN_ROS_EXT_H_

#include <global_planner/planner_core.h>

namespace navfn {
/**
   * @class NavfnROS
   * @brief Provides a ROS wrapper for the navfn planner which runs a fast, interpolated navigation function on a costmap.
   */
  class NavfnROSExt : public global_planner::GlobalPlanner {
  public:
	  NavfnROSExt();
	  NavfnROSExt(std::string name, costmap_2d::Costmap2D* costmap_ros, std::string frame_id);
	  /**
	   * Note: it is overload function
	   */
	  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

	  /**
	   * This is important !!!. Move_base call this, need to force this
	   * call swapped makePlan
	   */
	  bool makePlan(const geometry_msgs::PoseStamped& start,
			  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
  };
};



#endif /* TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_NAVFN_ROS_EXT_H_ */
