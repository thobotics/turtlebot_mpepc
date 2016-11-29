/*
 * navfn_ros_ext.h
 *
 *  Created on: Nov 28, 2016
 *      Author: thobotics
 */

#ifndef TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_NAVFN_ROS_EXT_H_
#define TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_NAVFN_ROS_EXT_H_

#include <navfn/navfn_ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <turtlebot_mpepc/GetNavCost.h>

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
	  bool getNavigationCost(turtlebot_mpepc::GetNavCost::Request& req, turtlebot_mpepc::GetNavCost::Response& resp);
  private:
	  ros::ServiceServer cost_service_;
  };
};



#endif /* TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_NAVFN_ROS_EXT_H_ */
