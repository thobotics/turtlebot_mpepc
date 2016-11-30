/*
 * mpepc_local_planner.h
 *
 *  Created on: Nov 29, 2016
 *      Author: thobotics
 */

#ifndef TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_MPEPC_LOCAL_PLANNER_H_
#define TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_MPEPC_LOCAL_PLANNER_H_

#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
//#include <turtlebot_mpepc/GetNavCost.h>

namespace mpepc_local_planner {
	class MPEPCPlanner : public nav_core::BaseLocalPlanner {
	public:
	      /**
	       * @brief  Constructor for AckermannPlannerROS
	       */
		  MPEPCPlanner();

	      /**
	       * @brief  Constructs the planner
	       * @param name The name to give this instance of the trajectory planner
	       * @param tf A pointer to a transform listener
	       * @param costmap The cost map to use for assigning costs to trajectories
	       */
	      virtual void initialize(std::string name, tf::TransformListener* tf,
	          costmap_2d::Costmap2DROS* costmap_ros);

	      /**
	       * @brief  Destructor for the planner
	       */
	      virtual ~MPEPCPlanner();

	      /**
	       * @brief  Given the current position, orientation, and velocity of the robot,
	       * compute velocity commands to send to the base
	       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
	       * @return True if a valid trajectory was found, false otherwise
	       */
	      virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

	      /**
	       * @brief  Set the plan that the controller is following
	       * @param orig_global_plan The plan to pass to the controller
	       * @return True if the plan was updated successfully, false otherwise
	       */
	      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

	      /**
	       * @brief  Check if the goal pose has been achieved
	       * @return True if achieved, false otherwise
	       */
	      virtual bool isGoalReached();
	private:
	      bool initialized_;
	      bool goal_reached_;
	      tf::TransformListener* tf_;
	      std::vector<geometry_msgs::PoseStamped> global_plan_;
	      base_local_planner::OdometryHelperRos odom_helper_;

	      // for visualization, publishers of global and local plan
		  ros::Publisher g_plan_pub_, l_plan_pub_;

		  costmap_2d::Costmap2DROS* costmap_ros_;

		  ros::ServiceClient navfn_cost_;

	      bool isInitialized() {
			  return initialized_;
	      }
	};
};

#endif /* TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_MPEPC_LOCAL_PLANNER_H_ */
