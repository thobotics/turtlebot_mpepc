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
#ifndef MPEPC_LOCAL_PLANNER_MPEPC_PLANNER_ROS_H_
#define MPEPC_LOCAL_PLANNER_MPEPC_PLANNER_ROS_H_

#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <mpepc_local_planner/control_law.h>
#include <mpepc_local_planner/EgoGoal.h>
#include <math.h>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <nlopt.hpp>
#include "flann/flann.hpp"

namespace mpepc_local_planner {
  /**
   * @class MpepcPlannerROS
   * @brief ROS Wrapper for the AckermannPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class MpepcPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Constructor for MpepcPlannerROS
       */
      MpepcPlannerROS();

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
      virtual ~MpepcPlannerROS();

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
      bool isInitialized() {
    	  return initialized_;
	  }
      bool initialized_;
	  bool goal_reached_;
	  tf::TransformListener* tf_;
	  std::vector<geometry_msgs::PoseStamped> global_plan_;
	  base_local_planner::OdometryHelperRos odom_helper_;

	  // for visualization, publishers of global and local plan
	  ros::Publisher g_plan_pub_, l_plan_pub_;

	  costmap_2d::Costmap2DROS* costmap_ros_;

	  ros::ServiceClient navfn_cost_;

	  // Properties for mpepc optimization
	  boost::mutex pose_mutex_, cost_map_mutex_;
	  ControlLaw * cl;

	  flann::Index<flann::L2<float> > * obs_tree;
	  flann::Matrix<float> * data;
	  int trajectory_count;

	  double map_resolution;
	  double interp_rotation_factor;

	  // Function for mpepc optimization
	  double getGlobalPlannerCost(geometry_msgs::Point &world_point);
	  vector<MinDistResult> find_points_within_threshold(Point newPoint, double threshold);
	  MinDistResult find_nearest_neighbor(Point queryPoint);
	  double min_distance_to_obstacle(geometry_msgs::Pose local_current_pose, double *heading);
  };

  struct Point {
    float a;
    float b;
    int member;
    int p_idx;
    Point(float x, float y) : a(x), b(y), member(-1), p_idx(0) {}
    Point() : a(0), b(0), member(-1), p_idx(0) {}
    inline bool operator==(Point p) {
       if (p.a == a && p.b == b)
          return true;
       else
          return false;
    }
  };

  struct MinDistResult {
    Point p;
    double dist;
  };

  double mod(double x, double y)
  {
    double m= x - y * floor(x/y);
    // handle boundary cases resulted from floating-point cut off:
    if (y > 0)              // modulo range: [0..y)
    {
      if (m >= y)           // Mod(-1e-16             , 360.    ): m= 360.
        return 0;

      if (m < 0)
      {
        if (y+m == y)
          return 0;     // just in case...
        else
          return y+m;  // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14
      }
    }
    else                    // modulo range: (y..0]
    {
      if (m <= y)           // Mod(1e-16              , -360.   ): m= -360.
        return 0;

      if (m>0 )
      {
        if (y+m == y)
          return 0;    // just in case...
        else
          return y+m;  // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14
      }
    }

    return m;
  }

  double distance(double pose_x, double pose_y, double obx, double oby)
  {
    double diffx = obx - pose_x;
    double diffy = oby - pose_y;
    double dist = sqrt(diffx*diffx + diffy*diffy);
    return dist;
  }
};
#endif
