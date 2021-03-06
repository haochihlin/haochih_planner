/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
* Author: HaoChih, LIN
* Email: f44006076@gmail.com
*********************************************************************/
#ifndef HAOCHIH_LOCAL_PLANNER_HAOCHIH_PLANNER_ROS_H_
#define HAOCHIH_LOCAL_PLANNER_HAOCHIH_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <math.h>

#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <haochih_local_planner/HaoChihPlannerConfig.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

namespace haochih_local_planner 
{
  /**
   * @class HaoChihPlannerROS
   * @brief ROS Wrapper for the HaoChihPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class HaoChihPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Constructor for DWAPlannerROS wrapper
       */
      HaoChihPlannerROS();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~HaoChihPlannerROS();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);


      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();


      bool isInitialized() {
        return initialized_;
      }

    private:
      ros::Timer timer_update_; // for ROS Timer loop;

      // Private functions
      void reconfigureCB(HaoChihPlannerConfig &config, uint32_t level);
      void TimerUpdatePlan(const ros::TimerEvent&);
      bool UpdateGlobalPlan();
      inline double DistSquare(const double &a_x, const double &a_y, const double &b_x, const double &b_y)
      {
        return (a_x - b_x)*(a_x - b_x) + (a_y - b_y)*(a_y - b_y);
      }
      inline double FindDist(const double &a_x, const double &a_y, const double &b_x, const double &b_y)
      {
        return sqrt( (a_x - b_x)*(a_x - b_x) + (a_y - b_y)*(a_y - b_y) );
      }
      // Private Parameters
      double l1_dist_;
      int ClosetSearchStartId_;
      bool initialized_;
      bool GoalReached_;
      bool TimerPaused_;
      
      vector<geometry_msgs::PoseStamped> l1_based_global_plan_;
      
      tf::TransformListener* tf_; ///< @brief Used for transforming point clouds

      tf::Stamped<tf::Pose> current_pose_;

      boost::shared_ptr<dwa_local_planner::DWAPlannerROS> dwaObj_; ///< @DWA controller obj

      costmap_2d::Costmap2DROS* costmap_ros_;

      dynamic_reconfigure::Server<HaoChihPlannerConfig> *dsrv_;

  };
};
#endif
