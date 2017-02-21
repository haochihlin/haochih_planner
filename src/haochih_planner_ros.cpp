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

#include <haochih_local_planner/haochih_planner_ros.h>
#include <Eigen/Core>
#include <cmath>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>

#define MAX_DIST 1E+20

using namespace std;

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(haochih_local_planner::HaoChihPlannerROS, nav_core::BaseLocalPlanner)

namespace haochih_local_planner
{

  HaoChihPlannerROS::HaoChihPlannerROS() : initialized_(false), l1_dist_(2.0), ClosetSearchStartId_(0)
  {
    this->dwaObj_ = boost::shared_ptr<dwa_local_planner::DWAPlannerROS>(new dwa_local_planner::DWAPlannerROS);
  }


  HaoChihPlannerROS::~HaoChihPlannerROS()
  {
    //make sure to clean things up
    this->dwaObj_->~DWAPlannerROS();
    delete this->dsrv_;
    this->l1_based_global_plan_.clear();
  }


  void HaoChihPlannerROS::initialize( string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if (! initialized_)
    {
      ros::NodeHandle private_nh("~/" + name + "_l1_setting");
      this->tf_ = tf;
      this->costmap_ros_ = costmap_ros;
      this->costmap_ros_->getRobotPose(this->current_pose_);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = this->costmap_ros_->getCostmap();

      // Initialize self parameters
      this->initialized_ = true;
      this->ClosetSearchStartId_ = 0;
      this->l1_based_global_plan_.clear();
      
      // Initialize the dynamic_reconfigure servie
      this->dsrv_ = new dynamic_reconfigure::Server<HaoChihPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<HaoChihPlannerConfig>::CallbackType cb = boost::bind(&HaoChihPlannerROS::reconfigureCB, this, _1, _2);
      this->dsrv_->setCallback(cb);

      // Initialize the ROS Timer loop;
      this->timer_update_ = private_nh.createTimer(ros::Duration(0.2), &HaoChihPlannerROS::TimerUpdatePlan, this); // Duration(0.2) -> 5Hz
      
      // Initialize the DWA-ROS Obj
      this->dwaObj_->initialize( name, tf, costmap_ros);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  

  bool HaoChihPlannerROS::setPlan(const vector<geometry_msgs::PoseStamped>& orig_global_plan)
  {
    if (! initialized_) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    // Clear previous global plan vector once receives new plan vector
    this->l1_based_global_plan_.clear();
    // Assign new plan data into "l1_based_global_plan" vector
    this->l1_based_global_plan_.assign(orig_global_plan.begin(), orig_global_plan.end());

    this->ClosetSearchStartId_ = 0;

    return UpdateGlobalPlan();
  }


  void HaoChihPlannerROS::TimerUpdatePlan(const ros::TimerEvent&)
  {
    if(! initialized_)
      return ;
    
    if(this->l1_based_global_plan_.size() == 0)
      return ;

    #cout << "Ho !" << endl;
    UpdateGlobalPlan();
  }


  bool HaoChihPlannerROS::UpdateGlobalPlan()
  {
    if(this->dwaObj_->isGoalReached() )
      return false;
  
    // Get robot current pose
    tf::Stamped<tf::Pose> current_pose;
    this->costmap_ros_->getRobotPose(current_pose);
    double robot_x = current_pose.getOrigin().getX();
    double robot_y = current_pose.getOrigin().getY();
    double closet_dist = MAX_DIST;

    // Find closet wayponit on global plan
    for(int i = this->ClosetSearchStartId_ ; i < this->l1_based_global_plan_.size(); ++i)
    {
      double g_x = this->l1_based_global_plan_[i].pose.position.x;
      double g_y = this->l1_based_global_plan_[i].pose.position.y;
      double curr_closet_dist = DistSquare(g_x, g_y, robot_x, robot_y);
      if(curr_closet_dist <= closet_dist)
        closet_dist = curr_closet_dist;
      else
      {
        this->ClosetSearchStartId_ = i - 1;
        break;
      }
    }

    if( this->ClosetSearchStartId_ < 0 || this->ClosetSearchStartId_ >= this->l1_based_global_plan_.size() )
    {
      ROS_ERROR("Can not find the closet waypoint on the global plan.");
      return false;
    }

    // Generate l1 based global plan
    double l1_dist = this->l1_dist_; // To avoide the value of l1 dist being dynamically changed in this loop
    double accumulate_dist = 0;
    vector<geometry_msgs::PoseStamped> updated_l1_plan;
    updated_l1_plan.push_back(this->l1_based_global_plan_[this->ClosetSearchStartId_]); // put the first waypoint into l1-based plan
    for(int i = this->ClosetSearchStartId_ ; i < (this->l1_based_global_plan_.size()-3) ; ++i)
    {
      double curr_dist = FindDist(this->l1_based_global_plan_[i].pose.position.x,   this->l1_based_global_plan_[i].pose.position.y,
                                  this->l1_based_global_plan_[i+1].pose.position.x, this->l1_based_global_plan_[i+1].pose.position.y  );
      accumulate_dist = accumulate_dist + curr_dist;                            
      if(accumulate_dist <= l1_dist)
        updated_l1_plan.push_back(this->l1_based_global_plan_[i+1]);
      else
        break;
    }
    
    if(updated_l1_plan.size() >= 1)
      return this->dwaObj_->setPlan(updated_l1_plan); //Send l1-based plan to DWA planner
    else
    {
      ROS_ERROR("Can not generate the l1 based global plan.");
      return false;
    }
  }


  bool HaoChihPlannerROS::isGoalReached() 
  {
    return this->dwaObj_->isGoalReached();
  }


  bool HaoChihPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) 
  {
    this->dwaObj_->computeVelocityCommands(cmd_vel);
  }

  void HaoChihPlannerROS::reconfigureCB(HaoChihPlannerConfig &config, uint32_t level) 
  {
    this->l1_dist_ = config.l1_dist;   
  }

};
