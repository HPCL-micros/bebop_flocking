/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <tf_bridge/tf_bridge.h>

#include <actionlib/server/simple_action_server.h>
//#include <move_base_msgs/MoveBaseAction.h>
#include <decide_softbus_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
//#include <nav_msgs/GetPlan.h>
#include <decide_softbus_msgs/GetPlan.h>
#include <decide_softbus_msgs/SwarmConfig.h>
#include <decide_softbus_msgs/SetSwarmConfig.h>
#include <decide_softbus_msgs/SetControlling.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<decide_softbus_msgs::MoveBaseAction> MoveBaseActionServer;

  enum RecoveryTrigger
  {
    PLANNING_R,
    CONTROLLING_R,
    OSCILLATION_R
  };

  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf::TransformListener& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

    private:

      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      //bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);
      bool planService(decide_softbus_msgs::GetPlan::Request &req, decide_softbus_msgs::GetPlan::Response &resp);

      /**
       * @brief  A service call that can be made when setting the swarm config
       * @param  req The swarm config request
       * @param  resp The feedback request
       * @return True if setting succeeded, false otherwise
       */
      bool setSimpleSwarmConfigService(decide_softbus_msgs::SetSwarmConfig::Request &req, decide_softbus_msgs::SetSwarmConfig::Response &resp);

      /**
       * @brief  A service call that can be made when setting the controlling state
       * @param  req The controlling request
       * @param  resp The feedback request
       * @return True if setting succeeded, false otherwise
       */
      bool setControllingService(decide_softbus_msgs::SetControlling::Request &req, decide_softbus_msgs::SetControlling::Response &resp);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      //bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      bool makePlan(const decide_softbus_msgs::NavigationPoint& goal, std::vector<decide_softbus_msgs::NavigationPoint>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

      //void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
      void goalCB(const decide_softbus_msgs::NavigationPoint::ConstPtr& goal);

      void planThread();

      //void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);
      void executeCb(const decide_softbus_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      //double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
      double distance(const decide_softbus_msgs::NavigationPoint& p1, const decide_softbus_msgs::NavigationPoint& p2);

      //geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);
      decide_softbus_msgs::NavigationPoint goalToGlobalFrame(const decide_softbus_msgs::NavigationPoint& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      tf::TransformListener& tf_;

      MoveBaseActionServer* as_;

      //boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
      std::string robot_base_frame_, global_frame_;

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      unsigned int recovery_index_;

      tf::Stamped<tf::Pose> global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_rate_;
      double planner_patience_, controller_patience_;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;
      ros::Publisher swarm_vel_pub_;
      ros::Publisher action_start_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      ros::ServiceServer set_simple_swarm_config_srv_;
      ros::ServiceServer set_controlling_srv_;
      ros::Publisher swarm_plan_pub_;
      ros::Publisher rviz_plan_pub_;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      double oscillation_timeout_, oscillation_distance_;

      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      double failed_plan_count_;
      double planner_try_times_;
      //geometry_msgs::PoseStamped oscillation_pose_;
      decide_softbus_msgs::NavigationPoint oscillation_pose_;
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      //pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      //std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      //std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      //std::vector<geometry_msgs::PoseStamped>* controller_plan_;
      std::vector<decide_softbus_msgs::NavigationPoint>* planner_plan_;
      std::vector<decide_softbus_msgs::NavigationPoint>* latest_plan_;
      std::vector<decide_softbus_msgs::NavigationPoint>* controller_plan_;

      //set up the planner's thread
      bool runPlanner_;
      bool planSuccess_;
      boost::mutex planner_mutex_;
      boost::condition_variable planner_cond_;
      //geometry_msgs::PoseStamped planner_goal_;
      decide_softbus_msgs::NavigationPoint planner_goal_;
      boost::thread* planner_thread_;

      bool planner_try_best_;
      boost::mutex swarm_config_mutex_;
      boost::mutex controlling_mutex_;
      boost::mutex planner_try_best_mutex_;

      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;
      
      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;

      //swarm configs
      decide_softbus_msgs::SwarmConfig swarm_config_;

      //swarm config for every navigation point
      std::vector<decide_softbus_msgs::SwarmConfig>* swarm_configs_cache_;
  };
};
#endif

