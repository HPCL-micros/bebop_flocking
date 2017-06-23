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
* Author: Xuefeng Chang
*********************************************************************/
#ifndef ACTION_SOFTBUS_H_
#define ACTION_SOFTBUS_H_

#include <vector>
#include <string>
#include <queue>

#include <ros/ros.h>

#include <tf_bridge/tf_bridge.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
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
#include "action_softbus/ActionSoftbusConfig.h"

#include "action_softbus/action_core.h"

namespace action_softbus {

  enum ActionSoftbusState {
      FREE,
      WAITING,
      CONTROLLING,
      RECOVERYING
  };

  /**
   * @class ActionSoftbus
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class ActionSoftbus {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      ActionSoftbus(tf::TransformListener& tf);
      virtual ~ActionSoftbus();

      void actionThread();

      void setActionPath();

      void setDWAActionPath();

      bool executeCycle();

      bool executeDWACycle();

      void swarmPlanCB(const decide_softbus_msgs::Path::ConstPtr& pre_plan);

      void actionStartCB(const std_msgs::Int32::ConstPtr& action_start);

      void publishZeroVelocity();

      void resetState();

      ros::Subscriber swarm_plan_sub_;
      ros::Subscriber action_start_sub_;
      
      ros::Publisher vel_pub_;

      ros::Publisher leader_vel_pub_;

      //swarm pre plan in decide phase
      std::vector<decide_softbus_msgs::NavigationPoint>* swarm_pre_plan_;
      std::vector<decide_softbus_msgs::NavigationPoint>* controller_plan_;
      bool new_pre_plan_;
      bool action_start_;

      boost::mutex swarm_pre_plan_mutex_;
      boost::mutex action_start_mutex_;
      boost::thread* actioner_thread_;

      ActionSoftbusState action_softbus_state_;

      boost::shared_ptr<action_softbus::ActionCore> action_plugin_;
      pluginlib::ClassLoader<action_softbus::ActionCore> ac_loader_;

      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<action_softbus::ActionSoftbusConfig> *dsrv_;
      bool setup_;
      double action_frequency_;
      ros::Time last_valid_control_;
      double controller_patience_;

      void reconfigureCB(action_softbus::ActionSoftbusConfig &config, uint32_t level);

      action_softbus::ActionSoftbusConfig last_config_;
      action_softbus::ActionSoftbusConfig default_config_;

      //maybe taishitu later
      tf::TransformListener& tf_;
      costmap_2d::Costmap2DROS* controller_costmap_ros_;

      //for the purpose that using dwa
      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;

      //dwa flag
      bool runDWA_;
      
      //action plugin name
      std::string action_plugin_name_;
  };
};
#endif

