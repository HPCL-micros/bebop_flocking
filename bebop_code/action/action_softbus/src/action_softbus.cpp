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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sstream>
#include <iostream>

#include <pluginlib/class_list_macros.h>
#include <action_softbus/action_softbus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

namespace action_softbus {

  ActionSoftbus::ActionSoftbus(tf::TransformListener& tf):tf_(tf),ac_loader_("action_softbus", "action_softbus::ActionCore"), blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
      action_softbus_state_(FREE),action_start_(false), action_frequency_(20),new_pre_plan_(false),setup_(false), controller_patience_(5.0)
  {
      ros::NodeHandle nh;
      ros::NodeHandle priv_node("~");
      //if(!ros::param::get("~run_dwa", runDWA_))
      //{
          //ROS_INFO("not set run_dwa param.");
      //    runDWA_ = false;
      //}
      priv_node.param<bool>("run_dwa", runDWA_, false);
      priv_node.param<std::string>("action_plugin_name", action_plugin_name_, "action_softbus/DemoAction");

      swarm_pre_plan_ = new std::vector<decide_softbus_msgs::NavigationPoint>();
      controller_plan_ = new std::vector<decide_softbus_msgs::NavigationPoint>();
      swarm_plan_sub_ = nh.subscribe<decide_softbus_msgs::Path>("/swarm_plan", 1, boost::bind(&ActionSoftbus::swarmPlanCB, this, _1));
      action_start_sub_ = nh.subscribe<std_msgs::Int32>("/action_start", 1, boost::bind(&ActionSoftbus::actionStartCB, this, _1));
      vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      leader_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/leader_cmd_vel", 1000);

      //set up the planner's thread
      actioner_thread_ = new boost::thread(boost::bind(&ActionSoftbus::actionThread, this));

      //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
      controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
      controller_costmap_ros_->pause();
      
      if(runDWA_) {
          //create the dwa local planner
          try {
              //std::string local_planner = "base_local_planner/TrajectoryPlannerROS";
              std::string local_planner = "dwa_local_planner/DWAPlannerROS";
              tc_ = blp_loader_.createInstance(local_planner);
              ROS_INFO("Created local_planner %s", local_planner.c_str());
              tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
          } catch (const pluginlib::PluginlibException &ex) {
              ROS_FATAL(
                      "Failed to create the local planner, are you sure it is properly registered and that the containing library is built? Exception: %s",
                      ex.what());
              exit(1);
          }
      }

      controller_costmap_ros_->start();
      //initialize the action plugin
      try {
          //action_plugin_ = ac_loader_.createInstance("action_softbus/DemoAction");
          //action_plugin_->initialize("DemoAction", &tf_, controller_costmap_ros_);
          action_plugin_ = ac_loader_.createInstance(action_plugin_name_);
          action_plugin_->initialize(action_plugin_name_, &tf_, controller_costmap_ros_);
          ROS_INFO("action plugin %s is loaded successfully", action_plugin_name_.c_str());
      } catch (const pluginlib::PluginlibException& ex) {
          ROS_FATAL("Exception: %s", ex.what());
          exit(1);
      }
      dsrv_ = new dynamic_reconfigure::Server<action_softbus::ActionSoftbusConfig>(ros::NodeHandle("~"));
      dynamic_reconfigure::Server<action_softbus::ActionSoftbusConfig>::CallbackType cb = boost::bind(&ActionSoftbus::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
  }

  ActionSoftbus::~ActionSoftbus()
  {
      actioner_thread_->interrupt();
      actioner_thread_->join();
      delete actioner_thread_;

      if(controller_costmap_ros_ != NULL)
          delete controller_costmap_ros_;

      delete swarm_pre_plan_;
      delete controller_plan_;

      action_plugin_.reset();  //unload the plugin
      tc_.reset();
  }

  void ActionSoftbus::reconfigureCB(action_softbus::ActionSoftbusConfig &config, uint32_t level){
      boost::recursive_mutex::scoped_lock l(configuration_mutex_);

      //The first time we're called, we just want to make sure we have the
      //original configuration
      if(!setup_)
      {
          last_config_ = config;
          default_config_ = config;
          setup_ = true;
          return;
      }

      if(config.restore_defaults) {
          config = default_config_;
          //if someone sets restore defaults on the parameter server, prevent looping
          config.restore_defaults = false;
      }

      if(action_frequency_ != config.action_frequency)
      {
          action_frequency_ = config.action_frequency;
      }

      if(config.action_plugin != last_config_.action_plugin) {
          boost::shared_ptr<action_softbus::ActionCore> old_action_plugin = action_plugin_;
          //initialize the new action plugin
          std::cout<<"Loading new action plugin "<<config.action_plugin.c_str()<<std::endl;
          try {
              action_plugin_ = ac_loader_.createInstance(config.action_plugin);

              // wait for the current planner to finish planning
              //boost::unique_lock<boost::mutex> lock(planner_mutex_);

              // Clean up before initializing the new planner
              //lock.unlock();
          } catch (const pluginlib::PluginlibException& ex) {
              ROS_FATAL("Failed to create the %s action_plugin, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.action_plugin.c_str(), ex.what());
              action_plugin_ = old_action_plugin;
              config.action_plugin = last_config_.action_plugin;
          }

          ROS_INFO("reconfigure action plugin successfully!!!");
      }

      last_config_ = config;
  }

  void ActionSoftbus::resetState()
  {
      new_pre_plan_ = false;
      action_start_ = false;
      action_softbus_state_ = FREE;
  }

  void ActionSoftbus::swarmPlanCB(const decide_softbus_msgs::Path::ConstPtr& pre_plan)
  {
      std::vector<decide_softbus_msgs::NavigationPoint>* temp_plan = swarm_pre_plan_;

      boost::unique_lock<boost::mutex> lock(swarm_pre_plan_mutex_);
      swarm_pre_plan_->resize(pre_plan->navigationpoints.size());
      swarm_pre_plan_->clear();
      for(int i = 0; i < pre_plan->navigationpoints.size(); i++) {
          swarm_pre_plan_->push_back(pre_plan->navigationpoints[i]);
      }
      new_pre_plan_ = true;
      ROS_INFO("got new swarm pre plan. size: %d.", swarm_pre_plan_->size());
      lock.unlock();
  }

  void ActionSoftbus::actionStartCB(const std_msgs::Int32::ConstPtr& action_start)
  {
      if(action_start->data == 1)
      {
          boost::unique_lock<boost::mutex> lock(action_start_mutex_);
          action_start_ = true;
          lock.unlock();
      }
      else if(action_start->data == -1)
      {
          resetState();
      }
  }

  void ActionSoftbus::publishZeroVelocity(){
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      vel_pub_.publish(cmd_vel);
  }

  void ActionSoftbus::setActionPath()
  {
      if(!action_plugin_->setPath(*controller_plan_))
      {
          ROS_ERROR("Failed to pass swarm pre plan to the controller, aborting.");
          resetState();
          action_softbus_state_ = FREE;
      }
      else
      {
          ROS_INFO("Successed to pass swarm pre plan to the controller.");
          resetState();
          action_softbus_state_ = WAITING;
      }
  }

  void ActionSoftbus::setDWAActionPath()
  {
      std::vector<geometry_msgs::PoseStamped> controller_plan_tmp_;
      controller_plan_tmp_.clear();
      for(int i=0; i<(*controller_plan_).size(); i++)
      {
          geometry_msgs::PoseStamped tmp;
          tmp.header=(*controller_plan_)[i].header;
          tmp.pose=(*controller_plan_)[i].pose;
          controller_plan_tmp_.push_back(tmp);
      }

      if(!tc_->setPlan(controller_plan_tmp_))
      {
          ROS_ERROR("Failed to pass swarm pre plan to the controller, aborting.");
          resetState();
          action_softbus_state_ = FREE;
      }
      else
      {
          ROS_INFO("Successed to pass swarm pre plan to the controller.");
          resetState();
          action_softbus_state_ = WAITING;
      }
  }

  void ActionSoftbus::actionThread()
  {
      ros::NodeHandle n;
      ros::Rate r(action_frequency_);

      while(n.ok())
      {
          switch (action_softbus_state_){
              case FREE:
              {
                  if(new_pre_plan_){
                      boost::unique_lock<boost::mutex> lock(swarm_pre_plan_mutex_);
                      new_pre_plan_ = false;
                      std::vector<decide_softbus_msgs::NavigationPoint>* temp_plan = swarm_pre_plan_;
                      controller_plan_ = swarm_pre_plan_;
                      lock.unlock();

                      if(!runDWA_)
                          setActionPath();
                      else
                          setDWAActionPath();
                  }
                  break;
              }
              case WAITING:
              {
                  if(new_pre_plan_){
                      boost::unique_lock<boost::mutex> lock(swarm_pre_plan_mutex_);
                      new_pre_plan_ = false;
                      std::vector<decide_softbus_msgs::NavigationPoint>* temp_plan = controller_plan_;
                      controller_plan_ = swarm_pre_plan_;
                      lock.unlock();

                      if(!runDWA_)
                          setActionPath();
                      else
                          setDWAActionPath();
                  }

                  if(action_start_){
                      boost::unique_lock<boost::mutex> lock(action_start_mutex_);
                      action_start_ = false;
                      lock.unlock();
                      action_softbus_state_ = CONTROLLING;
                  }

                  break;
              }
              case CONTROLLING:
              {
                  if(new_pre_plan_){
                      boost::unique_lock<boost::mutex> lock(swarm_pre_plan_mutex_);
                      new_pre_plan_ = false;
                      std::vector<decide_softbus_msgs::NavigationPoint>* temp_plan = controller_plan_;
                      controller_plan_ = swarm_pre_plan_;
                      lock.unlock();

                      if(!runDWA_)
                          setActionPath();
                      else
                          setDWAActionPath();
                  }

                  bool done = false;
                  if(!runDWA_)
                      done = executeCycle();
                  else
                      done = executeDWACycle();
                  if(done)
                  {
                      ROS_INFO("action successed.");
                      action_softbus_state_ = FREE;
                  }

                  break;
              }
              case RECOVERYING:
              {
                  ROS_INFO("start recoverying...");
                  //TODO. recovery behaviours
                  action_softbus_state_ = FREE;
                  break;
              }
          }

          r.sleep();
      }
  }

  bool ActionSoftbus::executeCycle()
  {
      geometry_msgs::Twist cmd_vel;

      if(!controller_costmap_ros_->isCurrent()){
          ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
          publishZeroVelocity();
          return false;
      }

      if(action_plugin_->isGoalReached()){
          ROS_DEBUG_NAMED("action_softbus","Goal reached!");
          return true;
      }

      {
          if(action_plugin_->computeVelCmd(cmd_vel)) {
              last_valid_control_ = ros::Time::now();
              vel_pub_.publish(cmd_vel);
          }
          else{
              ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);
              if(ros::Time::now() > attempt_end) {
                  ROS_INFO("could not get cmd_vel.");
                  publishZeroVelocity();
                  action_softbus_state_ = RECOVERYING;
              }
              else
              {
                  //ROS_INFO("recovery successfully.");
                  //publishZeroVelocity();
                  //setActionPath();
                  //action_softbus_state_ = CONTROLLING;
                  publishZeroVelocity();
                  ROS_INFO("waiting for valid cmd_vel...");
              }
          }
      }

      return false;
  }


    bool ActionSoftbus::executeDWACycle()
    {
        geometry_msgs::Twist cmd_vel;

        if(!controller_costmap_ros_->isCurrent()){
            ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
            publishZeroVelocity();
            return false;
        }

        if(tc_->isGoalReached()){
            ROS_DEBUG_NAMED("action_softbus","Goal reached!");
            return true;
        }

        {
            boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
            if(tc_->computeVelocityCommands(cmd_vel)){
                last_valid_control_ = ros::Time::now();
                vel_pub_.publish(cmd_vel);
                leader_vel_pub_.publish(cmd_vel);
            }
            else{
                ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);
                if(ros::Time::now() > attempt_end) {
                    ROS_INFO("could not get cmd_vel.");
                    publishZeroVelocity();
                    action_softbus_state_ = RECOVERYING;
                }
                else
                {
                    ROS_INFO("recovery successfully.");
                    publishZeroVelocity();
                    setDWAActionPath();
                    action_softbus_state_ = CONTROLLING;
                }
            }
        }

        return false;
    }
};

