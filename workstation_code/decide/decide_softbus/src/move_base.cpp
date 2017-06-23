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
#include <decide_softbus/move_base.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

namespace move_base {

  MoveBase::MoveBase(tf::TransformListener& tf) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    //blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), 
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL), planner_try_best_(false),
    runPlanner_(false), planSuccess_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false){

    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    std::string global_planner;
    //std::string local_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    //private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("/map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("planner_rate", planner_rate_, 20.0);
    private_nh.param("planner_try_times", planner_try_times_, 10.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 10.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    //set up plan triple buffer
    //planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    //latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    //controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    planner_plan_ = new std::vector<decide_softbus_msgs::NavigationPoint>();
    latest_plan_ = new std::vector<decide_softbus_msgs::NavigationPoint>();
    controller_plan_ = new std::vector<decide_softbus_msgs::NavigationPoint>();

    //set up the planner's thread
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    //for comanding the base
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //for commanding the swarm virtual leader
    swarm_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/swarm_cmd_vel", 1);

    //for commanding the action softbus
    action_start_pub_ = nh.advertise<std_msgs::Int32>("/action_start", 1);

    //current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );
    current_goal_pub_ = private_nh.advertise<decide_softbus_msgs::NavigationPoint>("current_goal", 0 );

    ros::NodeHandle action_nh("move_base");
    //action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
    action_goal_pub_ = action_nh.advertise<decide_softbus_msgs::MoveBaseActionGoal>("goal", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_simple");
    //goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));
    goal_sub_ = simple_nh.subscribe<decide_softbus_msgs::NavigationPoint>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    // swarm plan publisher for visualization purpose
    //swarm_plan_pub_ = private_nh.advertise<decide_softbus_msgs::Path>("swarm_plan", 1);
    swarm_plan_pub_ = nh.advertise<decide_softbus_msgs::Path>("/swarm_plan", 1);
    
    rviz_plan_pub_ = nh.advertise<geometry_msgs::PoseArray>("/rviz_plan", 1);

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    try {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    /*try {
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }*/

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //advertise a service for getting a plan
    //make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //advertise a service for setting swarm config
    set_simple_swarm_config_srv_ = private_nh.advertiseService("set_simple_swarm_config", &MoveBase::setSimpleSwarmConfigService, this);

    //advertise a service for setting controlling state
    set_controlling_srv_ = private_nh.advertiseService("set_controlling", &MoveBase::setControllingService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    //we're all set up now so we can start the action server
    as_->start();

    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    //initialize the swarm config
    swarm_config_.arrival_time = ros::Time(0);
    swarm_config_.formation = 0;
    swarm_config_.parameters.clear();

    swarm_configs_cache_ = new std::vector<decide_softbus_msgs::SwarmConfig>();
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
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

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }

      ROS_INFO("reconfigure global planner successfully!!!");
    }

    /*if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }*/

    last_config_ = config;
  }

  //void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
  void MoveBase::goalCB(const decide_softbus_msgs::NavigationPoint::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    //move_base_msgs::MoveBaseActionGoal action_goal;
    decide_softbus_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    tf::Stamped<tf::Pose> global_pose;

    //clear the planner's costmap
    planner_costmap_ros_->getRobotPose(global_pose);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.getOrigin().x();
    double y = global_pose.getOrigin().y();
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    controller_costmap_ros_->getRobotPose(global_pose);

    clear_poly.clear();
    x = global_pose.getOrigin().x();
    y = global_pose.getOrigin().y();

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    planner_costmap_ros_->resetLayers();
    controller_costmap_ros_->resetLayers();
    return true;
  }


  //bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
  bool MoveBase::planService(decide_softbus_msgs::GetPlan::Request &req, decide_softbus_msgs::GetPlan::Response &resp){
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose)){
      ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
      return false;
    }
    //geometry_msgs::PoseStamped start;
    decide_softbus_msgs::NavigationPoint start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id == "")
    {
      //tf::poseStampedTFToMsg(global_pose, start);
      tf_bridge::poseStampedTFToMsg(global_pose, start);
    }
    else
      start = req.start;

    //update the copy of the costmap the planner uses
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

    //first try to make a plan to the exact desired goal
    //std::vector<geometry_msgs::PoseStamped> global_plan;
    std::vector<decide_softbus_msgs::NavigationPoint> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance", 
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      //geometry_msgs::PoseStamped p;
      decide_softbus_msgs::NavigationPoint p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                    //(the reachable goal should have been added by the global planner)
                    global_plan.push_back(req.goal);

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    //resp.plan.poses.resize(global_plan.size());
    resp.plan.navigationpoints.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      //resp.plan.poses[i] = global_plan[i];
      resp.plan.navigationpoints[i] = global_plan[i];
    }

    return true;
  }

  bool MoveBase::setSimpleSwarmConfigService(decide_softbus_msgs::SetSwarmConfig::Request &req, decide_softbus_msgs::SetSwarmConfig::Response &resp)
  {
    boost::unique_lock<boost::mutex> lock(swarm_config_mutex_);
    swarm_config_.arrival_time = req.arrival_time;
    swarm_config_.formation = req.formation;
    swarm_config_.parameters = req.parameters;
    lock.unlock();
    ROS_INFO("seted swram config: formation: %d", swarm_config_.formation);
    ROS_INFO("parameters: ");
    for(int i=0; i<swarm_config_.parameters.size();i++)
    {
      ROS_INFO("%d ", swarm_config_.parameters[i]);
    }

    resp.feedback = 1;

    return true;
  }

  bool MoveBase::setControllingService(decide_softbus_msgs::SetControlling::Request &req, decide_softbus_msgs::SetControlling::Response &resp)
  {
      boost::unique_lock<boost::mutex> lock(controlling_mutex_);
      //make sure we only start the controller if we still haven't reached the goal
      if(req.CONTROLLING==1) {
          //start_controlling_ = true;
          //ROS_INFO("set start_controlling_: %d", start_controlling_);

          //publish the controlling msg to the action softbus
          std_msgs::Int32 a_st;
          a_st.data = 1;
          //for(int i=0; i<10; i++)
          action_start_pub_.publish(a_st);
      }

      if(req.CONTROLLING==0){
          /*state_ = WAITING;
          //publishZeroVelocity();

          //publish the controlling msg to the action softbus
          std_msgs::Int32 a_st;
          a_st.data = 0;
          action_start_pub_.publish(a_st);*/
      }
      lock.unlock();

      resp.feedback = 1;
      return true;
  }


  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    //tc_.reset();
  }

  //bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
  bool MoveBase::makePlan(const decide_softbus_msgs::NavigationPoint& goal, std::vector<decide_softbus_msgs::NavigationPoint>& plan){
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    //geometry_msgs::PoseStamped start;
    decide_softbus_msgs::NavigationPoint start;
    //tf::poseStampedTFToMsg(global_pose, start);
    tf_bridge::poseStampedTFToMsg(global_pose, start);

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
    swarm_vel_pub_.publish(cmd_vel);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  //geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
  decide_softbus_msgs::NavigationPoint MoveBase::goalToGlobalFrame(const decide_softbus_msgs::NavigationPoint& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    tf::Stamped<tf::Pose> goal_pose, global_pose;
    //tf::poseStampedMsgToTF(goal_pose_msg, goal_pose);
    tf_bridge::poseStampedMsgToTF(goal_pose_msg, goal_pose);

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.stamp_ = ros::Time();

    try{
      tf_.transformPose(global_frame, goal_pose, global_pose);
    }
    catch(tf::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    //geometry_msgs::PoseStamped global_pose_msg;
    decide_softbus_msgs::NavigationPoint global_pose_msg;
    //tf::poseStampedTFToMsg(global_pose, global_pose_msg);
    tf_bridge::poseStampedTFToMsg(global_pose, global_pose_msg);
    return global_pose_msg;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    failed_plan_count_=0;
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }

      planSuccess_ = false;
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      //geometry_msgs::PoseStamped temp_goal = planner_goal_;
      decide_softbus_msgs::NavigationPoint temp_goal = planner_goal_;
      lock.unlock();
      ROS_INFO("move_base_plan_thread, planning...");

      //run planner
      planner_plan_->clear();
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan){
        //reset failed_plan_count
        failed_plan_count_=0;
      
        //std::vector<geometry_msgs::PoseStamped> rviz_plan;
        geometry_msgs::PoseArray rviz_plan;
        rviz_plan.header.seq=(*planner_plan_)[0].header.seq;
        rviz_plan.header.stamp=(*planner_plan_)[0].header.stamp;
        rviz_plan.header.frame_id=(*planner_plan_)[0].header.frame_id;
        for(int i=0; i<planner_plan_->size(); i++)
        {
            geometry_msgs::Pose tmp_pose;
            //tmp_pose.header.seq=(*planner_plan_)[i].header.seq;
            //tmp_pose.header.stamp=(*planner_plan_)[i].header.stamp;
            //tmp_pose.header.frame_id=(*planner_plan_)[i].header.frame_id;
            
            tmp_pose.position.x=(*planner_plan_)[i].pose.position.x;
            tmp_pose.position.y=(*planner_plan_)[i].pose.position.y;
            tmp_pose.position.z=(*planner_plan_)[i].pose.position.z;
            
            tmp_pose.orientation.x=(*planner_plan_)[i].pose.orientation.x;
            tmp_pose.orientation.y=(*planner_plan_)[i].pose.orientation.y;
            tmp_pose.orientation.z=(*planner_plan_)[i].pose.orientation.z;
            tmp_pose.orientation.w=(*planner_plan_)[i].pose.orientation.w;
            
            rviz_plan.poses.push_back(tmp_pose);
        }
        
        rviz_plan_pub_.publish(rviz_plan);
      
        //set swarm configs in the planner_plan_
        for(int i=0; i<planner_plan_->size(); i++)
        {
           (*planner_plan_)[i].arrival_time = swarm_config_.arrival_time;  //TODO, set this time seperately
           (*planner_plan_)[i].formation = swarm_config_.formation;
           (*planner_plan_)[i].parameters = swarm_config_.parameters;
        }

        // publish the plan, only for visualization purpose
        decide_softbus_msgs::Path gui_plan;
        gui_plan.navigationpoints.resize(planner_plan_->size());

        if(!planner_plan_->empty())
        {
          gui_plan.header.frame_id = (*planner_plan_)[0].header.frame_id;
          gui_plan.header.stamp = (*planner_plan_)[0].header.stamp;
        }

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for(unsigned int i=0; i < planner_plan_->size(); i++){
          //gui_path.poses[i] = path[i];
          gui_plan.navigationpoints[i] = (*planner_plan_)[i];
        }

        swarm_plan_pub_.publish(gui_plan);

        std::cout<<"move_base_plan_thread, Got Plan with "<<((*planner_plan_).size())<<" points!"<<std::endl;

        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        //std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;
        std::vector<decide_softbus_msgs::NavigationPoint>* temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        //if(runPlanner_) {
        //    state_ = CONTROLLING;
        //    state_ = WAITING;
        //    runPlanner_ = false;
        //}
        //if(planner_frequency_ <= 0)
        //  runPlanner_ = false;

        runPlanner_ = false;
        planSuccess_ = true;
        lock.unlock();

        planner_try_best_mutex_.lock();
        planner_try_best_=false;
        planner_try_best_mutex_.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      //else if(state_==PLANNING){  //TODO, need new recovery behavior
      else{
        //ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        //ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit
        lock.lock();
        //if(ros::Time::now() > attempt_end && runPlanner_){
        //try max times to get a path in the planner thread
        failed_plan_count_++;
        if(failed_plan_count_>=planner_try_times_){
          //we'll move into our obstacle clearing mode
          //state_ = CLEARING;
          publishZeroVelocity();
          //recovery_trigger_ = PLANNING_R;
          runPlanner_ = false;
          planSuccess_ = false;
          ROS_INFO("plan failed even though do the best-effort! could not get a valid swarm pre plan. plan thread sleep.");
          planner_try_best_mutex_.lock();
          planner_try_best_=true;
          planner_try_best_mutex_.unlock();

          //publish the controlling msg to the action softbus, stop controlling.
          std_msgs::Int32 a_st;
          a_st.data = -1;
          action_start_pub_.publish(a_st);
        }
        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //setup sleep interface if needed
      if(planner_frequency_ > 0){
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

  //void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  void MoveBase::executeCb(const decide_softbus_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      //as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      as_->setAborted(decide_softbus_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    //geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);
    decide_softbus_msgs::NavigationPoint goal = goalToGlobalFrame(move_base_goal->target_pose);

    //we have a goal so start the planner
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    current_goal_pub_.publish(goal);
    //std::vector<geometry_msgs::PoseStamped> global_plan;
    std::vector<decide_softbus_msgs::NavigationPoint> global_plan;

    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();

    ros::NodeHandle n;
    int loop_count=0;
    ros::Rate r(planner_rate_);
    //while(n.ok()&&loop_count<(planner_patience_*planner_rate_))
    while(n.ok()&&(!planner_try_best_))
    {
      //if(c_freq_change_)
      //{
      //  ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
      //  r = ros::Rate(controller_frequency_);
      //  c_freq_change_ = false;
      //}

      if(as_->isPreemptRequested()){
        if(as_->isNewGoalAvailable()){
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          //move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
          ROS_INFO("new goal preempt");
          resetState();
          decide_softbus_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            //as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            as_->setAborted(decide_softbus_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          recovery_index_ = 0;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          //ROS_INFO("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
        }
        else {
          //if we've been preempted explicitly we need to shut things down
          resetState();

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){

        goal = goalToGlobalFrame(goal);

        //we want to go back to the planning state for the next execution cycle
        recovery_index_ = 0;

        planner_try_best_mutex_.lock();
        planner_try_best_=false;
        planner_try_best_mutex_.unlock();

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
      }

      if(planSuccess_)
      {
          resetState();
          as_->setSucceeded(decide_softbus_msgs::MoveBaseResult(), "Get swarm pre plan successfully.");
          ROS_INFO("Get swarm pre plan successfully.");
          return;
      }

      //for timing that gives real time even in simulation
      //ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      //bool done = executeCycle(goal, global_plan);

      //if we're done, then we'll return from execute
      //if(done)
      //{
      //  return;
      //}

      //check if execution of the goal has completed in some way
      //ros::WallDuration t_diff = ros::WallTime::now() - start;
      //ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

      loop_count++;
      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      //if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
      //  ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //wake up the planner thread so that it can exit cleanly
    //lock.lock();
    //runPlanner_ = true;
    //planner_cond_.notify_one();
    //lock.unlock();

    //if the node is killed then we'll abort and return
    //as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    //as_->setAborted(decide_softbus_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    //ROS_INFO("Aborting on the goal because the plan thread could not get a valid swarm pre plan");
    as_->setAborted(decide_softbus_msgs::MoveBaseResult(), "Aborting on the goal");
    ROS_INFO("Aborting on the goal because the plan thread could not get a valid swarm pre plan.");
    planner_try_best_mutex_.lock();
    planner_try_best_=false;
    planner_try_best_mutex_.unlock();
    return;
  }

  //double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  double MoveBase::distance(const decide_softbus_msgs::NavigationPoint& p1, const decide_softbus_msgs::NavigationPoint& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.", 
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.", 
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(ags_clear);

      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_)
        recovery_behaviors_.push_back(rotate);
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    runPlanner_ = false;
    planSuccess_ = false;
    lock.unlock();

    planner_try_best_mutex_.lock();
    planner_try_best_=false;
    planner_try_best_mutex_.unlock();

    // Reset statemachine
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //Reset the swarm config
    swarm_config_.arrival_time = ros::Time(0);
    swarm_config_.formation = 0;
    swarm_config_.parameters.clear();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }
};
