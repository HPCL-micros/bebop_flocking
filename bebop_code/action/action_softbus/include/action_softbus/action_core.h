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
#ifndef ACTION_CORE_H_
#define ACTION_CORE_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <tf_bridge/tf_bridge.h>

#include <actionlib/server/simple_action_server.h>
#include <decide_softbus_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
//#include <nav_msgs/GetPlan.h>
#include <decide_softbus_msgs/GetPlan.h>
#include <decide_softbus_msgs/Path.h>
#include <decide_softbus_msgs/SwarmConfig.h>
#include <decide_softbus_msgs/SetSwarmConfig.h>
#include <decide_softbus_msgs/SetControlling.h>

#include <tf/transform_listener.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>

namespace action_softbus {

    /**
     * action interface
     */
    class ActionCore {
    public:
        virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) = 0;
        virtual bool setPath(const std::vector<decide_softbus_msgs::NavigationPoint>& path) = 0;
        //virtual geometry_msgs::Twist::ConstPtr computeVelCmd(const geometry_msgs::Twist::ConstPtr &swarm_vel)=0;
        virtual bool computeVelCmd(geometry_msgs::Twist& vel_cmd)=0;
        virtual bool isGoalReached() = 0;
        virtual ~ActionCore(){}
    protected:
        ActionCore(){}
    private:
        std::string name_;
        std::string global_frame_;

        costmap_2d::Costmap2D* costmap_;
        tf::TransformListener* tf_;

        decide_softbus_msgs::Path path_;

        bool initialized_;
    };
};
#endif

