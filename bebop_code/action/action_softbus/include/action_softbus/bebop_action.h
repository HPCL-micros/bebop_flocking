#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sstream>
#include <iostream>
#include <queue>

#include "std_msgs/String.h"

#include <pluginlib/class_list_macros.h>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <action_softbus/action_core.h>

#include "fix_alpha.h"

namespace action_softbus{

    class BebopAction : public ActionCore
    {
    public:
        BebopAction();
        virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
        virtual bool setPath(const std::vector<decide_softbus_msgs::NavigationPoint>& path);
        virtual bool computeVelCmd(geometry_msgs::Twist& vel_cmd);
        virtual bool isGoalReached();

        void leaderVelCB(const geometry_msgs::Twist::ConstPtr& swarm_vel);
    private:
        std::vector<decide_softbus_msgs::NavigationPoint> path_;
        ros::Subscriber leader_vel_sub_;
        std::queue<geometry_msgs::Twist> leader_vel_queue_;
        bebop_flock::BebopFlockingAlgorithm fa;
    };
}//namespace nodelet_test

