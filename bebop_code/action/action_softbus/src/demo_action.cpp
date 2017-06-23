#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sstream>
#include <iostream>

#include "std_msgs/String.h"

#include <pluginlib/class_list_macros.h>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <action_softbus/action_core.h>
#include "action_softbus/demo_action.h"

// Register the nodelet
PLUGINLIB_EXPORT_CLASS(action_softbus::DemoAction, action_softbus::ActionCore)


namespace action_softbus{

    DemoAction::DemoAction() {}

    void DemoAction::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ros::NodeHandle nh;
        //ros::Subscriber swarm_plan_sub_;
        //leader_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("/leader_cmd_vel", 1000, boost::bind(&DemoAction::leaderVelCB, this, _1));
        leader_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("/leader_cmd_vel", 1000, &DemoAction::leaderVelCB, this);
    }

    void DemoAction::leaderVelCB(const geometry_msgs::Twist::ConstPtr& swarm_vel)
    {
        leader_vel_queue_.push(*swarm_vel);
    }


    bool DemoAction::setPath(const std::vector<decide_softbus_msgs::NavigationPoint>& path)
    {
        path_ = path;
        return true;
    }

    bool DemoAction::computeVelCmd(geometry_msgs::Twist& vel_cmd)
    {
        /*if(!leader_vel_queue_.empty()){
            vel_cmd = leader_vel_queue_.front();
            leader_vel_queue_.pop();
            return true;
        }

        return false;
         */
        static int count=0;
        double mypx = micros_flock::my_position.first;
        double mypy = micros_flock::my_position.second;
        int index =0;
        double vpx = path_[0].pose.position.x;
        double vpy = path_[0].pose.position.y;
        double mindist = (mypx-vpx)*(mypx-vpx)+(mypy-vpy)*(mypy-vpy);
        /*
        for(int i=1;i<path_.size();i++)
        {
            vpx = path_[i].pose.position.x;
            vpy = path_[i].pose.position.y;
            double dist = (mypx-vpx)*(mypx-vpx)+(mypy-vpy)*(mypy-vpy);
            if(dist<mindist)
            {
                mindist=dist;index=i;
            }
        }*/
        index = count / 2;
        if(index > path_.size()-1) index =path_.size()-1;
        double vvx=0,vvy=0;
        if(index!=path_.size()-1)
        {
            vvx = path_[index+1].pose.position.x-path_[index].pose.position.x;
            vvy = path_[index+1].pose.position.y-path_[index].pose.position.y;
            double scale = sqrt(vvx*vvx+vvy*vvy)+0.0001;
            vvx=vvx/scale*10;
            vvy=vvy/scale*10;
            cout<<scale<<endl;
        }
        vpx = path_[index].pose.position.x;
        vpy = path_[index].pose.position.y;
        //cout<<vpx<<' '<<vpy<<' '<<vvx<<' '<<vvy<<endl;
        fa.setVL(vpx,vpy,vvx,vvy);
        
        fa.run(vel_cmd);
        count++;
        return true;
    }

    bool DemoAction::isGoalReached()
    {
        return false;
    }
}

