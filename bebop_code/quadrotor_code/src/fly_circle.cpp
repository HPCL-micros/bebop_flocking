#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
//#include "micros_flocking/Position.h"
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
#include <ctime>
#include <fstream>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include <geometry_msgs/Twist.h>
using namespace std;

float start_yaw;
float current_yaw;

void cb(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr & msg)
{
    static bool is_start=true;
    if(is_start)
    {
        start_yaw=msg->yaw ;
        current_yaw=msg->yaw;
        is_start=false;
    }

    current_yaw=msg->yaw;
}
/*
void hectorcb(const micros_flocking::Position & msg)
{
    static bool is_start=true;
    if(is_start)
    {
        start_yaw=msg->yaw ;
        current_yaw=msg->yaw;
        is_start=false;
    }

    current_yaw=msg->yaw;
}*/

int main(int argc, char** argv)
{
   //srand(time(0));
   ros::init(argc,argv,"fly_circle_node");
   ros::NodeHandle n;
   ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
   ros::Subscriber sub = n.subscribe("/bebop/states/ardrone3/PilotingState/AttitudeChanged", 1000, cb);

   ros::Rate r(10);
   ros::Rate r2(10);
   int count=0;

   bool end = false;
   for(int i=0;i<20;i++)
   {
       r.sleep();
       ros::spinOnce();
   }
   while(ros::ok())
   {
       if(count%30==0)
       {
           //float diff = current_yaw-(start_yaw-1.57);
           if(end)
           break;
           while(current_yaw>start_yaw-1.57)
           {
               geometry_msgs::Twist msg;
               msg.angular.z = 0.2;
               pub.publish(msg);
               r2.sleep();
               ros::spinOnce(); 
               if(current_yaw >0 && start_yaw-1.57 < -3)
               {
                  end = true;break;
               }
           }
           start_yaw = current_yaw;
       }

      
       count++;
       geometry_msgs::Twist msg;
       msg.linear.x = 0.3;
       pub.publish(msg);
       r.sleep();
       ros::spinOnce();
   
   }
   
   ros::spin();
}
