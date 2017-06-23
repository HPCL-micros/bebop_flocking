#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
#include <ctime>
#include <geodesy/utm.h>
#include <fstream>
#include <iomanip>
using namespace std;


int main(int argc, char** argv)
{
   srand(time(0));
   ros::init(argc,argv,"gpstest_node");
   ros::NodeHandle n;
   ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
   geometry_msgs::Twist msg;
   ros::Rate loop_rate(10);
   for(int i=0;i<10;i++)
   {
   loop_rate.sleep();
   pub.publish(msg);
   }
   //ros::spin();
}
