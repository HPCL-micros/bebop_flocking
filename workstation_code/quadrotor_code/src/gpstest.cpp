#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
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

double maxlatitude = 0,maxlongitude = 0;
double minlatitude = 0,minlongitude = 0;

double maxn=0,minn=0,maxe=0,mine=0;

ofstream fout("gpsresult.txt");
pair<double,double> my_position;
void fix_cb(const sensor_msgs::NavSatFix::ConstPtr & msg)
{
    
  geographic_msgs::GeoPoint geo_pt;
  geo_pt.latitude = msg->latitude;
  geo_pt.longitude = msg->longitude;
  geo_pt.altitude = msg->altitude;
  geodesy::UTMPoint utm_pt(geo_pt);
  my_position.first = utm_pt.easting;
  my_position.second = utm_pt.northing;
  cout<<setprecision(10)<<my_position.first<<" "<<my_position.second<<endl;
  
  if( geo_pt.latitude > maxlatitude || maxlatitude ==0)
      maxlatitude = geo_pt.latitude;
  if( geo_pt.latitude < minlatitude || minlatitude ==0)
      minlatitude = geo_pt.latitude;
      
  if( geo_pt.longitude > maxlongitude || maxlongitude ==0)
      maxlongitude = geo_pt.longitude;
  if( geo_pt.longitude < minlongitude || minlongitude ==0)
      minlongitude = geo_pt.longitude;
      
  if( my_position.first > maxe || maxe==0 )
      maxe = my_position.first;
  if( my_position.first < mine || mine==0 )
      mine = my_position.first;
      
  if( my_position.second > maxn || maxn==0 )
      maxn = my_position.first;
  if( my_position.first < minn || minn==0 )
      minn = my_position.first;
      
  fout<<my_position.first<<" "<<my_position.second<<" "<<msg->altitude<<endl;
}


int main(int argc, char** argv)
{
   srand(time(0));
   ros::init(argc,argv,"gpstest_node");
   ros::NodeHandle n;
   //ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
   ros::Subscriber sub = n.subscribe("fix", 1000, fix_cb);
   ros::spin();
}
