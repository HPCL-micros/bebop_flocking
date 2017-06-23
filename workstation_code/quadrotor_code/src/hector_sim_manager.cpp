#define TF_EULER_DEFAULT_ZYX

#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "decide_softbus_msgs/NavigationPoint.h"
#include "geometry_msgs/PointStamped.h"
#include <decide_softbus_msgs/SetControlling.h>
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateSpeedChanged.h"
#include "quadrotor_code/Status.h"
#include "quadrotor_code/Neighbor.h"
#include <tf/LinearMath/Quaternion.h>
#include <geodesy/utm.h>
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseArray.h"
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
#include <ctime>
#include <fstream>
using namespace std;
#define R 7
#define sendT 1
#define perror 0  //value
#define verror 0 //rate
double PI=acos(-1);

double delta_dis=5; //无人机之间的间距
int stand_vx=0, stand_vy=0;//作为标准的无人机的虚拟坐标
double stand_x, stand_y;//作为标准的无人机的实际坐标

class OdomHandle
{
    public:
    //ros::Subscriber sub_yaw,sub_vel,sub_fix;
    ros::Subscriber sub,sub_fix;
    ros::Publisher pub;
    ros::Publisher neighbor_pub;
    double _px,_py,_vx,_vy;
    double _pz,_vz,_theta;
    int _r_id; 
    int count;
    bool yawrcv,velrcv,fixrcv;

    double start_x, start_y;//起始位置坐标
    int vx, vy;//在虚拟坐标系中的坐标
    double delta_x,delta_y;//误差向量
    
    double qx,qy,qz,qw;//orientation
    
   
    OdomHandle(int r_id)
    {
        ros::NodeHandle n;
        yawrcv = false;
        velrcv = false;
        fixrcv = false;
        /*
        stringstream ss;
        ss<<"/uav"<<r_id<<"/states/ardrone3/PilotingState/AttitudeChanged";
        sub_yaw = n.subscribe(ss.str(), 1000, &OdomHandle::yawcb,this);
 
        stringstream ss1;
        ss1<<"/uav"<<r_id<<"/states/ardrone3/PilotingState/SpeedChanged";
        //pub = n.advertise<micros_flocking::Position>(ss1.str(),1000);
        sub_vel = n.subscribe(ss1.str(), 1000, &OdomHandle::velcb,this);

        stringstream ss2;
        ss2<<"/uav"<<r_id<<"/fix";
        sub_fix = n.subscribe(ss2.str(), 1000, &OdomHandle::fixcb,this);
        */
        stringstream ss;
        ss<<"/uav"<<r_id<<"/ground_truth/state";
        sub = n.subscribe(ss.str(), 1000, &OdomHandle::cb,this);
        
        stringstream ss2;
        ss2<<"/uav"<<r_id<<"/fix";
       // sub_fix = n.subscribe(ss2.str(), 1000, &OdomHandle::fixcb,this);
        
        stringstream ss3;
        ss3<<"/uav"<<r_id<<"/position";
        pub = n.advertise<quadrotor_code::Status>(ss3.str(),1000);
        
        stringstream ss4;
        ss4<<"/uav"<<r_id<<"/neighbor";
        neighbor_pub = n.advertise<quadrotor_code::Neighbor>(ss4.str(),1000);
        _px=0;
        _py=0;
        _vx=0;
        _vy=0;
        _pz=0;
        _vz=0;
        _r_id = r_id;
        vx=r_id/3;
        vy=r_id%3;
        
       
       count =0;
       qx=0;qy=0;qz=0;qw=1;
    }
    
    void fixcb(const sensor_msgs::NavSatFix::ConstPtr & msg)
    {
        geographic_msgs::GeoPoint geo_pt;
        geo_pt.latitude = msg->latitude;
        geo_pt.longitude = msg->longitude;
        geo_pt.altitude = msg->altitude;
        geodesy::UTMPoint utm_pt(geo_pt);
        _py = utm_pt.easting;
        _px = utm_pt.northing;

        
        //if(_r_id==2)
        //cout<<count<<endl;
        if(count<30)
        {
            start_x=utm_pt.northing;
            start_y=utm_pt.easting;

            if(vx==stand_vx && vy==stand_vy && count==15)
            {
                stand_x=start_x;
                stand_y=start_y;
            }

            if(count==29)
            {
                delta_x=((vx-stand_vx)*delta_dis+stand_x)-start_x;
                delta_y=((vy-stand_vy)*delta_dis+stand_y)-start_y;
                //if(_r_id ==2)
                //cout<<delta_x<<" "<<delta_y<<endl;
                cout<<"uav"<<_r_id<<" aligned"<<endl;
            }

            count++;
        }
        else
        {
           _py = _py+delta_y - stand_y;
           _px = _px+delta_x - stand_x;
        }

        fixrcv = true;
        _py = -_py;
    }
    
     void cb(const nav_msgs::Odometry::ConstPtr & msg)
    {
        //cout<<this->_px<<" "<<_r_id<<" "<<_vy<<endl;
       
        _px=msg->pose.pose.position.x;
        _py=msg->pose.pose.position.y;
  
        _vx=msg->twist.twist.linear.x;
        _vy=msg->twist.twist.linear.y;

        //_position.first=_px;_position.second=_py;
        //_velocity.first=_vx;_velocity.second=_vy;
        _theta = tf::getYaw(msg->pose.pose.orientation);
        
        tf::Quaternion q1(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
        
        tf::Matrix3x3 m(q1);
        double ro,pi,ya;
        m.getRPY(ro,pi,ya);
        qx= msg->pose.pose.orientation.x;
        qy = msg->pose.pose.orientation.y;
        qz = msg->pose.pose.orientation.z;
        qw = msg->pose.pose.orientation.w;
    }
};

static vector<OdomHandle*> odom_list;
int robotnum=50;
vector<vector<int> > adj_list;

double dist(int i,int j)
{
    double re=pow(odom_list[i]->_px-odom_list[j]->_px,2)+pow(odom_list[i]->_py-odom_list[j]->_py,2);
    //if(i==4)
    //cout<<re<<endl;
    return sqrt(re);
    
}

ros::Publisher rviz_goal_pub_;

void rvizGoalCb(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    decide_softbus_msgs::NavigationPoint tmp_goal;
    tmp_goal.header.seq=msg->header.seq;
    tmp_goal.header.stamp=msg->header.stamp;
    tmp_goal.header.frame_id=msg->header.frame_id;
            
    tmp_goal.pose.position.x=msg->pose.position.x;
    tmp_goal.pose.position.y=msg->pose.position.y;
    tmp_goal.pose.position.z=msg->pose.position.z;
            
    tmp_goal.pose.orientation.x=msg->pose.orientation.x;
    tmp_goal.pose.orientation.y=msg->pose.orientation.y;
    tmp_goal.pose.orientation.z=msg->pose.orientation.z;
    tmp_goal.pose.orientation.w=msg->pose.orientation.w;
    
    rviz_goal_pub_.publish(tmp_goal);
}

ros::ServiceClient rviz_start_action_client_;

void rvizStartActionCb(const geometry_msgs::PointStamped::ConstPtr & msg)
{
    decide_softbus_msgs::SetControlling srv;
    srv.request.CONTROLLING = 1;
    if (rviz_start_action_client_.call(srv))
    {
        ROS_INFO("start action...");
    }
    else
    {
        ROS_ERROR("Failed to call service: /uav0/move_base/set_controlling");
    }
}

int main(int argc, char** argv)
{

   ros::init(argc,argv,"sim_manager");
   ros::NodeHandle n;
   srand(time(0));
   bool param_ok = ros::param::get ("~robotnum", robotnum);
   //ofstream fout("/home/liminglong/czx/traject.txt");
   //ofstream fout2("/home/liminglong/czx/velocity.txt");
   
   ros::Publisher posepub = n.advertise<geometry_msgs::PoseArray>("/swarm_pose",1000);
   rviz_goal_pub_ = n.advertise<decide_softbus_msgs::NavigationPoint>("/uav0/move_base_simple/goal", 0 );
   ros::Subscriber rviz_sub = n.subscribe<geometry_msgs::PoseStamped>("/rviz_simple/goal", 1000, rvizGoalCb);
   
   rviz_start_action_client_= n.serviceClient<decide_softbus_msgs::SetControlling>("/uav0/move_base/set_controlling");
   ros::Subscriber rviz_start_action_sub = n.subscribe<geometry_msgs::PointStamped>("/rviz_simple/start_action", 1000, rvizStartActionCb);
   
   tf::TransformBroadcaster br;
   for(int i=0;i<robotnum;i++)
   {
      OdomHandle *p=new OdomHandle(i);
      //int x=i/3;
      //int y=i%3;
      //p->vx=x;
     // p->vy=y;

      odom_list.push_back(p);
      adj_list.push_back(vector<int>());
   }
   //neighbor_list.push_back(NeighborHandle(1));
   ros::Rate loop_rate(20);
   int count = 1;
   while(ros::ok())
   {
      geometry_msgs::PoseArray sendpose;
       sendpose.header.frame_id="world";
      for(int i=0;i<robotnum;i++)
      {
          geometry_msgs::Pose p;
         
          
          p.position.x = odom_list[i]-> _px;
          p.position.y = odom_list[i]-> _py;
          p.position.z = odom_list[i]-> _pz;
          
          p.orientation.x = odom_list[i] -> qx;
          p.orientation.y = odom_list[i] -> qy;
          p.orientation.z = odom_list[i] -> qz;
          p.orientation.w = odom_list[i] -> qw;
          sendpose.poses.push_back(p);
      }
      posepub.publish(sendpose);
      ros::spinOnce();
      for(int i=0;i<robotnum;i++)
      {
           for(int j=i+1;j<robotnum;j++)
           {
                if(dist(i,j)>0&&dist(i,j)<R)
                {
                    adj_list[i].push_back(j);
                    adj_list[j].push_back(i);
                }
           }
           quadrotor_code::Status sendmsg;
           sendmsg.px = odom_list[i]-> _px;
           sendmsg.py = odom_list[i]-> _py;
           sendmsg.vx = odom_list[i]-> _vx;
           sendmsg.vy = odom_list[i]-> _vy;
           sendmsg.theta =  odom_list[i]-> _theta;
           odom_list[i]->pub.publish(sendmsg);
      }
      
      for(int i=0;i<robotnum;i++)
      {
           quadrotor_code::Neighbor sendmsg;
           sendmsg.data = adj_list[i];
           odom_list[i]->neighbor_pub.publish(sendmsg);
           adj_list[i]=vector<int>();
      }
      
      //publish tf map->uavx/odom (aligned)
      for(int i=0;i<robotnum;i++)
      {
          tf::Transform transform;
          transform.setOrigin( tf::Vector3(0.0,0.0 , 0.0) );
          tf::Quaternion q;
          q.setRPY(0, 0, 0);
          transform.setRotation(q);
           stringstream ss;
          ss<<"uav"<<i<<"/odom";
          br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", ss.str().c_str()));
      }
      
      //publish tf map->world (alogned)
      {
          tf::Transform transform;
          transform.setOrigin( tf::Vector3(0.0,0.0 , 0.0) );
          tf::Quaternion q;
          q.setRPY(0, 0, 0);
          transform.setRotation(q);
           
          br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "world"));
      }
      /*
      //publish uavx/odom->uavx/base_link
      for(int i=0;i<robotnum;i++)
      {
          tf::Transform transform;
          transform.setOrigin( tf::Vector3(odom_list[i]->_px,odom_list[i]->_py , odom_list[i]->_pz) );
          tf::Quaternion q;
          q.setRPY(0, 0, odom_list[i]->_theta);
          transform.setRotation(q);
          
          stringstream ss;
          ss<<"uav"<<i<<"/odom";
           stringstream ss1;
          ss1<<"uav"<<i<<"/base_link";
          br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), ss.str().c_str(), ss1.str().c_str()));
          //cout<<"sent"<<endl;
      }*/
      count++;
      loop_rate.sleep();
   }
   return 0;
}
