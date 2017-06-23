#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "quadrotor_code/Neighbor.h"
//#include "micros_flocking/Position.h"
//#include "micros_flocking/Gradient.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "quadrotor_code/Status.h"
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
#include <ctime>
#include <boost/thread/thread.hpp> 
#include <geodesy/utm.h>
using namespace std;
bool simulation = false;
double PI=acos(-1);
#define EPSILON 0.1
#define A 5
#define B 5
const double C = abs(A-B) / sqrt(4*A*B);
#define H 0.2
#define D 5
#define R 7
#define C1 0.6
#define C2 0.6
#define rspeedlimit 0.3
double basespeed = 0;
double baseomega = 0;
double base_angle = 0;
double vlvx = 0;
double vlvy = 0;
double targetx = 0;//477508;
double targety = 0;//5523144;
#define ploss 0 //max1000
#define diffdrive false
#define max_turn 0.7
bool move_vl = false;
bool neighbor_loss=false;
int hz=10;
bool delay_enabled = false;
int delay_time = 200;

#define krho (0.3*3)
double kalpha =0.8*3;
#define kbeta (-0.15*3)
double keepdistance = sqrt(rspeedlimit/krho);


double   interval=1.0/hz;
double pm1=0.03,pm2=0.3,pm3=0.05;
class NeighborHandle
{
    public:
    //ros::Subscriber sub,sub2;
    ros::Subscriber subStatus;
    double _px,_py,_vx,_vy;
    pair<double,double> _position,_velocity;
    int _r_id; 
    double mypm_g;
    int gradient;
    NeighborHandle(int r_id)
    {
        ros::NodeHandle n;
        stringstream ss;
        ss<<"/uav"<<r_id<<"/position";
        subStatus = n.subscribe(ss.str(), 1000, &NeighborHandle::statuscb,this);
        /*
        ss<<"/uav"<<r_id<<"/fix";
        sub = n.subscribe(ss.str(), 1000, &NeighborHandle::fixcb,this);
        stringstream ss1;
        ss1<<"/uav"<<r_id<<"/ground_truth/state";
        sub2 = n.subscribe(ss1.str(), 1000, &NeighborHandle::velcb,this);*/
        _px=0;
        _py=0;
        _vx=0;
        _vy=0;
        _position=pair<double,double>(0,0);
        _velocity=pair<double,double>(0,0);
        _r_id = r_id;
        mypm_g=1;
        gradient = -1;
    }
    /*
    void cb(const micros_flocking::Position::ConstPtr & msg)
    {
        //cout<<this->_px<<" "<<_r_id<<" "<<_vy<<endl;
        //_py=1;_px=1;
        if(rand()%1000<ploss)
             return;
        
        _px=msg->px;
        _py=msg->py;
        _vx=msg->vx;
        _vy=msg->vy;
        _position.first=_px;_position.second=_py;
        _velocity.first=_vx;_velocity.second=_vy;
        gradient=msg->gradient;
        //cout<<"neighbor pose updated"<<endl;
        //_vx=1;_vy=1;//myx=1;myy=1;
        //cout<<_r_id<<endl;
    }
    
    void fixcb(const sensor_msgs::NavSatFix::ConstPtr & msg)
    {
        
        geographic_msgs::GeoPoint geo_pt;
        geo_pt.latitude = msg->latitude;
        geo_pt.longitude = msg->longitude;
        geo_pt.altitude = msg->altitude;
        geodesy::UTMPoint utm_pt(geo_pt);
        _px = utm_pt.easting;
        _py = utm_pt.northing;
        //_px=msg->px;
        //_py=msg->py;
        //_vx=msg->vx;
        //_vy=msg->vy;
        _position.first=_px;_position.second=_py;
        //_velocity.first=_vx;_velocity.second=_vy;

    }
    
    void velcb(const nav_msgs::Odometry::ConstPtr & msg)
    {
        
        //_px=msg->pose.pose.position.x;
        //_py=msg->pose.pose.position.y;
        //_pz=msg->pose.pose.position.z;
        _vx=msg->twist.twist.linear.x;
        _vy=msg->twist.twist.linear.y;
        //_vz=msg->twist.twist.linear.z;
        _position.first=_px;_position.second=_py;
        _velocity.first=_vx;_velocity.second=_vy;

        
    }*/
    
    void statuscb(const quadrotor_code::Status::ConstPtr & msg)
    {
        _px = msg -> px;
        _py = msg -> py;
        _vx = msg -> vx;
        _vy = msg -> vy;
        _position.first=_px;_position.second=_py;
        _velocity.first=_vx;_velocity.second=_vy;
    }
    
    void close()
    {
        subStatus.shutdown();
    }
};
static list<NeighborHandle*> neighbor_list;
pair<double,double> my_vpoint_position=pair<double,double>(0,0);
pair<double,double> my_vpoint_velocity=pair<double,double>(0,0);
pair<double,double> my_position=pair<double,double>(0,0);
pair<double,double> my_velocity=pair<double,double>(0,0);
double my_theta = 0;
bool findInMyList(int r_id)
{
    for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
    {
        if (r_id == (*i)->_r_id)
            return true;
    }
    return false;
}

bool findInVector(int r_id,vector<int> v)
{
    for(int i=0;i<v.size();i++)
        if(r_id == v[i])
            return true;
    return false;
}

static void neighbor_cb(const quadrotor_code::Neighbor::ConstPtr & msg)
{
    if(rand()%1000<ploss && neighbor_loss)
             return;
    for(int i=0;i< msg->data.size();i++)
        if (!findInMyList(msg->data[i]))
        {
            //neighbor_list.push_back(NeighborHandle(msg->data[i]));
            NeighborHandle* x= new NeighborHandle(msg->data[i]);
            neighbor_list.push_back(x);
            //cout<<"add"<<endl;
        }
    for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
    {
        if(!findInVector((*i)->_r_id,msg->data))
        {
            (*i)->close();
            delete (*i);
            i=neighbor_list.erase(i);
            //cout<<"close"<<endl;
        }
    }
    //cout<<"exit callback"<<endl;
    
}

double my_gradient = -1;

void my_vpoint_position_cb(const quadrotor_code::Status::ConstPtr & msg)
{
    my_vpoint_position.first = msg->px;
    my_vpoint_position.second = msg->py;
    my_vpoint_velocity.first = msg->vx;
    my_vpoint_velocity.second = msg->vy;
    //my_theta = msg->theta;
    //my_gradient = msg->gradient;
    //cout<<"my pose updated"<<endl;
}


void my_theta_cb(const quadrotor_code::Status::ConstPtr & msg)
{

    my_position.first = msg->px;
    my_position.second = msg->py;
    my_velocity.first = msg->vx;
    my_velocity.second = msg->vy;
    
    my_theta = msg->theta;
    //my_gradient = msg->gradient;
    //cout<<"my pose updated"<<endl;
}

/*
void my_position_cb(const sensor_msgs::NavSatFix::ConstPtr & msg)
{
    
  geographic_msgs::GeoPoint geo_pt;
  geo_pt.latitude = msg->latitude;
  geo_pt.longitude = msg->longitude;
  geo_pt.altitude = msg->altitude;
  geodesy::UTMPoint utm_pt(geo_pt);
  my_position.first = utm_pt.easting;
  my_position.second = utm_pt.northing;
}*/

pair<double,double> get_vector(pair<double,double> start,pair<double,double> end)
{
    pair<double,double> re=pair<double,double>(0,0);
    re.first=end.first-start.first;
    re.second=end.second-start.second;
    return re;
}

double segma_norm(pair<double,double> v)
{
    double re = EPSILON*(v.first*v.first+v.second*v.second);
    re = sqrt(1+re)-1;
    re /= EPSILON;
    return re;
}

double R_alpha = segma_norm(pair<double,double>(R,0));
double D_alpha = segma_norm(pair<double,double>(D,0));

pair<double,double> segma_epsilon(pair<double,double> v)
{
    pair<double,double> re = pair<double,double>(0,0);
    double scale = 1+EPSILON*(v.first*v.first+v.second*v.second);
    scale = sqrt(scale);
    re.first = v.first / scale;
    re.second = v.second / scale;
    return re;
}

double segma_1(double z)
{
    return z / sqrt(1+z*z);
}

double phi(double z)
{
    return 0.5*((A+B)*segma_1(z+C)+A-B);
}

double rho(double z)
{
    if(z<H)
        return 1;
    if(z>1)
        return 0;
    return 0.5*(1+cos(PI*(z-H)/(1-H)));
}

double phi_alpha(double z)
{
    return rho(z/R_alpha)*phi(z-D_alpha);
}

pair<double,double> f_g()
{
    pair<double,double> re = pair<double,double>(0,0);
    for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
    {
        pair<double,double> q_ij = get_vector(my_vpoint_position,(*i)->_position);
        if(q_ij.first*q_ij.first+q_ij.second*q_ij.second >R*R)
        continue;
        pair<double,double> n_ij = segma_epsilon(q_ij);
        re.first += phi_alpha(segma_norm(q_ij))*n_ij.first*(*i)->mypm_g;
        re.second += phi_alpha(segma_norm(q_ij))*n_ij.second*(*i)->mypm_g;
    }
    return re;
}

double a_ij(pair<double,double> j_p)
{
    return rho(segma_norm(get_vector(my_vpoint_position,j_p)) / R_alpha);
}

pair<double,double> f_d()
{
    pair<double,double> re = pair<double,double>(0,0);
   // int count=0;
    for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
    {
        pair<double,double> q_ij = get_vector(my_vpoint_position,(*i)->_position);
        pair<double,double> p_ij = get_vector(my_vpoint_velocity,(*i)->_velocity);
        if(q_ij.first*q_ij.first+q_ij.second*q_ij.second >R*R)
        continue;
        re.first += a_ij((*i)->_position) * p_ij.first;
        re.second += a_ij((*i)->_position) * p_ij.second;
       // count++;
    }
    //cout<<count<<endl;
    return re;
}

pair<double,double> q_r = pair<double,double>(targetx,targety);
pair<double,double> p_r = pair<double,double>(basespeed,basespeed);
pair<double,double> f_r()
{
    pair<double,double> re = pair<double,double>(0,0);
    
    
   
    //cout<<q_r.first<<' '<<q_r.second;
    re.first = -C1*get_vector(q_r,my_vpoint_position).first - C2*get_vector(p_r,my_vpoint_velocity).first;
    re.second = -C1*get_vector(q_r,my_vpoint_position).second - C2*get_vector(p_r,my_vpoint_velocity).second;
    //cout<<q_r.first<<' '<<q_r.second<<' '<<get_vector(q_r,my_vpoint_position).first<<' '<<get_vector(q_r,my_vpoint_position).second<<endl;
    return re;
}

void update_param()
{
    double mindist=2*R;
    for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
    {
        pair<double,double> q_ij = get_vector(my_vpoint_position,(*i)->_position);
        double dist = sqrt(q_ij.first*q_ij.first+q_ij.second*q_ij.second);
        if(mindist>dist)
             mindist=dist;/*
        if(dist<0.9D)
        {
             (*i)->mypm_g=1+(1*D-mindist)/D *2;
             
         }
         else
             (*i)->mypm_g=1;*/
         //cout<<dist<<endl;
        // cout<<(*i)->mypm_g<<endl;
    }
    if (mindist<0.9*D)
        pm1=1+(1*D-mindist)/D *10;
    else
        pm1=1;
}

void spin_thread()
{
    while(ros::ok())
    {
       if(delay_enabled)
        {
           int sleep_sec = delay_time;
            boost::this_thread::sleep(boost::posix_time::millisec(sleep_sec));
        }
       else
       {
             ros::Rate loop_rate(100);
             loop_rate.sleep();
        }
       ros::spinOnce();
    }
}



class diff_tracking_thread
{
    public:
    ros::Publisher * pub;
    diff_tracking_thread(ros::Publisher * p)
    {
        pub = p;
    }
    void operator() ()
    {
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
         double delta_x = my_vpoint_position.first - my_position.first;
          double delta_y = my_vpoint_position.second - my_position.second;
          
          double alpha = atan(delta_y/delta_x) - my_theta;
          double beta = - alpha - my_theta;
          if(my_vpoint_position.first == my_position.first)
          {
              loop_rate.sleep();continue;
          }
          else if(delta_x < 0.0)
          {
              double r_theta = PI - my_theta;//if my_theta >0
              if(my_theta < 0.0)
                 r_theta = -PI - my_theta;
              alpha = -atan(delta_y/delta_x) - r_theta;
              beta = - alpha - r_theta;
          }
          double rho = sqrt(delta_x*delta_x+delta_y*delta_y);
          geometry_msgs:: Twist senddiffmsg;
          senddiffmsg.linear.x = krho*rho;
          senddiffmsg.angular.z = kalpha*alpha + kbeta*beta;

          if(delta_x < 0.0)
              senddiffmsg.angular.z = - senddiffmsg.angular.z;
          pub->publish(senddiffmsg);
          loop_rate.sleep();
          
    }
    }
};
int main(int argc, char** argv)
{
   srand(time(0));
   ros::init(argc,argv,"turtlebot_roll_node");
   ros::NodeHandle n;
   ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
   ros::Subscriber sub = n.subscribe("neighbor", 1000, neighbor_cb);
  //test ros::Subscriber sub_vpoint_pos = n.subscribe("vpoint_position", 1000, my_vpoint_position_cb);
   //ros::Subscriber sub_pos = n.subscribe("fix", 1000, my_position_cb);
   ros::Subscriber sub_theta = n.subscribe("position", 1000, my_theta_cb);
  
  // ros::Publisher gradient_pub = n.advertise<micros_flocking::Gradient>("gradient",1000);
   ros::Publisher vpoint_pub = n.advertise<quadrotor_code::Status>("vpoint_position",1000);

   ros::Rate loop_rate(hz);
   //ros::spin();
   geometry_msgs:: Twist msg;
   //msg.linear.x=(rand()%100)/100.0;
   //msg.linear.y=(rand()%100)/100.0;
   geometry_msgs:: Twist sendmsg;
   geometry_msgs:: Twist lastmsg;
   boost::thread thrd(&spin_thread);

   int time_count=0;
   bool diff_thread_ok = false;

   p_r.first = vlvx + basespeed * cos(base_angle);
   p_r.second = vlvy + basespeed * sin(base_angle);
   if(simulation){
   for(int i=0;i<3*hz;i++)
   {
         geometry_msgs:: Twist upmsg;
         upmsg.linear.z=1;
         pub.publish(upmsg);
         loop_rate.sleep();
   }
   }
   while(my_position.first == 0)
   {
       ros::spinOnce();
       loop_rate.sleep();
   }
   my_vpoint_position.first = my_position.first+keepdistance;
   my_vpoint_position.second = my_position.second;//+keepdistance;
   my_vpoint_velocity.first = my_velocity.first;
   my_vpoint_velocity.second = my_velocity.second;
   
  
   
   while(ros::ok())
   {
      
      //ros::spinOnce();
      //update_param();
      //cout<<pm1<<endl;
       if(time_count>=25*hz && time_count<=25*hz+10*hz)
      baseomega = PI/2.0/10;//PI/30;
      else if(time_count >= 50*hz && time_count <= 50*hz+10*hz)
      baseomega = PI/2.0/10;
      else if (time_count >= 80*hz && time_count<=80*hz+20*hz)
      baseomega = PI/2.0/10;
      else if (time_count >=105*hz)
      basespeed=0;
      else
      baseomega =0;
      
       my_vpoint_position.first += my_vpoint_velocity.first / (double)hz;
      my_vpoint_position.second += my_vpoint_velocity.second / (double)hz;
      if(!diffdrive){
          my_vpoint_position.first = my_position.first;
          my_vpoint_position.second = my_position.second;
      }
      pair<double,double> temp = f_r();
      //if(my_gradient <= 10){
      msg.linear.x += (f_g().first*pm1+f_d().first*pm2+temp.first*pm3)/hz;
      cout<<f_g().first*pm1<<' '<<f_d().first*pm2<<' '<<temp.first*pm3<<endl;
      msg.linear.y += (f_g().second*pm1+f_d().second*pm2+temp.second*pm3)/hz;//}
     /*else
      {
          msg.linear.x += (f_g().first*pm1+f_d().first*pm2)/hz;
          msg.linear.y += (f_g().second*pm1+f_d().second*pm2)/hz;
       }*/

       if(move_vl)
       {
           p_r.first = 1;
           if(time_count>10*120)
               p_r.second = -1;
       }
     // cout<<f_g().first<<' '<<f_d().first<<' '<<msg.linear.x<<endl;
      //cout<<f_g().second<<' '<<f_d().second<<' '<<msg.linear.y<<endl;
     /*
      if (msg.linear.x >rspeedlimit)
         if(abs(msg.linear.x) >= abs(msg.linear.y))
         {
             msg.linear.x=rspeedlimit;
             msg.linear.y=msg.linear.y/msg.linear.x*rspeedlimit;
         }
         
      if (msg.linear.x <-rspeedlimit)
         if(abs(msg.linear.x) >= abs(msg.linear.y))
         {
             msg.linear.x=-rspeedlimit;
             msg.linear.y=msg.linear.y/abs(msg.linear.x)*rspeedlimit;
         }
      if (msg.linear.y >rspeedlimit)
         if(abs(msg.linear.y) >= abs(msg.linear.x))
         {
             msg.linear.y=rspeedlimit;
             msg.linear.x=msg.linear.x/msg.linear.y*rspeedlimit;
         }
      if (msg.linear.y <-rspeedlimit)
         if(abs(msg.linear.y) >= abs(msg.linear.x))
         {
             msg.linear.y=-rspeedlimit;
             msg.linear.x=msg.linear.x/abs(msg.linear.y)*rspeedlimit;
         }*/
      if(abs(msg.linear.x) > rspeedlimit || abs(msg.linear.y) > rspeedlimit)
      {
          double maxabs = abs(msg.linear.x);
          if(maxabs< abs(msg.linear.y))
              maxabs = abs(msg.linear.y);
          msg.linear.x = msg.linear.x / maxabs * rspeedlimit;
          msg.linear.y = msg.linear.y / maxabs * rspeedlimit;
      }
      
      base_angle += baseomega /hz;
      p_r.first = vlvx + basespeed * cos(base_angle);
      p_r.second = vlvy + basespeed * sin(base_angle);
       q_r.first += p_r.first * interval;
       q_r.second += p_r.second * interval;
//       cout<<q_r.first<<' '<<q_r.second<<endl;
      sendmsg.linear.x = basespeed * cos(base_angle);
      sendmsg.linear.y = basespeed * sin(base_angle);
      //sendmsg.linear.x = basespeed;
      //sendmsg.linear.y = basespeed;
//      cout<<sendmsg.linear.x<<" and "<<msg.linear.x<<endl;
      sendmsg.linear.x += msg.linear.x;
      sendmsg.linear.y += msg.linear.y;

      /*
      micros_flocking::Gradient sendgradient;
      double dist_vl = pow(get_vector(q_r,my_vpoint_position).first,2)+pow(get_vector(q_r,my_vpoint_position).second,2);
      if(dist_vl<= R*R)
      {
          sendgradient.gradient=1;
      }
      else
      {
           int minNeighbor = -1;bool inswarm=false;
           for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
           {
                if(!inswarm &&(*i)->gradient >0 )
                {
                    minNeighbor = (*i)->gradient;
                    inswarm=true;               
                 }
                if((*i)->gradient >0&& (*i)->gradient <minNeighbor )
                    minNeighbor = (*i)->gradient;
           }
           if(!inswarm || minNeighbor > my_gradient)
               sendgradient.gradient = -1;
           else
               sendgradient.gradient = minNeighbor + 1;
      }
      gradient_pub.publish(sendgradient);*/
      
      double v_dist_x = my_vpoint_position.first - my_position.first;
      double v_dist_y = my_vpoint_position.second - my_position.second;
      double v_dist = sqrt(v_dist_x*v_dist_x + v_dist_y*v_dist_y);
      if( v_dist > 1)
          cout<<"warning:"<<v_dist<<endl;
      my_vpoint_velocity.first = sendmsg.linear.x;
      my_vpoint_velocity.second = sendmsg.linear.y;
     
          
//test
     // my_vpoint_velocity.first = my_velocity.first;
      //my_vpoint_velocity.second = my_velocity.second;
      

      quadrotor_code::Status sendvpoint;
      sendvpoint.px =  my_vpoint_position.first;
      sendvpoint.py =  my_vpoint_position.second;
      sendvpoint.vx =  my_vpoint_velocity.first;
      sendvpoint.vy =  my_vpoint_velocity.second;
      vpoint_pub.publish(sendvpoint);

      if(!diffdrive)
      {
          if(sendmsg.linear.x==0 && sendmsg.linear.y==0)
          pub.publish(sendmsg);
          else{
          double fi = PI/2;
          if(sendmsg.linear.x!=0) 
          {
              fi=atan(sendmsg.linear.y/sendmsg.linear.x);
              if(sendmsg.linear.x<0&&sendmsg.linear.y>=0)
                  fi+=PI;
              else if (sendmsg.linear.x<0 && sendmsg.linear.y<0)
                  fi-=PI;
          }
          else if (sendmsg.linear.y<0)
          fi= -PI/2;
          double v_scale = sqrt(sendmsg.linear.x*sendmsg.linear.x+sendmsg.linear.y*sendmsg.linear.y);
          //cout<<sendmsg.linear.x<<' '<<sendmsg.linear.y<<"___";
          sendmsg.linear.x =  v_scale*cos(fi-my_theta);
          sendmsg.linear.y = v_scale *sin(fi-my_theta);
          //cout<<sendmsg.linear.x<<' '<<sendmsg.linear.y<<endl;
          //cout<<fi<<' '<<my_theta<<endl;
          pub.publish(sendmsg);
          }
      }
      else
      {/*
         geometry_msgs:: Twist senddiff;
         if(sendmsg.linear.x==0 && sendmsg.linear.y==0)
             pub.publish(senddiff);
          else{
             double fi = PI/2;
          if(sendmsg.linear.x!=0) 
          {
              fi=atan(sendmsg.linear.y/sendmsg.linear.x);
              if(sendmsg.linear.x<0&&sendmsg.linear.y>=0)
                  fi+=PI;
              else if (sendmsg.linear.x<0 && sendmsg.linear.y<0)
                  fi-=PI;
          }
          else if (sendmsg.linear.y<0)
             fi= -PI/2;
          
          double theta_diff= fi - my_theta;
          double v_value = sqrt(sendmsg.linear.x*sendmsg.linear.x+sendmsg.linear.y*sendmsg.linear.y);
          //double my_v_value = sqrt(my_vpoint_velocity.first*my_vpoint_velocity.first+my_vpoint_velocity.second*my_vpoint_velocity.second);
          senddiff.linear.x = v_value;
          senddiff.angular.z = theta_diff * hz;
          if(senddiff.angular.z > max_turn)
              senddiff.angular.z = max_turn;
          if(senddiff.angular.z < - max_turn)
              senddiff.angular.z = - max_turn;
          pub.publish(senddiff);
          }
        */
         //tracking vpoint
/*
          double delta_x = -my_vpoint_position.first +my_position.first;
          double delta_y = -my_vpoint_position.second +my_position.second;
          if(delta_x!=0 ){
          double alpha = atan(delta_y/delta_x) - my_theta;
          double beta = - alpha - my_theta;

          double rho = sqrt(delta_x*delta_x+delta_y*delta_y);
          geometry_msgs:: Twist senddiffmsg;
          senddiffmsg.linear.x = krho*rho;
          if(delta_x > 0)
             senddiffmsg.linear.x *= -1.0;
          senddiffmsg.angular.z = kalpha*alpha + kbeta*beta;
      
          pub.publish(senddiffmsg);
          }*/
          /*
          if(!diff_thread_ok)
          {
              diff_thread_ok = true;
              diff_tracking_thread dtt(& pub);
              boost::thread thrd2(dtt);
          }*/
          
          double delta_x = my_vpoint_position.first - my_position.first;
          double delta_y = my_vpoint_position.second - my_position.second;
          
          double alpha = atan(delta_y/delta_x) - my_theta;
          double beta = - alpha - my_theta;
          if(my_vpoint_position.first == my_position.first)
          {
              loop_rate.sleep();continue;
          }
          else if(delta_x < 0.0)
          {
              double r_theta = PI - my_theta;//if my_theta >0
              if(my_theta < 0.0)
                 r_theta = -PI - my_theta;
              alpha = -atan(delta_y/delta_x) - r_theta;
              beta = - alpha - r_theta;
          }
          double rho = sqrt(delta_x*delta_x+delta_y*delta_y);
          
          geometry_msgs:: Twist senddiffmsg;
          senddiffmsg.linear.x = krho*rho;
          senddiffmsg.angular.z = kalpha*alpha + kbeta*beta;
          if(rho<0.1)
              senddiffmsg.linear.x = 0;
          if(delta_x < 0.0)
              senddiffmsg.angular.z = - senddiffmsg.angular.z;
          pub.publish(senddiffmsg);
      }
      
      lastmsg.linear.x = sendmsg.linear.x;
      lastmsg.linear.y = sendmsg.linear.y;
      loop_rate.sleep();
      time_count++;
   }
   return 0;
}
