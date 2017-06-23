#ifndef MICROS_FLOCK_H_
#define MICROS_FLOCK_H_

#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "micros_flocking/Neighbor.h"
#include "micros_flocking/Position.h"
#include "micros_flocking/Gradient.h"
#include "sensor_msgs/LaserScan.h"
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
#include <ctime>
#include <boost/thread/thread.hpp>
using namespace std;

namespace micros_flock{

    double PI=acos(-1);
#define EPSILON 0.1
#define A 5
#define B 5
    const double C = abs(A-B) / sqrt(4*A*B);
#define H 0.2
#define D 20
#define R 25
#define C1 0.3
#define C2 0.3
#define rspeedlimit 8.0
    double basespeed = 0;
    double baseomega = 0;//PI/60;
    double base_angle =0;
#define ploss 0 //max1000
#define diffdrive false
#define max_turn 0.7
#define vlvx 0
#define vlvy 0//注意，vlvx,vlvy仅在xiangdui坐标系

    double obdist=12;
    double obD = 10;
    bool move_vl = false;
    bool neighbor_loss=false;
    int hz=10;
    bool delay_enabled = false;
    int delay_time = 200;

#define krho 0.3*(rspeedlimit+basespeed)/2
    double kalpha =0.8*(rspeedlimit+basespeed)/2;
#define kbeta -0.15*(rspeedlimit+basespeed)/2
    double keepdistance = sqrt(rspeedlimit/krho);


    double   interval=1.0/hz;
    double pm1=0.3,pm2=1,pm3=0.3;//pm=0.3 pm2=1 edited

    pair<double,double> q_r = pair<double,double>(0,0);
    pair<double,double> p_r = pair<double,double>(basespeed,basespeed);

    void leaderPosCB(const nav_msgs::Odometry::ConstPtr& leader_pos)
    {
        p_r.first = leader_pos->pose.pose.position.x;
        p_r.second = leader_pos->pose.pose.position.y;

        q_r.first = leader_pos->twist.twist.linear.x;
        q_r.second = leader_pos->twist.twist.linear.y;
    }

    ros::NodeHandle nh;
    //ros::Subscriber leader_pos_sub_ = nh.subscribe<nav_msgs::Odometry>("/robot_0/base_pose_ground_truth", 1000, leaderPosCB);

    class NeighborHandle
    {
    public:
        ros::Subscriber sub;
        double _px,_py,_vx,_vy;
        pair<double,double> _position,_velocity;
        int _r_id;
        double mypm_g;
        int gradient;
        NeighborHandle(int r_id)
        {
            ros::NodeHandle n;
            stringstream ss;
            ss<<"/robot_"<<r_id<<"/vpoint_position";
            sub = n.subscribe(ss.str(), 1000, &NeighborHandle::cb,this);
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

        void close()
        {
            sub.shutdown();
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

    static void neighbor_cb(const micros_flocking::Neighbor::ConstPtr & msg)
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
    void my_vpoint_position_cb(const micros_flocking::Position::ConstPtr & msg)
    {
        my_vpoint_position.first = msg->px;
        my_vpoint_position.second = msg->py;
        my_vpoint_velocity.first = msg->vx;
        my_vpoint_velocity.second = msg->vy;
        //my_theta = msg->theta;
        my_gradient = msg->gradient;
        //cout<<"my pose updated"<<endl;
    }

    void my_position_cb(const micros_flocking::Position::ConstPtr & msg)
    {
        my_position.first = msg->px;
        my_position.second = msg->py;
        my_velocity.first = msg->vx;
        my_velocity.second = msg->vy;
        my_theta = msg->theta;


        //my_gradient = msg->gradient;
        //cout<<"my pose updated"<<endl;
    }

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
    double obR_alpha = segma_norm(pair<double,double>(obdist,0));
    double obD_alpha = segma_norm(pair<double,double>(obD,0));

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
            pair<double,double> p_ij = get_vector(my_vpoint_velocity,(*i)->_velocity);
            re.first += a_ij((*i)->_position) * p_ij.first;
            re.second += a_ij((*i)->_position) * p_ij.second;
            // count++;
        }
        //cout<<count<<endl;
        return re;
    }

    //pair<double,double> q_r = pair<double,double>(0,0);
    //pair<double,double> p_r = pair<double,double>(basespeed,basespeed);
    pair<double,double> f_r()
    {
        pair<double,double> re = pair<double,double>(0,0);


        //q_r.first += p_r.first * interval;
        //q_r.second += p_r.second * interval;
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

    bool obdetected = false;
    pair<double,double> obpos=pair<double,double>(0,0);
    double ob_theta = 0;
    pair<double,double> obvel=pair<double,double>(0,0);

    void scan_cb(const sensor_msgs::LaserScan::ConstPtr & msg)
    {
        double start=msg->angle_min;
        double inc = msg->angle_increment;
        double min = msg->range_max;
        double minindex =0;
        for(int i=0;i<msg->ranges.size();i++)
        {
            if(min>msg->ranges[i])
            {
                min=msg->ranges[i];
                minindex = i;
            }

        }
        if(min > obdist)
            obdetected = false;
        else
        {
            double angle = start+inc * minindex;
            //cout<<angle<<' '<<inc<<' '<<min<<endl;
            obpos.first = my_position.first + min*cos(angle+my_theta);
            obpos.second = my_position.second + min*sin(angle+my_theta);
            ob_theta = angle+my_theta;
            if(ob_theta > PI) ob_theta -= 2*PI;
            if(ob_theta < -PI) ob_theta += 2*PI;
            double my_v_theta = atan(my_vpoint_velocity.second / my_vpoint_velocity.first);
            if(my_vpoint_velocity.first<0.0&&my_vpoint_velocity.second>=0.0)
                my_v_theta +=PI;
            else if (my_vpoint_velocity.first<0.0&&my_vpoint_velocity.second<0.0)
                my_v_theta -=PI;
            //double delta_theta = ob_theta - my_v_theta;
            pair<double,double> ak = pair<double,double>(-cos(ob_theta),-sin(ob_theta));
            double x =my_vpoint_velocity.first,y=my_vpoint_velocity.second,ak1 = ak.first,ak2 = ak.second;
            obvel.first = (1-ak1*ak1)*x - ak1*ak2*y;
            obvel.second = - ak1*ak2*x + (1-ak2*ak2)*y;
            obdetected = true;
        }
        //cout<<"my pose updated"<<endl;
    }

    class FlockingAlgorithm{
    public:
        ros::Subscriber sub;
        ros::Subscriber sub_pos;
        ros::Publisher vpoint_pub;
        ros::Subscriber sub_scan;
        geometry_msgs:: Twist msg;

        geometry_msgs:: Twist sendmsg;
        geometry_msgs:: Twist lastmsg;
        ros::Publisher pub;
        int time_count;
        bool diff_thread_ok;
        FlockingAlgorithm()
        {
            ros::NodeHandle n;
            pub = n.advertise<geometry_msgs::Twist>("cmd_vel111",1000);//!
            sub = n.subscribe("neighbor", 1000, neighbor_cb);
            sub_pos = n.subscribe("position", 1000, my_position_cb);
            vpoint_pub = n.advertise<micros_flocking::Position>("vpoint_position",1000);
            //sub_scan = n.subscribe("base_scan", 1000, scan_cb);
            ros::Rate loop_rate(hz);
            p_r.first = vlvx + basespeed * cos(base_angle);
            p_r.second = vlvy + basespeed * sin(base_angle);
            time_count=0;
            diff_thread_ok = false;
            /*
            while(my_position.first == 0)
            {
                ros::spinOnce();
                loop_rate.sleep();
            }*/
            my_vpoint_position.first = my_position.first+keepdistance;
            my_vpoint_position.second = my_position.second+keepdistance;
            my_vpoint_velocity.first = my_velocity.first;
            my_vpoint_velocity.second = my_velocity.second;
        }

        void setVL(double px,double py,double vx,double vy)
        {
            p_r.first=px;
            p_r.second=py;
            q_r.first=vx;
            q_r.second=vy;
        }
        
        void run(geometry_msgs:: Twist &result_msg)
        {
            my_vpoint_position.first += my_vpoint_velocity.first / (double)hz;
            my_vpoint_position.second += my_vpoint_velocity.second / (double)hz;
            if(!diffdrive)
            {
                my_vpoint_position.first = my_position.first;
                my_vpoint_position.second = my_position.second;
                my_vpoint_velocity.first = my_velocity.first;
                my_vpoint_velocity.second = my_velocity.second;
            }
            //my_vpoint_position.first = my_position.first;
            //my_vpoint_position.second = my_position.second;
            pair<double,double> temp = f_r();
            //if(my_gradient <= 10){
            msg.linear.x += (f_g().first*pm1+f_d().first*pm2+temp.first*pm3)/hz;
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
            if(obdetected)
            {
                pair<double,double> re = pair<double,double>(0,0);
                pair<double,double> q_ij = get_vector(my_vpoint_position,obpos);
                pair<double,double> n_ij = segma_epsilon(q_ij);
                double z=segma_norm(q_ij);
                re.first += rho(z/obR_alpha)*phi(z-obD_alpha)*n_ij.first*(1+basespeed*0.5+vlvx*0.5);
                re.second += rho(z/obR_alpha)*phi(z-obD_alpha)*n_ij.second*(1+basespeed*0.5+vlvx*0.5);
                //re.first += phi_alpha(segma_norm(q_ij))*n_ij.first;
                //re.second += phi_alpha(segma_norm(q_ij))*n_ij.second;
                msg.linear.x +=re.first*pm1/hz;
                msg.linear.y +=re.second*pm1/hz;
                re.first = 0;re.second = 0;

                pair<double,double> p_ij = get_vector(my_vpoint_velocity,obvel);
                re.first += a_ij(obpos) * p_ij.first;
                re.second += a_ij(obpos) * p_ij.second;

                msg.linear.x +=re.first*pm2/hz*(1+basespeed*1+vlvx*1.0);
                msg.linear.y +=re.second*pm2/hz*(1+basespeed*1+vlvx*1.0);
                cout<<"obstacle detected "<<q_ij.first<<" "<<q_ij.second<<endl;
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
            sendmsg.linear.x = basespeed * cos(base_angle);
            sendmsg.linear.y = basespeed * sin(base_angle);
            //sendmsg.linear.x = basespeed;
            //sendmsg.linear.y = basespeed;
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
            if( v_dist > R)
                cout<<"warning:"<<v_dist<<endl;
            my_vpoint_velocity.first = sendmsg.linear.x;
            my_vpoint_velocity.second = sendmsg.linear.y;


//test
            // my_vpoint_velocity.first = my_velocity.first;
            //my_vpoint_velocity.second = my_velocity.second;


            micros_flocking::Position sendvpoint;
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
                    result_msg=sendmsg;
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
                if(!diff_thread_ok)
                {
                    diff_thread_ok = true;
                    diff_tracking_thread dtt(& pub);
                    boost::thread thrd2(dtt);
                }
            }
        }

    };
}

#endif
