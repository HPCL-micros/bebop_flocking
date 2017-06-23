#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <cmath>

using namespace std;

struct Position{
    double x;
    double y;
    double z;

    Position():x(0.0),y(0.0),z(0.0){}
};

double rot_cov;
std::string frame_id, child_frame_id;
bool get_odom;
double x_limit, y_limit, z_limit;
double x_vel_limit, y_vel_limit, z_vel_limit;
bool need_hover=false;
boost::mutex mut;
bool stop_enabled=false;

std::string robot_name;

static ros::Publisher cmd_vel_pub;
//static ros::Publisher driver_down_pub;

Position start;
Position current;

void wait(float seconds)
{
    boost::this_thread::sleep(boost::posix_time::seconds(seconds));
}

void fixCallBack(const sensor_msgs::NavSatFixConstPtr& fix)
{
    if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
        ROS_INFO("No fix.");
        return;
    }

    if (fix->header.stamp == ros::Time(0)) {
        return;
    }

    double northing, easting;
    std::string zone;

    gps_common::LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

    if(get_odom) {
        nav_msgs::Odometry odom;
        odom.header.stamp = fix->header.stamp;

        if (frame_id.empty())
            odom.header.frame_id = fix->header.frame_id;
        else
            odom.header.frame_id = frame_id;

        odom.child_frame_id = child_frame_id;

        odom.pose.pose.position.x = easting;
        odom.pose.pose.position.y = northing;
        odom.pose.pose.position.z = fix->altitude;

        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.z = 0;
        odom.pose.pose.orientation.w = 1;

        // Use ENU covariance to build XYZRPY covariance
        boost::array<double, 36> covariance = {{
            fix->position_covariance[0],
            fix->position_covariance[1],
            fix->position_covariance[2],
            0, 0, 0,
            fix->position_covariance[3],
            fix->position_covariance[4],
            fix->position_covariance[5],
            0, 0, 0,
            fix->position_covariance[6],
            fix->position_covariance[7],
            fix->position_covariance[8],
            0, 0, 0,
            0, 0, 0, rot_cov, 0, 0,
            0, 0, 0, 0, rot_cov, 0,
            0, 0, 0, 0, 0, rot_cov
            }};

        odom.pose.covariance = covariance;
    }

    //static bool is_start=true;
    //if(is_start)  //第一次执行，设置初始位置和当前位置，需要注意当前位置此时等于初始位置
    static int count=0;
    if(count<30)
    {
        start.x=easting;
        start.y=northing;
        start.z=fix->altitude;
        current.x=start.x;
        current.y=start.y;
        current.z=start.z;

        //is_start = false;
        count++;

        //std::cout<<"start: "<<start.x<<","<<start.y<<","<<start.z<<std::endl;
    }
    else  //不是第一次执行，设置当前位置
    {
        current.x=easting;
        current.y=northing;
        current.z=fix->altitude;

        //std::cout<<"current: "<<current.x<<","<<current.y<<","<<current.z<<std::endl;
    }
}

void odomCallBack(const nav_msgs::OdometryConstPtr& odom)
{
    current.x=odom->pose.pose.position.x;
    current.y=odom->pose.pose.position.y;
    current.z=odom->pose.pose.position.z;
}

void cmdCallBack(const geometry_msgs::TwistConstPtr& cmd)
{
    if(need_hover)  //需要悬停，速度置0
    {
        /*geometry_msgs::Twist t;
        t.linear.x=0.0;
        t.linear.y=0.0;
        t.linear.z=0.0;

        cmd_vel_pub.publish(t);*/
        return;
    }
    else
    {
        geometry_msgs::Twist t;
        t.linear.x = abs(cmd->linear.x) < x_vel_limit?cmd->linear.x:x_vel_limit*(cmd->linear.x / abs(cmd->linear.x) );  //限制最大速度
        t.linear.y = abs(cmd->linear.y) < y_vel_limit?cmd->linear.y:y_vel_limit*(cmd->linear.y / abs(cmd->linear.y) );
        t.linear.z = abs(cmd->linear.z) < z_vel_limit?cmd->linear.z:z_vel_limit*(cmd->linear.z / abs(cmd->linear.z) );

        t.angular.z=cmd->angular.z;

        cmd_vel_pub.publish(t);
    }
}

void checkStatus()
{
    while(ros::ok())
    {
        if(stop_enabled){
            cout<<"topic stop activated"<<endl;
            wait(0.5);
            continue;
        }
        if (current.x - start.x >= x_limit || current.x - start.x <= -x_limit) {
            //std::cout << x_limit<<"," << current.x<<","<< start.x<<","<<current.x-start.x << std::endl;
            cout<<"x limit is over!!!"<<endl;
            mut.lock();
            need_hover = true;
            mut.unlock();
            return;
        }
        if (current.y - start.y >= y_limit || current.y - start.y <= -y_limit) {
            cout<<"y limit is over!!!"<<endl;
            mut.lock();
            need_hover = true;
            mut.unlock();
            return;
        }
        if (current.z - start.z >= z_limit || current.z - start.z <= -z_limit) {
            cout<<"z limit is over!!!"<<endl;
            mut.lock();
            need_hover = true;
            mut.unlock();
            return;
        }
        
        //mut.lock();
        //need_hover = false;
        //mut.unlock();
        wait(0.5);
    }
}

/*void waitSocket()
{
    int server_sockfd = socket(AF_INET,SOCK_STREAM, 0);

    struct sockaddr_in server_sockaddr;
    server_sockaddr.sin_family = AF_INET;
    server_sockaddr.sin_port = htons(45678);
    server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if(bind(server_sockfd,(struct sockaddr *)&server_sockaddr,sizeof(server_sockaddr))==-1)
    {
        perror("bind");
        exit(1);
    }

    if(listen(server_sockfd,20) == -1)
    {
        perror("listen");
        exit(1);
    }
    
    char buffer[64];
    struct sockaddr_in client_addr;
    socklen_t length = sizeof(client_addr);

    int conn = accept(server_sockfd, (struct sockaddr*)&client_addr, &length);
    if(conn<0)
    {
        perror("connect");
        exit(1);
    }

    while(ros::ok())
    {
        memset(buffer,0,sizeof(buffer));
        int len = recv(conn, buffer, sizeof(buffer),0);
        //if(strcmp(buffer,"down")==0)
        {
            ROS_ERROR("in flight barrier, received driver down signal");
            for(int i=0;i<5;i++)
            {    
                std_msgs::String msg;
                msg.data=robot_name;
                driver_down_pub.publish(msg);
            }    
        }
        wait(0.5);
    }
    close(conn);
    close(server_sockfd);
}
*/

void stopCallBack(const std_msgs::Int32ConstPtr& msg)
{
    mut.lock();
    need_hover = true;
    stop_enabled = true;
    mut.unlock();
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "flight_barrier_node");
    ros::NodeHandle node;
    ros::NodeHandle priv_node("~");

    start.x=0.0;
    start.y=0.0;
    start.z=0.0;

    current.x=0.0;
    current.y=0.0;
    current.z=0.0;

    priv_node.param<std::string>("frame_id", frame_id, "");
    priv_node.param<std::string>("child_frame_id", child_frame_id, "");
    priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
    priv_node.param<bool>("get_odom", get_odom, false);
    priv_node.param<double>("x_limit", x_limit, 20.0);
    priv_node.param<double>("y_limit", y_limit, 20.0);
    priv_node.param<double>("z_limit", z_limit, 10.0);
    priv_node.param<double>("x_vel_limit", x_vel_limit, 0.5);
    priv_node.param<double>("y_vel_limit", y_vel_limit, 0.5);
    priv_node.param<double>("z_vel_limit", z_vel_limit, 0.5);
    priv_node.param<std::string>("robot_name", robot_name, "uav0");

    //std::cout<<x_limit<<","<<y_limit<<","<<z_limit<<std::endl;
    //std::cout<<x_vel_limit<<","<<y_vel_limit<<","<<z_vel_limit<<std::endl;

    need_hover = false;

    //odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);

    //ros::Subscriber fix_sub = node.subscribe("fix", 10, fixCallBack);
    ros::Subscriber odom_sub = node.subscribe("odom", 10, odomCallBack);
    ros::Subscriber stop_sub = node.subscribe("/stop", 10, stopCallBack);
    ros::Subscriber cmd_vel_sub = node.subscribe("barrier_input_cmd_vel", 1000, cmdCallBack);
    cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    //driver_down_pub = node.advertise<std_msgs::String>("/uav_driver_down", 1000);

    boost::thread check(checkStatus);
    //boost::thread socket_pro(waitSocket);

    ros::spin();
    
    check.join();
    //socket_pro.join();
}

