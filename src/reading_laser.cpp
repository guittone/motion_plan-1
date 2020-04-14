#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <iomanip>

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::string s;
    std::string a;
    int i;
    float myFloat = 0;
    std::stringstream ss;
    s.clear();
    a.clear();

    myFloat = msg->ranges[0];
    ss << myFloat;
    s = ss.str();
    ss.str("");
    
    for(i=1;i<720;i++)
    {
        myFloat = msg->ranges[i];
        ss << myFloat;
        a = ss.str();
        s = s + "," + a;
        ss.str("");
        a.clear();
    }
   ROS_INFO("\n[Angolo minimo: %f]\n[Angolo massimo: %f]\n[Passo: %f]\n[Intrevallo temporale tra le misure: %f]\n[Tempo tra le scansioni: %f]\n[Range massimo: %f]\n[Range minimo: %f]\n[%s]\n",msg->angle_min,msg->angle_max, msg->angle_increment,msg->time_increment,msg->scan_time,msg->range_max,msg->range_min,s.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reading_laser");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/m2wr/laser/scan", 1000, laserCallback);
    ros::spin();
    return 0;
}