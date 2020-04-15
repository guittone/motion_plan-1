
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

//FUNCTION DECLARATION//
float calcola_minimo(const sensor_msgs::LaserScan::ConstPtr& msg,int n1,int n2);
void take_action(float regions[], float sft_dist);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);




//GLOBAL VARIABLES//
/* sft_dist  (float wich rapresent the safety distance below wich we would like to
    stay from an obstacle)
*/
float sft_dist_ = 1.0;

/*
	Defining the publisher we're going to use as a global variable
	is necessary to make it visible to the function inside wich it
	will publish
*/
ros::Publisher motion_pub = {};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "reading_laser");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/m2wr/laser/scan", 1000, laserCallback);
    ::motion_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::spin();
    return 0;
}

//FUNZIONI//

/*
    This function is used to calculate the minimum value of a float array with n2-n1 elements
    If the minimum value is grater than 10 the minimum value is set to 10
    Parameters: -const sensor_msgs::LaserScan::ConstPtr& msg (pointer to LaserScan message type)
                -n1 (Index of the first element of the array)
                -n2 (Index of the last element of the array +1)    
*/
float calcola_minimo(const sensor_msgs::LaserScan::ConstPtr& msg,int n1,int n2)
{
    int i;
    float minimo = msg->ranges[n1];
    for(i=n1+1 ; i<n2 ; i++)
        if(msg->ranges[i] < minimo)
            minimo = msg->ranges[i];
    if(minimo > 10)
        minimo = 10;
    return minimo;

}
/*
    This function is used to make the robot to avoid obstacles depending on the values contained
    in the array regios[]
    Parameters: -regions[] (float array that contain minimum distances from obstacles situated
                            in the 6 regions of the laser_ranges)
*/
void take_action(float regions[],float sft_dist)
{
	
	geometry_msgs::Twist msg;
	
	double linear_x  = 0.0;
	double angular_z = 0.0;
	
	std_msgs::String state_description;
    std::stringstream ss;

	float frr = regions[1];
	float ffr = regions[2];
	float ffl = regions[3];
	float fll = regions[4];
	

    	if((frr > sft_dist) && (ffr > sft_dist) && (ffl > sft_dist) && (fll > sft_dist))
		{
			ss<<"case 1 - nothing";
			linear_x = 0.6;
			angular_z = 0;
		}
		else if((frr > sft_dist) && (ffr > sft_dist) && (ffl > sft_dist) && (fll < sft_dist))
		{
			ss<<"case 2 - fll";
			linear_x = 0;
			angular_z = 0.3;
		}
		else if((frr > sft_dist) && (ffr > sft_dist) && (ffl < sft_dist) && (fll > sft_dist))
		{
			ss<<"case 3 - ffl";
			linear_x = 0;
			angular_z = 0.3;
			}
		else if((frr > sft_dist) && (ffr > sft_dist) && (ffl < sft_dist) && (fll < sft_dist))
		{
			ss<<"case 4 - ffl and fll";
			linear_x = 0;
			angular_z = 0.3;
		}
		else if((frr > sft_dist) && (ffr < sft_dist) && (ffl > sft_dist) && (fll > sft_dist))
		{
			ss<<"case 5 - ffr";
			linear_x = 0;
			angular_z = -0.3;
		}
		else if((frr > sft_dist) && (ffr < sft_dist) && (ffl > sft_dist) && (fll < sft_dist))
		{
			ss<<"case 6 - ffr and fll";
			linear_x = 0;
			angular_z = 0.3;
		}
		else if((frr > sft_dist) && (ffr < sft_dist) && (ffl < sft_dist) && (fll > sft_dist))
		{
			ss<<"case 7 - ffr and ffl";
			linear_x = 0;
			angular_z = 0.3;
		}
		else if((frr > sft_dist) && (ffr < sft_dist) && (ffl < sft_dist) && (fll < sft_dist))
		{
			ss<<"case 8 - ffr and ffl and fll";
			linear_x = 0;
			angular_z = 0.3;
		}
		else if((frr < sft_dist) && (ffr > sft_dist) && (ffl > sft_dist) && (fll > sft_dist))
		{
			ss<<"case 9 - frr";
			linear_x = 0;
			angular_z = -0.3;
		}
		else if((frr < sft_dist) && (ffr > sft_dist) && (ffl > sft_dist) && (fll < sft_dist))
		{
			ss<<"case 10 - frr and fll";
			linear_x = 0.3;
			angular_z = 0;
		}
		else if((frr < sft_dist) && (ffr > sft_dist) && (ffl < sft_dist) && (fll > sft_dist))
		{
			ss<<"case 11 - frr and ffl";
			linear_x = 0;
			angular_z = -0.3;
		}
		else if((frr < sft_dist) && (ffr > sft_dist) && (ffl < sft_dist) && (fll < sft_dist))
		{
			ss<<"case 12 - frr and ffl and fll";
			linear_x = 0;
			angular_z = 0.3;
	
		}
		else if((frr < sft_dist) && (ffr < sft_dist) && (ffl > sft_dist) && (fll > sft_dist))
		{
			ss<<"case 13 - frr and ffr";
			linear_x = 0;
			angular_z = -0.3;
		}
		else if((frr < sft_dist) && (ffr < sft_dist) && (ffl > sft_dist) && (fll < sft_dist))
		{
			ss<<"case 14 - frr and ffr and fll";
			linear_x = 0;
			angular_z = -0.3;
		}
		else if((frr < sft_dist) && (ffr < sft_dist) && (ffl < sft_dist) && (fll > sft_dist))
		{
			ss<<"case 15 - frr and ffr and ffl";
			linear_x = 0;
			angular_z = 0.3;
		}
		else if((frr < sft_dist) && (ffr < sft_dist) && (ffl < sft_dist) && (fll < sft_dist))
		{
			ss<<"case 16 - frr and ffr and ffl and fll";
			linear_x = 0;
			angular_z = 0.3;
		}
		else
		{
			ss<<"unknown case";
			ROS_INFO("%f-%f-%f-%f-%f-%f", regions[0], regions[1], regions[2], regions[3], regions[4], regions[5]);
			
		}
    state_description.data = ss.str();
    ROS_INFO("[%s]", state_description.data.c_str());
	

    //Putting the data inside the geometry_msgs msg
	msg.linear.x  = linear_x ; 
	msg.angular.z = angular_z;
    //Publishing the message msg
	motion_pub.publish(msg);
	}


//This function is called every time the reading_lase node read a new message from the /m2wr/laser/scan topic
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int k;
	float min_regions[6] = {0};
//calculates the minimum for each of the 6 regions
	for(k = 0; k < 6; k++)
		min_regions[k] = calcola_minimo(msg, k*120, (k+1)*120);
//Chiamata della funzione  take_action che si occuperà di pubblicare i messaggi sul /cmd_vel topic
//ed informarci in che caso ricadiamo
    take_action(min_regions, sft_dist_);
    
}