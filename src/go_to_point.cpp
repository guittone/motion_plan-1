// ROS Libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h" 
#include "tf/transform_listener.h" 
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"

// C++ Libraries
#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>
//DEFINE
#define _USE_MATH_DEFINES
//GLOBAL VARIABLES
ros::Subscriber sub;
ros::Subscriber odom_sub;
ros::Publisher motion_pub;
geometry_msgs::Twist motion_command;
//Parametri di posizione attuali
geometry_msgs::Point position;
double roll;
double pitch;
double yaw;
//Errore di posizione
double pos_err = 1000; //Inizializzata a valore casuale, per evitare che sia automaticamente inizializzata a 0
//Posizione d'arrivo
geometry_msgs::Point desired_position;
//Precisione desiderata
double yaw_precision = M_PI / 90; // +/- 2 degrees precision
float dist_precision = 0.8;
//
float sft_dist = 1.5; //safety distance from front, fleft, fright obstacles
float sft_lat_dist = 0.5; //Safety distance from left and right obstacle
double take_action_linear_speed = 0.4; 
double take_action_angular_speed = 0.3;
float min_regions[6] = {0};

typedef enum _ROBOT_STATE //Enum to classify the robot general state
{
    TURN_LEFT= 0,
    FOLLOW_WALL_RIGHT,
    FOLLOW_WALL_LEFT,
    GO_TO_POINT,
    UNKNOW
} ROBOT_STATE;

ROBOT_STATE state;

typedef enum _GOING_TO_POINT_STATE //Enum to classify the go_straight_to_point state
{
    FIX_YAW = 0,
    STRAIGHT,
    ARRIVED
} GOING_TO_POINT_STATE;

GOING_TO_POINT_STATE change_state = FIX_YAW; //Variable used during the go_straight_to_point phase

//FUNCTION PROTOTIPES
float calcola_minimo(const sensor_msgs::LaserScan::ConstPtr& msg,int n1,int n2);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void decide_actions(float regions[], float sft_dist);
void robot_moving(ROBOT_STATE move_type);
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_info);
void fix_yaw(const geometry_msgs::Point des_pos);
void go_straight(const geometry_msgs::Point des_pos);

//FUNCTIONS
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
    This function is called every time a LaserScan message arrive to our node
    It devides the sensor's ranges vector in five regions.
    The minimum value of the sensor's ranges is calculated for each region
    Then tha function to decide how to move is called
*/
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int k;
	
//calculates the minimum for each of the 6 regions
	for(k = 0; k < 6; k++)
		min_regions[k] = calcola_minimo(msg, k*120, (k+1)*120);

    
    
}

/*
    This function will change the robot general state according to the minimum ranges
    values for each region recived by the laserCallback function.

*/
void decide_actions(float regions[], float sft_dist)
{
	/*	
	 *	 we will exclude from computations the border regions left and right!
	 *
	 *    |------------------/front\--------------|
	 *    |--------/fright--/      \--fleft\------|
	 *    |-right-/                        \-left-|
	 *
	 */
    float right = regions[0];
	float fright = regions[1];
	float front  = regions[2];
	float fleft  = regions[3];
    float left = regions[4];
    int i;
	//std_msgs::String state_description;
    
    if(change_state!=ARRIVED) //Continue to scan since you're not arrived
    {
	if((fright > sft_dist) && (front > sft_dist) && (fleft > sft_dist)&& (left > sft_lat_dist) && (right > sft_lat_dist))
	{
		//state_description.data = "case 1 - nothing [Go to the point]";
        state = GO_TO_POINT;
        
	}
    else if((fright > sft_dist) && (front > sft_dist) && (fleft > sft_dist) && (left < sft_lat_dist))
	{
		//Estensione caso 1, necessaria per non passare troppo vicino a muri
        state = FOLLOW_WALL_LEFT;
        
	}
    else if((fright > sft_dist) && (front > sft_dist) && (fleft > sft_dist) && (right < sft_lat_dist))
	{
		//Estensione caso 1, necessaria per non passare troppo vicino a muri
        state = FOLLOW_WALL_RIGHT;
        
	}
	else if((fright > sft_dist) && (front > sft_dist) && (fleft < sft_dist))
	{
		
        state = FOLLOW_WALL_LEFT; //But getting away from it
        
	}
	else if((fright > sft_dist) && (front < sft_dist) && (fleft > sft_dist))
	{

        state = TURN_LEFT;
        
	}
	else if((fright > sft_dist) && (front < sft_dist) && (fleft < sft_dist))
	{

        state = TURN_LEFT;
        
	}
	else if((fright < sft_dist) && (front > sft_dist) && (fleft > sft_dist))
	{
	
        state = FOLLOW_WALL_RIGHT; //But getting away from it
        

	}
	else if((fright < sft_dist) && (front > sft_dist) && (fleft < sft_dist))
	{
		
        state = TURN_LEFT;
        

	}
	else if((fright < sft_dist) && (front < sft_dist) && (fleft > sft_dist))
	{
		
        state = TURN_LEFT;
        

	}
	else if((fright < sft_dist) && (front < sft_dist) && (fleft < sft_dist))	
	{
		
        state = TURN_LEFT;
        

	}
	else 
	{
		//state_description.data = "unknown case";
        state = UNKNOW;
		ROS_INFO("%f-%f-%f-%f-%f", regions[0], regions[1], regions[2], regions[3], regions[4]);
		
	}
    
    }
    else //If arrived is true we can shut down our node
    {
        ROS_INFO("Sono arrivato a destinazione");
        for(i=0; i<10;i++)
        {
        motion_command.linear.x = 0.0;
        motion_command.angular.z = 0.0;
        motion_pub.publish(motion_command);
        }
        ros::shutdown();
    }
    
}

/*
    Questa funzione si occupa dell'attuazione del moto in base all'argomento con la quale
    viene invocata che è deciso dalla decide_action fintanto che il robot non è arrivato
*/
void robot_moving(ROBOT_STATE move_type)
{   


    if(move_type == GO_TO_POINT)
    {

        ROS_INFO("Going to the point");
        if(change_state == FIX_YAW)
            fix_yaw(desired_position);
        else if(change_state == STRAIGHT)
                go_straight(desired_position);
        motion_pub.publish(motion_command);
        
    }
    else if (move_type == TURN_LEFT)
    {
        if(pos_err > dist_precision)
        {
        ROS_INFO("Turning left!");
        motion_command.linear.x = 0.0;
        motion_command.angular.z = take_action_angular_speed;
        motion_pub.publish(motion_command);
        }
        else change_state = ARRIVED;
       
    }
    else if (move_type == FOLLOW_WALL_RIGHT)
    {
        if(pos_err > dist_precision)
        {
        ROS_INFO("Avoiding the obstacle!");
         motion_command.linear.x = take_action_linear_speed;
          motion_command.angular.z = -0.3;
          motion_pub.publish(motion_command);
        }
        else change_state = ARRIVED;

    }

    else if (move_type == FOLLOW_WALL_LEFT)
    {
        if(pos_err > dist_precision)
        {
        ROS_INFO("Avoiding the obstacle!");
         motion_command.linear.x = take_action_linear_speed;
          motion_command.angular.z = 0.3;
          motion_pub.publish(motion_command);
        }
        else change_state = ARRIVED;

    }
       

}
//Funzione chiamata ogni volta che viene ricevuto un messaggio dal topic /odom
//Sarà sfruttata per inserire all'interno di variabili globali i valori di posizione e yaw attuali del bot
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_info)
 {
     //Aggiornare la posizione del robot
     position = odom_info->pose.pose.position;
     //aggiornare l'angolo del robot (yaw)
     tf::Quaternion q(odom_info->pose.pose.orientation.x,
                      odom_info->pose.pose.orientation.y,
                      odom_info->pose.pose.orientation.z,
                      odom_info->pose.pose.orientation.w);
     tf::Matrix3x3 m(q);
     m.getRPY(roll, pitch, yaw);
 }

 //Funzione per correggere lo yaw
 void fix_yaw(const geometry_msgs::Point des_pos)
 {
     double desired_yaw;
     double yaw_err;
     desired_yaw = std::atan2(des_pos.y - position.y, des_pos.x - position.x);
     yaw_err = desired_yaw - yaw;
     if (std::abs(yaw_err) > yaw_precision)
        if( yaw_err > 0)
            motion_command.angular.z = -0.4;
        else 
            motion_command.angular.z = 0.4; 
    else if (std::abs(yaw_err) <= yaw_precision)
            change_state = STRAIGHT;
        
     
 }

 void go_straight(const geometry_msgs::Point des_pos)
  { 
    double desired_yaw;
    double yaw_err;
    desired_yaw = std::atan2(des_pos.y - position.y, des_pos.x - position.x);
    yaw_err = desired_yaw - yaw;
    pos_err = std::sqrt(std::pow(des_pos.y - position.y, 2) + std::pow(des_pos.x - position.x,2));
    if(pos_err > dist_precision)
    {
        motion_command.angular.z = 0;
        motion_command.linear.x = take_action_linear_speed;
        
    }
    else 
        change_state = ARRIVED;
    if (std::abs(yaw_err) > yaw_precision)
        change_state = FIX_YAW;
        
  }



int main(int argc, char **argv)
{
    //ROS node initialization
    ros::init(argc, argv, "go_to_point");
    ros::NodeHandle n;
    //Subscribing to the laser topic to read laser messages
    sub = n.subscribe("/m2wr/laser/scan", 100, laserCallback);
    //Subscribing to the /odom topic to get all the position info
    odom_sub = n.subscribe("/odom",100, odomCallback);
    //Advertising the ROS master that we are  going to publish
    //Twist messages on the cmd_vel topic
    motion_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //Infinite loop while there are no error
    desired_position.x = 10;
    desired_position.y = 10;
    desired_position.z = 0;
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        decide_actions(min_regions, sft_dist);
        robot_moving(state);
        ros::spinOnce();
        loop_rate.sleep();
        
    }

    
    return 0;
}