// ROS Libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // Movement Commands
#include "sensor_msgs/LaserScan.h" // Laser Data
#include "tf/transform_listener.h" // tf Tree
#include "tf/transform_datatypes.h"

#define _USE_MATH_DEFINES

// C++ Libraries
#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>
//Global variables
ros::Subscriber sub;
ros::Subscriber odom_sub;
ros::Publisher motion_pub;
geometry_msgs::Twist motion_comand;
//Parametri di posizione attuali
geometry_msgs::Point position;
double roll;
double pitch;
double yaw;
//Posizione d'arrivo
geometry_msgs::Point desired_position;
desired_position.x = 15;
desired_position.y = 10;
desired_position.z = 0;
//Precisione desiderata
double yaw_precision = M_PI / 90 // +/- 2 degrees precision
float dist_precision = 0.3
//
float sft_dist_ = 1.0;
double take_action_linear_speed = 0.5;
double take_action_angular_speed = 0.3;
bool arrived = false

typedef enum _ROBOT_STATE
{
    STOP = 0,
    TURN_LEFT,
    FOLLOW_WALL,
    GO_TO_POINT,
    UNKNOW
} ROBOT_STATE;

//Function prototipes
float calcola_minimo(const sensor_msgs::LaserScan::ConstPtr& msg,int n1,int n2)
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
void decide_actions(float regions[], float sft_dist)
bool robot_moving(const ROBOT_STATE move_type)
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_info)


//Funcions
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
    decide_actions(min_regions, sft_dist)
    
}

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
	float fright = regions[1];
	float front  = regions[2];
	float fleft  = regions[3];
    bool arrived;
    const ROBOT_STATE state;
	std_msgs::String state_description;
    
    if(!arrived) //Fintanto che non sei arrivato esamina i valori del laser
    {
	if((fright > sft_dist) && (front > sft_dist) && (fleft > sft_dist))
	{
		state_description.data = "case 1 - nothing [Go to the point]";
        state = GO_TO_POINT;
	}
	else if((fright > sft_dist) && (front > sft_dist) && (fleft < sft_dist))
	{
		state_description.data = "case 2 - fleft [Go to the point]";
        state = GO_TO_POINT;
	}
	else if((fright > sft_dist) && (front < sft_dist) && (fleft > sft_dist))
	{
		state_description.data = "case 3 - front [Turn left]";
        state = TURN_LEFT;
	}
	else if((fright > sft_dist) && (front < sft_dist) && (fleft < sft_dist))
	{
		state_description.data = "case 4 - front and fleft [Turn left]";
        state = TURN_LEFT;
	}
	else if((fright < sft_dist) && (front > sft_dist) && (fleft > sft_dist))
	{
		state_description.data = "case 5 - fright [Follow the wall]";
        state = FOLLOW_WALL;

	}
	else if((fright < sft_dist) && (front > sft_dist) && (fleft < sft_dist))
	{
		state_description.data = "case 6 - fright and fleft [Go to the point]";
        state = GO_TO_POINT;

	}
	else if((fright < sft_dist) && (front < sft_dist) && (fleft > sft_dist))
	{
		state_description.data = "case 7 - fright and front [Follow the wall]";
        state = FOLLOW_WALL;

	}
	else if((fright < sft_dist) && (front < sft_dist) && (fleft < sft_dist))	
	{
		state_description.data = "case 8 - fright and front and fleft [Turn left]";
        state = TURN_LEFT;

	}
	else 
	{
		state_description.data = "unknown case";
        state = UNKNOW;
		ROS_INFO("%f-%f-%f-%f-%f", regions[0], regions[1], regions[2], regions[3], regions[4]);
		
	}

    arrived=robot_moving(state); //Assegno alla variabile arrived il valore di ritorno della robot_moving
    }
    else //Se la variabile arrived è true allora pubblico il messaggio di arrivo e chiudo il nodo
    {
        ROS_INFO("Sono arrivato a destinazione");
        ros::shutdown();
    }
    
}

/*
    Questa funzione si occupa dell'attuazione del moto in base all'argomento con la quale
    viene invocata che è deciso dalla decide_action fintanto che il robot non è arrivato
*/
bool robot_moving(const ROBOT_STATE move_type)
{   


    if(move_type == GO_TO_POINT)
    {
        //Inserire funzione per arrivare in un punto dritto e dare move_type= STOP all'arrivo
    }
    else if( move_type == STOP)
    {
       ROS_INFO("Arrivato!");
       motion_command.linear.x = 0.0;
       motion_comand.angular.z = 0.0;
       motion_pub.publish(motion_command);
       return false;
    }

    else if (move_type == TURN_LEFT)
    {
        ROS_INFO("Turning left! \n");
        motion_comand.angular.z = take_action_angular_speed;
    }
    else if (move_type == FOLLOW_WALL)
    {
        ROS_INFO("Avoiding the wall! \n");
         motion_command.linear.x = take_action_linear_speed;
    }
    motion_pub.publish(motion_command);
    return true;   

}
//Funzione chiamata ogni volta che viene ricevuto un messaggio dal topic /odom
//Sarà sfruttata per inserire all'interno di variabili globali i valori di posizione e yaw attuali del bot
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_info)
 {
     //Aggiornare la posizione del robot
     position = odom_info.pose.pose.position;
     //aggiornare l'angolo del robot (yaw)
     tf::Quaternion q(odom_info->pose.pose.orientation.x,
                      odom_info->pose.pose.orientation.y,
                      odom_info->pose.pose.orientation.z,
                      odom_info->pose.pose.orientation.w,);
     tf::Matrix3x3 m(q);
     m.getRPY(roll, pitch, yaw);
 }



int main(int argc, char **argv)
{
    //ROS node initialization
    ros::init(argc, argv, "reading_laser");
    ros::NodeHandle n;
    //Subscribing to the laser topic to read laser messages
    sub = n.subscribe("/m2wr/laser/scan", 1000, laserCallback);
    //Subscribing to the /odom topic to get all the position info
    odom_sub = n.subscribe("/odom",50, odomCallback);
    //Advertising the ROS master that we are  going to publish
    //Twist messages on the cmd_vel topic
    motion_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //Infinite loop while there are no error
    while (ros::ok())
    {
        ros::spinOnce();
    }

    
    return 0;
}