#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "sensor_msgs/LaserScan.h"
#include <iomanip>
/*
funzione per il calcolo del minimo in un array 
Parametri: a[] array di float , n1 indice 1, n2 indice 2

float calcola_minimo(const std::vector<float> a,int n1,int n2)
{
    int i;
    float minimo = a[n1];
    for(i=n1+1;i<=n2;i++)
        if(a[i]<minimo)
            minimo = a[i];
    return minimo;

}
*/
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    //Divisione dei 180 gradi con 720 letture in 6 regioni uguali
    //Calcolo del minimo per ogni regione
    float min_set1=0;
    float min_set2=0;
    float min_set3=0;
    float min_set4=0;
    float min_set5=0;
    float min_set6=0;
    int i;
    
    min_set1 = msg->ranges[0];
    for(i=0;i<120;i++)
    {
        if(msg->ranges[i] < min_set1)
            min_set1 = msg->ranges[i];
    }
    if(min_set1>10)
        min_set1 = 10;

    min_set2 = msg->ranges[120];
    for(i=120;i<240;i++)
    {
        if(msg->ranges[i] < min_set2)
            min_set2 = msg->ranges[i];
    }
    if(min_set2>10)
        min_set2 = 10;

    min_set3 = msg->ranges[240];
    for(i=240;i<360;i++)
    {
        if(msg->ranges[i] < min_set3)
            min_set3 = msg->ranges[i];
    }
    if(min_set3>10)
        min_set3 = 10;
    
    min_set4 = msg->ranges[360];
    for(i=360;i<480;i++)
    {
        if(msg->ranges[i] < min_set4)
            min_set4 = msg->ranges[i];
    }
    if(min_set4>10)
        min_set4 = 10;
    
    min_set5 = msg->ranges[480];
    for(i=480;i<600;i++)
    {
        if(msg->ranges[i] < min_set5)
            min_set5 = msg->ranges[i];
    }
    if(min_set5>10)
        min_set5 = 10;

    min_set6 = msg->ranges[600];
    for(i=600;i<720;i++)
    {
        if(msg->ranges[i] < min_set6)
            min_set6 = msg->ranges[i];
    }
    if(min_set6>10)
        min_set6 = 10;

/*Inserimento elementi in un unica stringa
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
    Ricordare in questo caso di usare s.c_str() nella ROS_INFO per converitre la stringa in stringa c
    */
   ROS_INFO("\n[Angolo minimo: %f]\n[Angolo massimo: %f]\n[Passo: %f]\n[Intrevallo temporale tra le misure: %f]\n[Tempo tra le scansioni: %f]\n[Range massimo: %f]\n[Range minimo: %f]\n[%f] [%f] [%f] [%f] [%f] [%f]",msg->angle_min,msg->angle_max, msg->angle_increment,msg->time_increment,msg->scan_time,msg->range_max,msg->range_min,min_set1,min_set2,min_set3,min_set4,min_set5,min_set6);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reading_laser");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/m2wr/laser/scan", 1000, laserCallback);
    ros::spin();
    return 0;
}