#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include "orca/RVO.h"


/*
 * This function computes relative distance between two
 * points using distance formula and returns square of 
 * the relative distace
 * 
 */
double norm2 (RVO::Vector2 pointA, RVO::Vector2 pointB){
    return pow(pointA.x() - pointB.x(),2) + pow(pointA.y() - pointA.y(),2);
}


void laser_cb(const sensor_msgs::LaserScanConstPtr& ptr){
    std::cout << " Hello world \n";

    size_t size = ptr->ranges.size();

    RVO::Vector2 point1;
    RVO::Vector2 point2;    
    for(size_t i=0; i< size; i++){

        
        RVO::Vector2 point2 = { ptr->ranges[i] * cos(i * (ptr->angle_increment)), ptr->ranges[i] * sin(i * (ptr->angle_increment))};


        if(i!=0 && abs(point2 - point1) /ptr->ranges[i] > (0.2/4) + 0.1f )
            ROS_INFO("~~~~~~~~~~~Starting new cluster!!!!!!!!!!!!!");
        std::cout << " Range = " << ptr->ranges[i] << "  Theta = " << ptr->angle_increment  * i * 180.0f / M_PI << "\n";


        point1 = point2;
    }
}



int main(int argc, char** argv){
    ros::init(argc, argv, "laser_aggregator");
    ros::NodeHandle nh;

    ros::Subscriber s = nh.subscribe("/scan", 1, laser_cb);

    ros::spin();

    return 0;
}
