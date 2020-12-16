#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include "orca/RVO.h"

tf::TransformListener* g_list;
tf::StampedTransform g_stmp_transform;

void cb(const sensor_msgs::LaserScanConstPtr & ptr);
RVO::Vector2 transformPoint(tf::Matrix3x3& mat, const tf::Vector3& origin  , const tf::Vector3& point);

int main (int argc, char** argv){

    ros::init(argc, argv, "tfPCL");
    ros::NodeHandle nh_;

    g_list = new tf::TransformListener();
    ros::Subscriber S = nh_.subscribe("scan",1, cb);

    ros::spin();

    delete g_list;
    return 0;
}


void cb(const sensor_msgs::LaserScanConstPtr & ptr){
    std::cout << " Call Back\n";

    size_t size = ptr->ranges.size();

    double d_angle = ptr->angle_increment;

    try{
        g_list->waitForTransform("/map", "/hokuyo", 
        ros::Time(0), ros::Duration(10.0));
        
        g_list->lookupTransform("/map", "/hokuyo",
        ros::Time(0), g_stmp_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    tf::Matrix3x3 mat = g_stmp_transform.getBasis();
    tf::Vector3 origin{g_stmp_transform.getOrigin()};
    std::cout << g_stmp_transform.getOrigin().getX() << " " <<g_stmp_transform.getOrigin().getY() << "\n";
    
    for(int i=0;i<size; i++){
        if(!isinf(ptr->ranges[i])){
            double angle = i * d_angle;
            double x = (ptr->ranges[i]) * cos(angle);
            double y = (ptr->ranges[i]) * sin(angle);           

            RVO::Vector2 point = transformPoint(mat, origin, tf::Vector3(x,y,0));
            std::cout << i << " " <<   point << "   angle= " << angle << "\n";
        }

    }  
    std::cout << "\n Hello World\n"; 
    
}

RVO::Vector2 transformPoint(tf::Matrix3x3& mat, const tf::Vector3& origin  , const tf::Vector3& point){
	return RVO::Vector2
                  (mat.getColumn(0).getX()*point.getX() + mat.getColumn(1).getX()*point.getY() + mat.getColumn(2).getX()*point.getZ() + origin.getX()*1,
				   mat.getColumn(0).getY()*point.getX() + mat.getColumn(1).getY()*point.getY() + mat.getColumn(2).getY()*point.getZ() + origin.getY()*1);
				   //mat.getColumn(0).getZ()*point.getX() + mat.getColumn(1).getZ()*point.getY() + mat.getColumn(2).getZ()*point.getZ() + origin.getZ()*1 );
}
