#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <vector>

ros::Publisher pub_frt;
ros::Publisher pub_rear;

//For my package I dont require intensities. So commented it out.

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg){

    //create sensor messages
	sensor_msgs::LaserScan scan_f;
	sensor_msgs::LaserScan scan_r;
	sensor_msgs::LaserScan scan;

	int Rgn = msg->ranges.size();

	//dividing scan data for front scan
	for(int i=0; i<Rgn/2 ;i++){
		
		scan_f.header.stamp = ros::Time::now();
    	scan_f.header.frame_id = "front_scanner";
    	scan_f.angle_min = 0.0;
    	scan_f.angle_max = M_PI;
		scan_f.angle_increment = msg->angle_increment;
		scan_f.scan_time =msg->scan_time;
		scan_f.time_increment = msg->time_increment;
    	scan_f.range_min = msg->range_min;
        scan_f.range_max = msg->range_max;         
        (scan_f.ranges).push_back(msg->ranges[i]) ;

		// scan_f.intensities = msg->intensities;         

	}

	//dividing scan data for rear scan
	for(int i=0; i<Rgn/2 ;i++){
		
		scan_r.header.stamp = ros::Time::now();
    	scan_r.header.frame_id = "rear_scanner";
        scan_r.angle_min = 0.0;
    	scan_r.angle_max = M_PI;
		scan_r.angle_increment = msg->angle_increment;
		scan_r.scan_time =msg->scan_time;
		scan_r.time_increment = msg->time_increment;
    	scan_r.range_min = msg->range_min;
        scan_r.range_max = msg->range_max;  
		(scan_r.ranges).push_back(msg->ranges[Rgn/2 + i]);

		// scan_r.intensities = msg->intensities;
		
	}


	pub_frt.publish(scan_f);
	pub_rear.publish(scan_r);

}

int main(int argc, char **argv){

	ros::init(argc,argv,"laser_split");
	ros::NodeHandle n;
	ros::Subscriber sub_laser = n.subscribe("/scan", 10, clbk_laser);
	pub_frt = n.advertise<sensor_msgs::LaserScan>("frt_scan",10);
	pub_rear = n.advertise<sensor_msgs::LaserScan>("rr_scan",10);
	ros::spin();
	return 0;
}


