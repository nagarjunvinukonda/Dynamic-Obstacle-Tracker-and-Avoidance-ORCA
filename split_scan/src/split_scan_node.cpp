#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <vector>

ros::Publisher pub_frt;
ros::Publisher pub_rear;

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg){

	// std::cout << "Hello World1 \n";
	sensor_msgs::LaserScan scan_f;
	sensor_msgs::LaserScan scan_r;
	sensor_msgs::LaserScan scan;

	int Rgn = msg->ranges.size();
  	// double laser_frequency = 40;  // change

	// std::cout << "Hello World2 \n";

    // scan_f.angle_increment = M_PI / (Rgn/2);
    // scan_f.time_increment = (1 / laser_frequency) / (Rgn/2);
    //scan_f._ranges_type(Rgn/2);
    //scan_f.set_intensities_size(Rgn/2);

	// std::cout << "Hello World3 \n";
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
		// std::cout << msg->ranges[i] << "\n";
		// scan_f.intensities = msg->intensities;

	}
	// std::cout << "Hello World4 \n";
	// std::cout<<scan_f.ranges.size();
	for(int i=0; i<Rgn/2 ;i++){
		
		scan_r.header.stamp = ros::Time::now();
    	scan_r.header.frame_id = "rear_scanner";
    	//scan_r.angle_min = M_PI/2;
        scan_r.angle_min = 0.0;
    	scan_r.angle_max = M_PI;
		scan_r.angle_increment = msg->angle_increment;
		scan_r.scan_time =msg->scan_time;
		scan_r.time_increment = msg->time_increment;
    	scan_r.range_min = msg->range_min;
        scan_r.range_max = msg->range_max;  
		(scan_r.ranges).push_back(msg->ranges[Rgn/2 + i]);
		// std::cout << (msg->ranges)[Rgn/2+i] << "\n";
		// scan_r.intensities = msg->intensities;
		
	}

	// std::cout << "Hello World5 \n";
	// std::cout<<scan_r.ranges.size();
	pub_frt.publish(scan_f);
	pub_rear.publish(scan_r);

}

int main(int argc, char **argv){

	ros::init(argc,argv,"my_node");
	ros::NodeHandle n;
	ros::Subscriber sub_laser = n.subscribe("/scan", 10, clbk_laser);
	pub_frt = n.advertise<sensor_msgs::LaserScan>("frt_scan",10);
	pub_rear = n.advertise<sensor_msgs::LaserScan>("rr_scan",10);
	ros::spin();
	return 0;
}


