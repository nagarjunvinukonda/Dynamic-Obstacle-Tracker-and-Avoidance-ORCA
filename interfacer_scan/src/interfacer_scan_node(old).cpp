#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <vector>

ros::Publisher pub_frt;
ros::Publisher pub_rear;

void clbk_laser(const sensor_msgs::LaserScan::ConstPtr& msg){

	std::vector<int>rear;
	std::vector<int>front;
	int Rgn = msg->ranges.size();
	for(int i=0; i<Rgn ;i++){
		if(i<Rgn/2 ){           
        		front.push_back(msg->ranges[i]);
		}
		else{
			rear.push_back(msg->ranges[i]);
		}
	}
	std_msgs::Int32MultiArray message;
	message.data = rear;
	pub_frt.publish(message);
	pub_rear.publish(message);

}

int main(int argc, char **argv){

	ros::init(argc,argv,"my_node");
	ros::NodeHandle n;
	ros::Subscriber sub_laser = n.subscribe("/scan", 10, clbk_laser);
	pub_frt = n.advertise<std_msgs::Int32>("frt_scan",10);
	pub_rear = n.advertise<std_msgs::Int32>("rr_scan",10);
	ros::spin();
	return 0;
}

/*	for(int i=0; i<(msg->ranges.size());i++){
		if(!(msg->ranges.empty()))
 		ROS_INFO("data %f",msg->ranges[i]);
		else{
			ROS_INFO("NO VALUE");
		}
	}
	
	int Rgn = msg->ranges.size();
	std::cout<<Rgn;

	for(int i=0; i<(msg->ranges.size());i++){           
        	ROS_INFO("range %lu",msg->ranges.size());
	}
*/
